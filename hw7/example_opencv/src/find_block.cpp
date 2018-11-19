//get images from topic "simple_camera/image_raw"; remap, as desired;
//search for red pixels;
// convert (sufficiently) red pixels to white, all other pixels black
// compute centroid of red pixels and display as a blue square
// publish result of processed image on topic "/image_converter/output_video"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <xform_utils/xform_utils.h>
//added in below for linear algebra
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>

#define PI  3.141592653589793238462643383280    /* pi */




static const std::string OPENCV_WINDOW = "Open-CV display window";
using namespace std;

int g_redratio; //threshold to decide if a pixel qualifies as dominantly "red"

const double BLOCK_HEIGHT=0.035; // hard-coded top surface of block relative to world frame

class ImageConverter {
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    ros::Publisher block_pose_publisher_; // = n.advertise<std_msgs::Float64>("topic1", 1);
    geometry_msgs::PoseStamped block_pose_;
    XformUtils xformUtils;

public:

    ImageConverter(ros::NodeHandle &nodehandle)
    : it_(nh_) {
        // Subscribe to input video feed and publish output video feed
        image_sub_ = it_.subscribe("simple_camera/image_raw", 1,
                &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        block_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("block_pose", 1, true); 
        block_pose_.header.frame_id = "world"; //specify the  block pose in world coords
        block_pose_.pose.position.z = BLOCK_HEIGHT;
        block_pose_.pose.position.x = 0.5; //not true, but legal
        block_pose_.pose.position.y = 0.0; //not true, but legal
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(0); //not true, but legal
        
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter() {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    //image comes in as a ROS message, but gets converted to an OpenCV type
    void imageCb(const sensor_msgs::ImageConstPtr& msg); 
    
}; //end of class definition

void ImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg){
        cv_bridge::CvImagePtr cv_ptr; //OpenCV data type
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // look for red pixels; turn all other pixels black, and turn red pixels white
        int npix = 0; //count the red pixels
        int isum = 0; //accumulate the column values of red pixels
        int jsum = 0; //accumulate the row values of red pixels
//--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	int total_pixels= 307200; //640 * 480
	float i_arr[total_pixels];	//initialize global variable for storing i values of white pixels -- pixel matrix of camera is 640 x 480 so i have more than enough space in array
	float j_arr[total_pixels];	//initialize global variable for storing j values of white pixels -- pixel matrix of camera is 640 x 480 so i have more than enough space in array
	for (int n=0; n<total_pixels; n++){
		i_arr[n]= -700;	//initialize i and j array values to -700 (an impossible value)
		j_arr[n]= -700;
	}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
        int redval, blueval, greenval, testval;
        cv::Vec3b rgbpix; // OpenCV representation of an RGB pixel
        //comb through all pixels (j,i)= (row,col)
        for (int i = 0; i < cv_ptr->image.cols; i++) {
            for (int j = 0; j < cv_ptr->image.rows; j++) {
                rgbpix = cv_ptr->image.at<cv::Vec3b>(j, i); //extract an RGB pixel
                //examine intensity of R, G and B components (0 to 255)
                redval = rgbpix[2] + 1; //add 1, to avoid divide by zero
                blueval = rgbpix[0] + 1;
                greenval = rgbpix[1] + 1;
                //look for red values that are large compared to blue+green
                testval = redval / (blueval + greenval);
                //if red (enough), paint this white:
                if (testval > g_redratio) {
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 255;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 255;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
		    i_arr[npix]= i - 320; //store i value into array
		    j_arr[npix]= 240 - j;//store respective j value into array
//-------------------------------------------------------------------------------------------------------------------------------------------------------------
                    npix++; //note that found another red pixel
                    isum += i; //accumulate row and col index vals
                    jsum += j;
                } else { //else paint it black
                    cv_ptr->image.at<cv::Vec3b>(j, i)[0] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[1] = 0;
                    cv_ptr->image.at<cv::Vec3b>(j, i)[2] = 0;
                }
            }
        }
        //cout << "npix: " << npix << endl;
        //paint in a blue square at the centroid:
        int half_box = 5; // choose size of box to paint
        int i_centroid, j_centroid;
        double x_centroid, y_centroid;
        if (npix > 0) {
            i_centroid = isum / npix; // average value of u component of red pixels
            j_centroid = jsum / npix; // avg v component
            x_centroid = ((double) isum)/((double) npix); //floating-pt version
            y_centroid = ((double) jsum)/((double) npix);
            ROS_INFO("u_avg: %f; v_avg: %f",x_centroid,y_centroid);
            //cout << "i_avg: " << i_centroid << endl; //i,j centroid of red pixels
            //cout << "j_avg: " << j_centroid << endl;
            for (int i_box = i_centroid - half_box; i_box <= i_centroid + half_box; i_box++) {
                for (int j_box = j_centroid - half_box; j_box <= j_centroid + half_box; j_box++) {
                    //make sure indices fit within the image 
                    if ((i_box >= 0)&&(j_box >= 0)&&(i_box < cv_ptr->image.cols)&&(j_box < cv_ptr->image.rows)) {
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[0] = 255; //(255,0,0) is pure blue
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[1] = 0;
                        cv_ptr->image.at<cv::Vec3b>(j_box, i_box)[2] = 0;
                    }
                }
            }

        }


        // Update GUI Window; this will display processed images on the open-cv viewer.
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3); //need waitKey call to update OpenCV image window

        // Also, publish the processed image as a ROS message on a ROS topic
        // can view this stream in ROS with: 
        //rosrun image_view image_view image:=/image_converter/output_video
        image_pub_.publish(cv_ptr->toImageMsg());

//----------------------------------------------------------------------------------------------------------------------------------------------------------------------

	float j_max= -700;//initialize to impossible value (basing max off of j value)
	float i_val_max=-700; //corresponding i value to max j

	float i_max = -700; //initialize to impossible value (basing max off of i value)
	float j_val_max= -700;//corresponding j value to max i

	float i_min = 700;  //initialize to impossible value (basing min off of i value) -- recognize 640 x 480 pixel camera
	float j_val_min = 700; //corresponding j value to min i
	

	if (npix >0){
		
		for (int count =0; count < total_pixels; count++){
			if (j_max<j_arr[count]){ //finding j_max point
				j_max = j_arr[count];
				i_val_max = i_arr[count]; //note that the max will always be a corner even if it is aligned due to the way that the scanner sifts through the pixels
			}
			if (i_max<i_arr[count]){//finding i_max point
				i_max = i_arr[count];
				j_val_max = j_arr[count];	//note that the max will always be a corner even if it is aligned due to the way that the scanner sifts through the pixels

			}
			if ( (i_min > i_arr[count]) && i_arr[count] != -700 ){ //finding i_min point, recall -700 is what the unfilled array values were initialized to
				i_min = i_arr[count];
				j_val_min = j_arr[count];	//note that the max will always be a corner even if it is aligned due to the way that the scanner sifts through the pixels
			}

		} //note this may have trouble is the whole object is not in the camera's view!!

	}else{
		ROS_INFO("Object not in sight"); //may not be necessary
	}


	float perpendicular_slope;
	float actual_slope;
	float angle;
	int sit;
	if( j_val_min == j_val_max){
		angle=PI/2; //alligned straight up with camera -- operpendicular to x axis
		sit = 1;
	}
	else if( j_max == j_val_max){
		angle= 0;  //alligned perpendicular with camera -- alligned parallel to x- axis
		sit = 2;
	}
	else if(j_val_max > (240 - j_centroid)){
		perpendicular_slope=(j_max - j_val_max)/(i_val_max - i_max);
		actual_slope= -1*(1/perpendicular_slope);
		angle=atan(actual_slope);
		sit = 3;
		}
	else if(j_val_max < (240 - j_centroid)){
		perpendicular_slope=(j_max - j_val_min)/(i_val_max - i_min);
		actual_slope= -1*(1/perpendicular_slope);
		angle=atan(actual_slope);
		sit = 4;
		}



//what I added in ---------------------------------------------------------------------------------------------------------------------------
	Eigen::Matrix3f rot_roll, rot_pitch,rot_yaw, rot_total; //matricies of 3x3 of type float
	
	float roll=0; //in radians
	float pitch =0; //in radians
	float yaw= -0.2; //in radians -- experimentally obtained by rotating object

	rot_roll.row(0) << 1, 0, 0;
	rot_roll.row(1) << 0, cos(roll), -1*sin(roll);
	rot_roll.row(2) << 0,sin(roll), cos(roll);


	rot_pitch.row(0) << cos(pitch), 0, sin(pitch);
	rot_pitch.row(1) << 0,1,0;
	rot_pitch.row(2) << -1*sin(pitch), 0, cos(pitch);


	rot_yaw.row(0) << cos(yaw), -1*sin(yaw), 0;
	rot_yaw.row(1) << sin(yaw), cos(yaw), 0;
	rot_yaw.row(2) << 0,0,1;

	
	rot_total = rot_yaw * rot_pitch * rot_roll; //trivial since only a yaw is needed to transform x and y parallel (z-doesnt matter so it can be 180 deg off)

	//NOTE: Vector is a column vector, must use RowVector for a row vector
	Eigen::Vector3f displacement(0.543,0.321,0); //vector 3x1 (x,y,z) of type float
	Eigen::RowVector4f H_convertor_den(0,0,0,1); //vector 1x4 of type float that holds zeros and a 1
//NOTE: H_convertor is actually in this new case camera_in_robot_reference (technically world reference but world and robot have same origin)

	Eigen::MatrixXf H_convertor_num(rot_total.rows(), (rot_total.cols() + displacement.cols())); //X used since I am not fully sure that the size of the matrix is 2x2 since it holds other matricies/vectors.  Here, I initialize the numerator for concatenation of desired matrices/vectors horizontally.

	H_convertor_num << rot_total, displacement; //fill in horizontal concatenation for H_convertor numerator

	Eigen::MatrixXf H_convertor((H_convertor_num.rows() + H_convertor_den.rows()), H_convertor_num.cols()); //X used since I am not fully sure that the size of the matrix is 2x2 since it holds other matricies.vectors.  Here, I initialize the denominator for concatenation of desired matrices/vectors vertically.
	
	H_convertor << H_convertor_num, H_convertor_den; //fill in vertical concatenation for H_convertor
	
	//storing values into positions for conversion matrix that turns camera coordinates into robot coordinates



	float meters_over_pixels = 2.9 * pow(10,-3); //units of meters/pixels (will convert data from camera from pixels to meters) -- experimentally obtained by moving object and seeing centroid location change in pixels and in meters

	float x_converted = meters_over_pixels * (x_centroid - 320); //converts x value from pixels to meters, normalizing 0,0 to center of camera
	float y_converted = meters_over_pixels * (240 - y_centroid); //converts y value from pixels to meters, normalizing 0,0 to center of camera
	//used x_centroid and y_centroid rather than i_centroid and j_centroid since the prior are doubles whereas the former are ints, so the values are more slightly more accurate



	Eigen::RowVector4f object_in_camera_reference_den(0,0,0,1); //vector 1x4 of type float that holds zeros and a 1

	//NOTE: Vector is a column vector
	Eigen::Vector3f object_coordinates_camera(x_converted,y_converted,0); //inserting the vision obtained centroid of object data (in pixels) with respect to camera (x,y,z,w) (NOTE: Vector is a

	Eigen::Matrix3f rot_object, rot_roll_obj, rot_pitch_obj, rot_yaw_obj; 
	float roll_obj= 0;
	float pitch_obj=0;
	float yaw_obj= angle;

	rot_roll_obj.row(0) << 1, 0, 0;
	rot_roll_obj.row(1) << 0, cos(roll_obj), -1*sin(roll_obj);
	rot_roll_obj.row(2) << 0,sin(roll_obj), cos(roll_obj);


	rot_pitch_obj.row(0) << cos(pitch_obj), 0, sin(pitch_obj);
	rot_pitch_obj.row(1) << 0,1,0;
	rot_pitch_obj.row(2) << -1*sin(pitch_obj), 0, cos(pitch_obj);


	rot_yaw_obj.row(0) << cos(yaw_obj), -1*sin(yaw_obj), 0;
	rot_yaw_obj.row(1) << sin(yaw_obj), cos(yaw_obj), 0;
	rot_yaw_obj.row(2) << 0,0,1;


	rot_object = rot_yaw_obj * rot_pitch_obj * rot_roll_obj;


/*
	//it will be unknown so put in identity matrix
	rot_object.row(0) << 1,0,0;
	rot_object.row(1) << 0,1,0;
	rot_object.row(2) << 0,0,1;
*/
	Eigen::MatrixXf object_in_camera_reference_num(rot_object.rows(), (rot_object.cols() + object_coordinates_camera.cols())); //initialize object in camera's reference frame numerator
	object_in_camera_reference_num << rot_object, object_coordinates_camera; //fill the numerator with values
	

	Eigen::MatrixXf object_in_camera_reference((object_in_camera_reference_den.rows() + object_in_camera_reference_num.rows()), object_in_camera_reference_den.cols()); //initialize object in camera's reference frame
	object_in_camera_reference << object_in_camera_reference_num, object_in_camera_reference_den;

	//NOTE: Vector is a column vector
	Eigen::MatrixXf object_in_robot_reference; //initialize matrix of type float (no specified dimensions) to store calculated/transformed robot coordinates (NOTE: Vector is a column vector)

	


	//equation/transform to solve for object coordinates with respect to the robot
	object_in_robot_reference = H_convertor * object_in_camera_reference; //4x4 output transform

	float x_coordinate_robot = object_in_robot_reference(0,3); //gets x displacement of object in reference to robot
	float y_coordinate_robot = object_in_robot_reference(1,3); //gets y displacement of object in reference to robot
	float angle_of_object_robot = atan(object_in_robot_reference(1,0)/object_in_robot_reference(0,0)); //objects angle with respect to the robot -- yaw


	ROS_INFO("Robot Coordinates of block:	x: %f	y: %f theta: %f ", x_coordinate_robot, y_coordinate_robot, angle_of_object_robot*180/PI);
//	ROS_INFO("Robot Coordinates of block:	x: %f	y: %f theta: %f  max_y: (%f,%f)  max_x: (%f,%f)   min_x: (%f,%f)", x_coordinate_robot, y_coordinate_robot, angle*180/PI, i_val_max, j_max, i_max, j_val_max, i_min, j_val_min);
//	ROS_INFO("perpendicular slope: %f	actual slope: %f	angle: %f	case: %d", perpendicular_slope, actual_slope, angle_of_object_robot*180/PI, sit );
 	ros::Duration(1.0).sleep();

        block_pose_.pose.position.x = x_coordinate_robot;//(what was originally here) i_centroid; //not true, but legal
        block_pose_.pose.position.y = y_coordinate_robot;//(What was originally here) j_centroid; //not true, but legal
	//float theta=0;
        
        // need camera info to fill in x,y,and orientation x,y,z,w
        //geometry_msgs::Quaternion quat_est
        //quat_est = xformUtils.convertPlanarPsi2Quaternion(yaw_est);
        block_pose_.pose.orientation = xformUtils.convertPlanarPsi2Quaternion(angle_of_object_robot); //theta //not true, but legal
        block_pose_publisher_.publish(block_pose_);
    }

int main(int argc, char** argv) {
    ros::init(argc, argv, "red_pixel_finder");
    ros::NodeHandle n; //        
    ImageConverter ic(n); // instantiate object of class ImageConverter
    //cout << "enter red ratio threshold: (e.g. 10) ";
    //cin >> g_redratio;
    g_redratio= 10; //choose a threshold to define what is "red" enough
    ros::Duration timer(0.1);
    double x, y, z;
    while (ros::ok()) {
        ros::spinOnce();
        timer.sleep();
    }
    return 0;
}
