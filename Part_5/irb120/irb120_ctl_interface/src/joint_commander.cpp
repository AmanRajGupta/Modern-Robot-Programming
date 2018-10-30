//test_traj_sender_irb120.cpp:
//wsn, Dec 2017
//a simple node to illustrate how to publish a trajectory to topic "joint_path_command"
// intended to work with example_robot_interface node, 
// but should work as well with real ROS-Industrial "motion_download_interface" node--
// BUT BE CAREFUL THAT THE TRAJECTORY SENT IS SAFE

//some generically useful stuff to include...
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include <ros/ros.h> //ALWAYS need to include this

//message types used in this example code;  include more message types, as needed
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <trajectory_msgs/JointTrajectory.h>
//#include <trajectory_msgs/JointTrajectoryPoint.h>


int main(int argc, char** argv) 
{
    // ROS set-ups:
    ros::init(argc, argv, "test_traj_sender"); //node name

    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    
    ros::Publisher pub = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);  
    
    //ros::Rate sleep_timer(UPDATE_RATE); //a timer for desired rate to send new traj points as commands
    trajectory_msgs::JointTrajectory new_trajectory; //   trajectory message
    trajectory_msgs::JointTrajectoryPoint trajectory_point0; //points to populate the trajectory message
    trajectory_msgs::JointTrajectoryPoint trajectory_point1;
    trajectory_msgs::JointTrajectoryPoint trajectory_point2,trajectory_point3,trajectory_point4,trajectory_point5,trajectory_point6, trajectory_point7; 
    
    new_trajectory.points.clear(); //fill in all the joint names
    new_trajectory.joint_names.push_back("joint1");
    new_trajectory.joint_names.push_back("joint2");
    new_trajectory.joint_names.push_back("joint3");
    new_trajectory.joint_names.push_back("joint4");
    new_trajectory.joint_names.push_back("joint5");
    new_trajectory.joint_names.push_back("joint6");    

    ros::Rate sleep_timer(1.0); //1Hz update rate
    

    // build an example trajectory:
    trajectory_point0.positions.clear();
    trajectory_point1.positions.clear();    
    trajectory_point2.positions.clear();    
    
    //specify start pose (all zeros) and two points:
    for (int ijnt=0;ijnt<6;ijnt++) {
        trajectory_point0.positions.push_back(0.0); //all zeros
        trajectory_point1.positions.push_back(0.0); // 
        trajectory_point2.positions.push_back(0.0); // stuff in position commands for 6 joints   
        trajectory_point3.positions.push_back(0.0);
        trajectory_point4.positions.push_back(0.0);
        trajectory_point5.positions.push_back(0.0);
        trajectory_point6.positions.push_back(0.0); 
	trajectory_point7.positions.push_back(0.0);
    }
    //specify arrival time (from start of trajectory)
    trajectory_point0.time_from_start = ros::Duration(1.0); //3 second to arrive
    
    //put this point into trajectory message
    new_trajectory.points.clear();
    new_trajectory.header.stamp = ros::Time::now();
    new_trajectory.points.push_back(trajectory_point0); //contains a single point--home
 
    ROS_INFO("sending robot to home position: ");
    int npts = new_trajectory.points.size();
    ROS_INFO("num pts in initial traj = %d",npts);
    for (int i=0;i<3;i++) //send this twice, just to make sure have communication w/ subscriber
    {
        pub.publish(new_trajectory);
            ros::spinOnce();
            ros::Duration(2.0).sleep();
    }
    

     //next trajectory:
    trajectory_point1.time_from_start = ros::Duration(1.0); //1 seconds to arrive, from previous point
    trajectory_point2.time_from_start = ros::Duration(2.0); //1 seconds to arrive, from previous point
    trajectory_point3.time_from_start = ros::Duration(3.0); //1 seconds to arrive, from previous point
    trajectory_point4.time_from_start = ros::Duration(4.0); //1 seconds to arrive, from previous point
    trajectory_point5.time_from_start = ros::Duration(5.0); //1 seconds to arrive, from previous point
    trajectory_point6.time_from_start = ros::Duration(6.0); //1 seconds to arrive, from previous point 
    trajectory_point6.time_from_start = ros::Duration(7.0); 
     new_trajectory.header.stamp = ros::Time::now();



     trajectory_point1.positions[0] = 0.2;//sin_arr[0];
     trajectory_point1.positions[1] = 0;
     trajectory_point1.positions[2] = 0;
     
     trajectory_point2.positions[0] = 0.2;//sin_arr[1];
     trajectory_point2.positions[1] = 0;
     trajectory_point2.positions[2] = -0.2;
     
     trajectory_point3.positions[0] = 0.2;//sin_arr[2];
     trajectory_point3.positions[1] = 0;
     trajectory_point3.positions[2] = -0.2;
     
     trajectory_point4.positions[0] = 0;//sin_arr[3];
     trajectory_point4.positions[1] = 0;
     trajectory_point4.positions[2] = 0;
     
     trajectory_point5.positions[0] = -0.2;//sin_arr[2];
     trajectory_point5.positions[1] = 0;
     trajectory_point5.positions[2] = 0;
     
     trajectory_point6.positions[0] = -0.2;//sin_arr[1];
     trajectory_point6.positions[1] = 0;
     trajectory_point6.positions[2] = -0.2;

     trajectory_point7.positions[0]=0;
     trajectory_point7.positions[1]=0;
     trajectory_point7.positions[2]=0; 
     
     new_trajectory.points.clear();
     new_trajectory.points.push_back(trajectory_point1); // add this trajectory point to the trajectory message
     new_trajectory.points.push_back(trajectory_point2); // append another point
     new_trajectory.points.push_back(trajectory_point3); // append another point
     new_trajectory.points.push_back(trajectory_point4); // append another point
     new_trajectory.points.push_back(trajectory_point5); // append another point
     new_trajectory.points.push_back(trajectory_point6); // append another point
     new_trajectory.points.push_back(trajectory_point7); // append another point
        

     npts = new_trajectory.points.size(); 
     int njnts = new_trajectory.points[0].positions.size();
    ROS_INFO("sending a trajectory with %d poses, each with %d joints ",npts,njnts);
    pub.publish(new_trajectory);
    //don't die yet--make sure message gets received
     for (int i=0;i<3;i++) {
            sleep_timer.sleep();
     }

    return 0;
} 











































//below is a failed code attempt, with joint limit error also




/*
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <math.h>
#include <std_msgs/Bool.h> 
#include <std_msgs/Float32.h>
#include <string>
#include <vector>

using namespace std;





int main(int argc, char **argv) {

ROS_INFO("Before initialization"); //-------

//topic and publisher setup
    ros::init(argc, argv, "joint_commander"); // name of this node will be "joint_commander"
    ros::NodeHandle nh; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = nh.advertise<trajectory_msgs::JointTrajectory>("joint_path_command", 1);
    //"topic1" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    

//instantiating a trajectory message object
    trajectory_msgs::JointTrajectory trajectory; //create a variable of type "Float64" (double in C++ code)
    trajectory_msgs::JointTrajectoryPoint trajectory_point;

//produce joint names for all used joints
	trajectory.points.clear(); //fill in all the joint names
	
	trajectory.joint_names.push_back("joint1"); //append joint names to field called joint names in trajectory msg
	trajectory.joint_names.push_back("joint2");
	trajectory.joint_names.push_back("joint3");
	trajectory.joint_names.push_back("joint4");
	trajectory.joint_names.push_back("joint5");
	trajectory.joint_names.push_back("joint6");


ROS_INFO("After joint name push back");  //-----
//produce enough position array space for all the joints
	int njnts = trajectory.joint_names.size(); // we specified this many joints;  need same size for position and velocity vectors






//variable definitions
	double displacement,velocity; //radians, radians/sec       
        
	//predefine parameters of sinusoidal movement
	double A= 0.5;
	double w=2*3.1415*1.0;  //w=2*pi*f = 2*pi*1 angular frequency
	double dt=0.1; //incrementation time for the position changes
	double timeFromStart=0;
	double angle=0;
	double max_angle = 2*3.1415; //2 pi radian max  -- the trajectory is being planned from 0 to 2*pi radians



//	int i=0; //initialize points to zero index for loop


ROS_INFO("Before for loop");

//loop to create trajectory plan and store into trajectory object
	for (angle=0; angle<=max_angle; angle+=w*dt) { //loop a set of positions from 0 to 2*pi with small increments (increment is 2*pi*0.1)  -- should be 11 loop throughs

		displacement = A*sin(angle); //here we make up a desired trajectory shape: displacement

		ROS_INFO("In for loop A");
		//trajectory.points[i].positions.resize(njnts);
		ROS_INFO("In for loop B");
		timeFromStart+= dt; 
		ROS_INFO("In for loop C");
		//storing the duration into field for all joints -- tells how long the following trajectory position should take.  In this case, 0.1 sec.

			ROS_INFO("In for loop D");

		//running the same trajectories for all joints
		trajectory_point.positions[0] = displacement; // do this for every joint, from 0 through njnts-1
		ROS_INFO("In for loop E");

		trajectory_point.positions[1] = displacement; // do this for every joint, from 0 through njnts-1


		trajectory_point.positions[2] = displacement; // do this for every joint, from 0 through njnts-1


		trajectory_point.positions[3] = displacement; // do this for every joint, from 0 through njnts-1


		trajectory_point.positions[4] = displacement; // do this for every joint, from 0 through njnts-1

	
		trajectory_point.positions[5] = displacement; // do this for every joint, from 0 through njnts-1

		trajectory_point.time_from_start=ros::Duration(timeFromStart);

		trajectory.points.push_back(trajectory_point);
		
	//	i+=1;

    	}


ROS_INFO("Before publish");  //-----

//publish trajectory to joint_path_command topic & we also provide some information on how many points are in our trajectory.
	//int npts = trajectory.points.size();  //we just created this many points in our trajectory message
	//ROS_INFO("populated trajectory with %d points",npts);
	//publishing the trajectory msg
	my_publisher_object.publish(trajectory);


	ros::Rate sleep_timer(1.0); //1Hz update rate

    //don't die yet--make sure message gets received
     for (int i=0;i<3;i++) {
            sleep_timer.sleep();
     }



}

*/




