//program control node for ariac 2018 hw 6
//helpful link with ariac command instructions: http://wiki.ros.org/ariac/2018/Tutorials/GEARInterface
//NOTE: For some reason since I created a new package, I need to do "source devel/setup.bash" before rosrun for it to work or else it wont recognize package.

#include <ros/ros.h>
#include <std_srvs/Trigger.h>	//package for starting competition message (found using "rosservice info /ariac/start_competition".  Note: can only be used when the service is running)
#include <osrf_gear/ConveyorBeltControl.h>	//package for conveyor belt message (found using "rosservice info /ariac/conveyor/control". Note: can only be used when the service is running)
#include <osrf_gear/LogicalCameraImage.h>	//package for logical camera 2 message (found using "rostopic info /ariac/logical_camera_2")
#include <osrf_gear/DroneControl.h>	//package for conveyor belt message (found using "rosservice info /ariac/drone". Note: can only be used when the service is running)
#include <iostream>
#include <string>
using namespace std;



//global variables
//bool g_take_new_snapshot= false;	//initialize snapshot as false
osrf_gear::LogicalCameraImage g_logical_camera_2_data;
float g_z_position_2; //storage variable for box z position with respect to logical camera 2
int g_x=0; //number of times a box has been centered under camera.


void logical_camera_2_callback(const osrf_gear::LogicalCameraImage& message_holder) //parameter passed is the published data to message_holder 
{ 
	g_logical_camera_2_data = message_holder; //gets all parameters of publisher into global variable
	if(message_holder.models.size() >= 1){ //only store if a model is sensed (or else segmentation error)
		g_z_position_2 = message_holder.models[0].pose.position.z;//stores published z value into global variable

		if(g_z_position_2 >= -0.01 && g_z_position_2 <= 0.01){	//check if it is centered under camera approx.
			g_x+=1; //say that another box has been at the center of the camera
		}
	}

} 



int main(int argc, char **argv) {
	ros::init(argc, argv, "program_control"); //node name is program_control
	ros::NodeHandle n;
	ros::ServiceClient start_competition_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition"); //create client for automatic competition startup
	ros::ServiceClient start_conveyor_belt_client = n.serviceClient<osrf_gear::ConveyorBeltControl>("/ariac/conveyor/control"); //create client for automatic conveyor belt startup
	ros::Subscriber my_subscriber_object= n.subscribe("/ariac/logical_camera_2",1,logical_camera_2_callback); //subscribe to data being published by logical camera 2
	ros::ServiceClient start_drone_client = n.serviceClient<osrf_gear::DroneControl>("/ariac/drone"); //create client for automatic conveyor belt startup
	


	std_srvs::Trigger start_competition; //start_competition client object
	osrf_gear::ConveyorBeltControl start_conveyor_belt; //start_conveyor_belt client object
	osrf_gear::DroneControl start_drone; //start_drone client object

//start_competition call

	start_competition.response.success = false; //initialize success field as false
	start_competition_client.call(start_competition); // client sends call to service to allow for a response of true in success field


    while(!start_competition.response.success){ //error check for start_competition
	ROS_WARN("start_competiton not successful");
	start_competition_client.call(start_competition); // put in here again to run it until it calls
	ros::Duration(0.5).sleep();
	}
	ROS_INFO("Start_competition successful");


//start_conveyor_belt call

	start_conveyor_belt.response.success = false; //initialize success field as false
	start_conveyor_belt.request.power = 100;
	start_conveyor_belt_client.call(start_conveyor_belt); // client sends call to service to allow for a response of true in success field



    while(!start_conveyor_belt.response.success){ //error check for start_conveyor_belt
	ROS_WARN("start_conveyor_belt not successful");
	start_conveyor_belt_client.call(start_conveyor_belt); // put in here again to run it until it calls
	ros::Duration(0.5).sleep();
	}
	ROS_INFO("Start_conveyor_belt successful");




//logical camera actions
	ROS_INFO("No object sensed");
	while(g_logical_camera_2_data.models.size() < 1){
		ros::spinOnce(); //go to callback to obtain data (in this case about the model size)
		//ros::Duration(0.5).sleep();
	} //wait for something to be in camera view

	ROS_INFO("Sensed object");
	while(g_logical_camera_2_data.models.size() >= 1 && g_x ==0){	//if the camera does not have a name of a model (in this case a shipping package)
		ros::spinOnce(); //pause node and looks for publish data
		//ros::Duration(0.5).sleep();
	}
	
	ROS_INFO("Getting Ready to stop");
	start_conveyor_belt.response.success= false; //reinitialize conveyor belt response to false for error check later
	start_conveyor_belt.request.power = 0; //stop the conveyor belt
	start_conveyor_belt_client.call(start_conveyor_belt); // client sends call to service to allow for a response of true in success field

	while(!start_conveyor_belt.response.success){ //error check for start_conveyor_belt
		ROS_WARN("stopping of conveyor belt not successful");
		start_conveyor_belt_client.call(start_conveyor_belt); // put in here again to run it until it calls
		ros::Duration(0.1).sleep();
	}
	ROS_INFO("stopping of the conveyor belt successful");


	ROS_INFO("Waiting for 5 seconds");
	ROS_INFO("The box's approximate z value is %f", g_z_position_2); //print out the box position when conveyor is stopped.
	ros::Duration(0.7).sleep(); //pause for 5 seconds -- for some reason duration(5) is not 5 sec pause, so I experimentally got 0.7 to be approx. an actual 5 sec pause


	start_conveyor_belt.response.success= false; //reinitialize conveyor belt response to false for error check later
	start_conveyor_belt.request.power = 100; //restart the conveyor belt
	start_conveyor_belt_client.call(start_conveyor_belt); // client sends call to service to allow for a response of true in success field

	while(!start_conveyor_belt.response.success){ //error check for start_conveyor_belt
		ROS_WARN("restarting of conveyor belt not successful");
		start_conveyor_belt_client.call(start_conveyor_belt); // put in here again to run it until it calls
		ros::Duration(0.1).sleep();
	}
	ROS_INFO("restarting of the conveyor belt successful");




//start_drone call
	
	start_drone.response.success = false; //initialize success field as false
	start_drone.request.shipment_type = "shipping_box_0"; //populate shipment_type string request to send to service
	start_drone_client.call(start_drone); // client sends call to service to allow for a response of true in success field



    while(!start_drone.response.success){ //error check for start_drone
	ROS_WARN("start_drone not successful");
	start_drone_client.call(start_drone); // put in here again to run it until it calls
	ros::Duration(0.1).sleep();
	}
	ROS_INFO("Start_drone successful");




    return 0;
}
