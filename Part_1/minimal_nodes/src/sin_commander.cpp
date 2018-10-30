#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <math.h>
#include <iostream>

#define PI 3.14159265

using namespace std;

int main(int argc, char **argv) {
    ros::init(argc, argv, "sin_commander"); // name of this node will be "sin_commander"
    ros::NodeHandle n; // two lines to create a publisher object that can talk to ROS
    ros::Publisher my_publisher_object = n.advertise<std_msgs::Float64>("vel_cmd", 1);
    //"velocity" is the name of the topic to which we will publish
    // the "1" argument says to use a buffer size of 1; could make larger, if expect network backups
    

	double t=0.0;
	double A;
	double f;
	std_msgs::Float64 velocity; //create a variable of type "Float64", 
    // as defined in: /opt/ros/indigo/share/std_msgs
    // any message published on a ROS topic must have a pre-defined format, 
    // so subscribers know how to interpret the serialized data transmission

	//velocity.data = 0.0;
    
	ROS_INFO("Please enter an Amplitude:\n");
	cin>>A;
	ROS_INFO("Please enter a frequency:\n");
	cin>>f;

	ros::Rate naptime(f*20);

    // do work here in infinite loop (desired for this example), but terminate if detect ROS has faulted
    while (ros::ok()) 
    {
        // this loop has no sleep timer, and thus it will consume excessive CPU time
        // expect one core to be 100% dedicated (wastefully) to this small task
        velocity.data = A*sin(2*PI*f*t); //make velocity a sin wave of input characteristis

        my_publisher_object.publish(velocity); // publish the value--of type Float64 -- velocity 
        //to the topic "velocity"
	
	t += 1.0/(20*f);		//use .data to access the value in message type object
	
	naptime.sleep(); 
    }
}



// prompt for amplitude and frequency

//ROSINFO("Please enter an Amplitude:\n");
//cin>>A;
//ROSINFO("Please enter a frequency:\n");
//cin>>f;

// command sinusoidal velocities to minimal controller  (write and publish a sine equation to min_con)
//velocity = A*sin(2*PI*f*t);



