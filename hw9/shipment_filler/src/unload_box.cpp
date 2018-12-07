//unload_box.cpp:
// moves a box under camera, then removes sensed parts from box

//use a RobotBehaviorInterface object to communicate with the robot behavior action server
#include <robot_behavior_interface/RobotBehaviorInterface.h>
#include <osrf_gear/DroneControl.h>	//package for conveyor belt message (found using "rosservice info /ariac/drone". Note: can only be used when the service is running)


#include<bin_inventory/bin_inventory.h>
//we define a message type for a "part" that includes name, pose and associated location code (e.g. bin # or box location)
#include<inventory_msgs/Part.h>

//a "box inspector" object can compare a packing list to a logical camera image to see how we are doing
#include<box_inspector/box_inspector2.h>	//used for when at station 2 (dependence on new robot motions and cam 2 etc)

//conveyor interface communicates with the conveyor action server
#include<conveyor_as/ConveyorInterface.h>

#include <math.h>
#include <std_srvs/Trigger.h>	//package for starting competition message (found using "rosservice info /ariac/start_competition".  Note: can only be used when the service is running)

#include<order_manager/order_manager.h>	//used to get the orders
#include<xform_utils/xform_utils.h>	//used to convert order poses to world coordinates

const double COMPETITION_TIMEOUT=500.0; // need to  know what this is for the finals;
// want to ship out partial credit before time runs out!

bool g_gotNewOrder=false;
osrf_gear::Order g_order;
XformUtils g_xformUtils;

void model_to_part(osrf_gear::Model model, inventory_msgs::Part &part, unsigned short int location) {
    part.name = model.type;
    part.pose.pose = model.pose;
    part.location = location; //by default
}

void order_callback(const osrf_gear::Order::ConstPtr &order_msg){
    	ROS_INFO_STREAM("Received order:\n" << *order_msg);
	//osrf_gear::Order order;
	g_order = *order_msg;
	g_gotNewOrder = true;
}


int main(int argc, char** argv) {
    // ROS set-ups:
    ros::init(argc, argv, "box_unloader"); //node name
    ros::NodeHandle nh; // create a node handle; need to pass this to the class constructor
    int ans;	//used for breakpoint(s) input

    ROS_INFO("instantiating a RobotBehaviorInterface");
    RobotBehaviorInterface robotBehaviorInterface(&nh); //shipmentFiller owns one as well

    ROS_INFO("instantiating a ConveyorInterface");
    ConveyorInterface conveyorInterface(&nh);

    ROS_INFO("instantiating a BoxInspector2 (new one)");
    BoxInspector2 boxInspector2(&nh);

    ROS_INFO("main: instantiating an object of type OrderManager");
    OrderManager orderManager(&nh); //shipmentFiller also owns one of these, which is public

    ROS_INFO("instantiationg a BinInventory object");
    BinInventory binInventory(&nh);  //shipmentFiller owns one of these, which is public

    

	ros::ServiceClient start_competition_client = nh.serviceClient<std_srvs::Trigger>("/ariac/start_competition"); //create client for automatic competition startup
  	ros::Subscriber order_getter_subscriber= nh.subscribe("/ariac/orders",10, &order_callback); 
	ros::ServiceClient drone_client = nh.serviceClient<osrf_gear::DroneControl>("/ariac/drone"); //create client for drone
    //instantiate an object of appropriate data type for our move-part commands
    inventory_msgs::Part current_part;

    inventory_msgs::Part desired_part;

    geometry_msgs::PoseStamped box_pose_wrt_world;  //camera sees box, coordinates are converted to world coords

	std_srvs::Trigger start_competition; //start_competition client object
	osrf_gear::DroneControl droneControl; //droneControl client object

    inventory_msgs::Inventory current_inventory;   

	bool go_on;	//state machine varariable for robot motion

    bool status;    //robot motion bool return variable
    int nparts;

	float min_distance = 10000000; //initialize large value
	int min_i;
    //for box inspector, need to define multiple vectors for args, 
    //box inspector will identify parts and convert their coords to world frame
    //in the present example, desired_models_wrt_world is left empty, so ALL observed parts will be considered "orphaned"
        vector<osrf_gear::Model> desired_models_wrt_world;
        vector<osrf_gear::Model> satisfied_models_wrt_world;
        vector<osrf_gear::Model> misplaced_models_actual_coords_wrt_world;
        vector<osrf_gear::Model> misplaced_models_desired_coords_wrt_world;
        vector<osrf_gear::Model> missing_models_wrt_world;
        vector<osrf_gear::Model> orphan_models_wrt_world;
        vector<int> part_indices_missing;
        vector<int> part_indices_misplaced;
        vector<int> part_indices_precisely_placed;


//initiate the start of the competition

	start_competition.response.success = false; //initialize success field as false
	start_competition_client.call(start_competition); // client sends call to service to allow for a response of true in success field

	while(!start_competition.response.success){ //error check for start_competition
		ROS_WARN("start_competiton not successful");
		start_competition_client.call(start_competition); // put in here again to run it until it calls
		ros::Duration(0.5).sleep();
	}
	ROS_INFO("Start_competition successful");

//update the bin inventory information
    while (!binInventory.update()) {	
	ROS_INFO("Unable to update binInventory");
	}

    ROS_INFO("got initial inventory");
    binInventory.get_inventory(current_inventory);


//move box to Q1
    //use conveyor action  server for multi-tasking
    ROS_INFO("getting a box into position: ");
    int nprint = 0;
    conveyorInterface.move_new_box_to_Q1();  //member function of conveyor interface to move a box to inspection station 1
    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q1) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to Q1...");
        }
    }

    //update box pose,  if possible              
    if (boxInspector2.get_box_pose_wrt_world(box_pose_wrt_world)) {
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
    }
    else {
        ROS_WARN("no box seen.  something is wrong! I quit!!");
        exit(1);
    }
    
    // if survive to here, then box is at Q1 inspection station; 


//obtain the order information

	while(!g_gotNewOrder){	//gets a new order
		ros::spinOnce();
	}
	osrf_gear::Shipment shipment = g_order.shipments[0];



//converts the order to world coordinate reference values
	//wanted to use compute_shipment_poses_wrt_world which was a function in the box_inspector.h but could not be found in the box_inspector.cpp file
	
	Eigen::Affine3d box_wrt_world_Affine, current_pose_wrt_world_Affine, current_pose_wrt_box_Affine;

	for(int i= 0; i < shipment.products.size(); i++){
		geometry_msgs::Pose current_pose = shipment.products[i].pose;

		box_wrt_world_Affine = g_xformUtils.transformPoseToEigenAffine3d(box_pose_wrt_world.pose);
		current_pose_wrt_box_Affine = g_xformUtils.transformPoseToEigenAffine3d(current_pose);
		current_pose_wrt_world_Affine = box_wrt_world_Affine * current_pose_wrt_box_Affine;

		osrf_gear::Model tempModel;
		tempModel.type = shipment.products[i].type;
		tempModel.pose = g_xformUtils.transformEigenAffine3dToPose(current_pose_wrt_world_Affine);
		
		desired_models_wrt_world.push_back(tempModel);  //after converting the shipment info to world frame, store into desired_models vector

	}

    
    //inspect the box and classify all observed parts
    boxInspector2.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);
    ROS_INFO("orphaned parts in box: ");
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num parts seen in box = %d",nparts);
    for (int i=0;i<nparts;i++) {
       ROS_INFO_STREAM("orphaned  parts: "<<orphan_models_wrt_world[i]<<endl);
    }
  

    while (boxInspector2.get_bad_part_Q1(current_part)) { //was originally an if -- really should use part_Q but I already had Q1 and want to remember this
        ROS_INFO("found bad part: ");
        ROS_INFO_STREAM(current_part<<endl);


	        for(int i=0; i< orphan_models_wrt_world.size(); i++){
			float distance = pow((current_part.pose.pose.position.x - orphan_models_wrt_world[i].pose.position.x), 2) +  pow((current_part.pose.pose.position.y - orphan_models_wrt_world[i].pose.position.y), 2);
			if ( distance < min_distance){
				min_distance = distance;
				min_i = i;
			}
		}
	ROS_INFO("Removing Defective Part");
	model_to_part(orphan_models_wrt_world[min_i], current_part, inventory_msgs::Part::QUALITY_SENSOR_1); //fills value in for current part
    	status = robotBehaviorInterface.pick_part_from_box(current_part); //picks up current part
        status = robotBehaviorInterface.discard_grasped_part(current_part); // discards current part


    }    


    //re-inspect the box to make sure there are no more bad parts and nothing else bumped or anything
    boxInspector2.update_inspection(desired_models_wrt_world,
        satisfied_models_wrt_world,misplaced_models_actual_coords_wrt_world,
        misplaced_models_desired_coords_wrt_world,missing_models_wrt_world,
        orphan_models_wrt_world,part_indices_missing,part_indices_misplaced,
        part_indices_precisely_placed);


//loop till no more missing parts are present (this is at station 1)
	while( missing_models_wrt_world.size() > 0){

		int n_missing_part = part_indices_missing[0];

		//model_to_part(desired_models_wrt_world[n_missing_part], current_part, inventory_msgs::Part::QUALITY_SENSOR_2);
		std::string part_name(desired_models_wrt_world[n_missing_part].type);

		ROS_INFO_STREAM("looking for part " << part_name << endl);
		int partnum_in_inventory;
		bool part_in_inventory = true;
		inventory_msgs::Part pick_part, place_part;


		binInventory.update();
		binInventory.get_inventory(current_inventory);
		part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);
		if (!part_in_inventory) {
		    ROS_WARN("could not find desired  part in inventory; giving up on process_part()");
		    return false; //nothing more can be done     
		}
		ROS_INFO_STREAM("found part: " << pick_part << endl);
		//specify place part:
		model_to_part(desired_models_wrt_world[n_missing_part], place_part, inventory_msgs::Part::QUALITY_SENSOR_1);	//note that this location code is at Q1

		go_on = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part, place_part);
		if (!go_on) {
		    ROS_WARN("could not compute key pickup and place poses for this part source and destination");
		}




		if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
		    ROS_INFO("pick failed");
		    go_on = false;
		    return false;
		    //gripperInterface_.release();     
		}

		ROS_INFO("moving to approach pose");
		if (!robotBehaviorInterface.move_part_to_approach_pose(place_part)) {
		    ROS_WARN("could not move to approach pose");
		    go_on = false;
		    robotBehaviorInterface.discard_grasped_part(place_part);
		    //return false;  // REMOVE THIS IF NEEDED
		}
		//place  part:
		if (!robotBehaviorInterface.place_part_in_box_no_release(place_part)) {
		    ROS_INFO("placement failed");
		    go_on = false;
		    return false;
		}
		
		status = robotBehaviorInterface.release_and_retract();

	    boxInspector2.update_inspection(desired_models_wrt_world,
		    satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
		    misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
		    orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
		    part_indices_precisely_placed);		//will decrement as looped through and items are placed will size is zero of missing_models_wrt_world; then the loop will end
	    nparts = orphan_models_wrt_world.size();
	
	}


//move box to station 2
	conveyorInterface.move_box_Q1_to_Q2(); //moves the box from Q1 to Q2 automatically
	while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SEEN_AT_Q2) {
		ros::spinOnce();
		ros::Duration(0.1).sleep();
		nprint++;
		if (nprint % 10 == 0) {
		ROS_INFO("waiting for conveyor to advance a box to Q2...");
		}
	}

    
    //update box pose,  if possible      
    if (boxInspector2.get_box_pose_wrt_world(box_pose_wrt_world, CAM2)) {		//CAM2 is defined in header file
        ROS_INFO_STREAM("box seen at: " << box_pose_wrt_world << endl);
    } else {
        ROS_WARN("no box seen.  something is wrong! I quit!!");
        exit(1);
    }


    boxInspector2.compute_shipment_poses_wrt_world(g_order.shipments[0], box_pose_wrt_world, desired_models_wrt_world);

    //inspect the box and classify all observed parts
    boxInspector2.update_inspection(desired_models_wrt_world,
            satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
            part_indices_precisely_placed, CAM2);
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num orphaned parts seen in box = %d", nparts);
    for (int i = 0; i < nparts; i++) {
        ROS_INFO_STREAM("orphaned  parts: " << orphan_models_wrt_world[i] << endl);
    }


//remove any bad parts at Q2 until there are none
    min_distance = 10000000; //reinitialize min distance

    while (boxInspector2.get_bad_part_Q(current_part, CAM2)) {
        ROS_INFO("found bad part: ");
        ROS_INFO_STREAM(current_part << endl);

         for(int i=0; i< orphan_models_wrt_world.size(); i++){
			float distance = pow((current_part.pose.pose.position.x - orphan_models_wrt_world[i].pose.position.x), 2) +  pow((current_part.pose.pose.position.y - orphan_models_wrt_world[i].pose.position.y), 2);
			if ( distance < min_distance){
				min_distance = distance;
				min_i = i;
			}
		}
	ROS_INFO("Removing Defective Part");
	model_to_part(orphan_models_wrt_world[min_i], current_part, inventory_msgs::Part::QUALITY_SENSOR_2); //fills value in for current part
    	status = robotBehaviorInterface.pick_part_from_box(current_part); //picks up current part
        status = robotBehaviorInterface.discard_grasped_part(current_part); // discards current part


    }

//re update box after removing bad parts
   boxInspector2.update_inspection(desired_models_wrt_world,
            satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
            part_indices_precisely_placed, CAM2);
    nparts = orphan_models_wrt_world.size();
    ROS_INFO("num orphaned parts seen in box = %d", nparts);
    for (int i = 0; i < nparts; i++) {
        ROS_INFO_STREAM("orphaned  parts: " << orphan_models_wrt_world[i] << endl);
    }


//readding missing part (replacing faulty part) -- done first in case this item is not placed in proper tolerances

	while( missing_models_wrt_world.size() > 0){

		int n_missing_part = part_indices_missing[0];

		//model_to_part(desired_models_wrt_world[n_missing_part], current_part, inventory_msgs::Part::QUALITY_SENSOR_2);
		std::string part_name(desired_models_wrt_world[n_missing_part].type);

		ROS_INFO_STREAM("looking for part " << part_name << endl);
		int partnum_in_inventory;
		bool part_in_inventory = true;
		inventory_msgs::Part pick_part, place_part;


		binInventory.update();
		binInventory.get_inventory(current_inventory);
		part_in_inventory = binInventory.find_part(current_inventory, part_name, pick_part, partnum_in_inventory);
		if (!part_in_inventory) {
		    ROS_WARN("could not find desired  part in inventory; giving up on process_part()");
		    return false; //nothing more can be done     
		}
		ROS_INFO_STREAM("found part: " << pick_part << endl);
		//specify place part:
		model_to_part(desired_models_wrt_world[n_missing_part], place_part, inventory_msgs::Part::QUALITY_SENSOR_2);

		go_on = robotBehaviorInterface.evaluate_key_pick_and_place_poses(pick_part, place_part);
		if (!go_on) {
		    ROS_WARN("could not compute key pickup and place poses for this part source and destination");
		}




		if (!robotBehaviorInterface.pick_part_from_bin(pick_part)) {
		    ROS_INFO("pick failed");
		    go_on = false;
		    return false;
		    //gripperInterface_.release();     
		}

		ROS_INFO("moving to approach pose");
		if (!robotBehaviorInterface.move_part_to_approach_pose(place_part)) {
		    ROS_WARN("could not move to approach pose");
		    go_on = false;
		    robotBehaviorInterface.discard_grasped_part(place_part);
		    //return false;  // REMOVE THIS IF NEEDED
		}
		//place  part:
		if (!robotBehaviorInterface.place_part_in_box_no_release(place_part)) {
		    ROS_INFO("placement failed");
		    go_on = false;
		    return false;
		}
		
		status = robotBehaviorInterface.release_and_retract();

	    boxInspector2.update_inspection(desired_models_wrt_world,
		    satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
		    misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
		    orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
		    part_indices_precisely_placed, CAM2);		//will decrement as looped through and items are placed will size is zero of missing_models_wrt_world; then the loop will end
	    nparts = orphan_models_wrt_world.size();
	
	}


    //after replacing the bad part, re-inspect the box:

    ROS_INFO("MOVE PARTS TO DISPLAY CORRECTING ABILITY:");
    cout << "enter 1 to re-inspect: "; //poor-man's breakpoint
    cin>>ans;

    boxInspector2.update_inspection(desired_models_wrt_world,
            satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
            misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
            orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
            part_indices_precisely_placed, CAM2);
    nparts = orphan_models_wrt_world.size();

    int nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();
    ROS_INFO("found %d misplaced parts", nparts_misplaced);
 //   bool go_on = false;
 

		while(nparts_misplaced > 0){	//loop till no more parts are misplaced

			model_to_part(misplaced_models_actual_coords_wrt_world[0], current_part, inventory_msgs::Part::QUALITY_SENSOR_2);
			//adjust_part_location_no_release(Part sourcePart, Part destinationPart, double timeout = MAX_ACTION_SERVER_WAIT_TIME);
			int index_des_part = part_indices_misplaced[0];
			model_to_part(desired_models_wrt_world[index_des_part], desired_part, inventory_msgs::Part::QUALITY_SENSOR_2);
			ROS_INFO("move part from: ");
			ROS_INFO_STREAM(current_part);
			ROS_INFO("move part to: ");
			ROS_INFO_STREAM(desired_part);

			//use the robot action server to grasp part in the box:
			status = robotBehaviorInterface.pick_part_from_box(current_part);

			//following fnc works ONLY if part is already grasped:
			status = robotBehaviorInterface.adjust_part_location_no_release(current_part, desired_part);
			status = robotBehaviorInterface.release_and_retract();


			    boxInspector2.update_inspection(desired_models_wrt_world,
				    satisfied_models_wrt_world, misplaced_models_actual_coords_wrt_world,
				    misplaced_models_desired_coords_wrt_world, missing_models_wrt_world,
				    orphan_models_wrt_world, part_indices_missing, part_indices_misplaced,
				    part_indices_precisely_placed, CAM2);
			    nparts = orphan_models_wrt_world.size();

			    nparts_misplaced = misplaced_models_actual_coords_wrt_world.size();	//this should gradually decrement to zero, when zero, the loop will end
			    ROS_INFO("Now there are %d misplaced parts", nparts_misplaced);
//			    bool go_on = false;
		}


 //move box to drone depot and giving the drone shipping name to successfully allow it to pick up the shipment
    ROS_INFO("advancing box to loading dock for shipment");
    conveyorInterface.move_box_Q2_to_drone_depot(); //member function of conveyor interface to move a box to shipping dock

    while (conveyorInterface.get_box_status() != conveyor_as::conveyorResult::BOX_SENSED_AT_DRONE_DEPOT) {
        ros::spinOnce();
        ros::Duration(0.1).sleep();
        nprint++;
        if (nprint % 10 == 0) {
            ROS_INFO("waiting for conveyor to advance a box to loading dock...");
        }
    }
    ROS_INFO("calling drone");


    droneControl.request.shipment_type = shipment.shipment_type;
    ROS_INFO_STREAM("shipment name: " << shipment.shipment_type << endl);

    droneControl.response.success = false;
    while (!droneControl.response.success) {
        drone_client.call(droneControl);
        ros::Duration(0.5).sleep();
    }


	ROS_INFO(" Objective Complete");	//signify the end and potentially prevents said error noted after the return


            return 0;
    //here's an oddity: this node runs to completion.  But sometimes, Linux complains bitterly about
    // *** Error in `/home/wyatt/ros_ws/devel/lib/shipment_filler/unload_box': corrupted size vs. prev_size: 0x000000000227c7c0 ***
    // don't know why.  But does not seem to matter.  If anyone figures this  out, please let me know.
}
