/*
  Derived from youbot_move_group_tests package
  by Aneesh PA on 20/09/2017

  Objective is to
  1. Verify trajectory before execution
  2. Avoid repeated planning
  3. Smoothen the trajectory by avoiding stops in between
*/

#include<moveit/move_group_interface/move_group.h>
#include<moveit/planning_scene_interface/planning_scene_interface.h>

#include<moveit_msgs/DisplayRobotState.h>
#include<moveit_msgs/DisplayTrajectory.h>

#include<moveit_msgs/CollisionObject.h>
#include<moveit_msgs/AttachedCollisionObject.h>

#include<brics_actuator/JointPositions.h>

#include<string.h>

//void openGripper();
// Actuate gripper to open/hold/release : 0.1/0.05/0.0
void move_gripper(float, float);

// Gripper positions
const float gr_close = 0.0;
const float gr_open = 0.01;
const float gr_hold = 0.005;

// Pause in seconds between target pose and start of next plan
const float t_pause = 1 ;

// loop count for repeating pick_n_place
const int PICK_N_PLACE_LOOP_COUNT = 3;  //

// IMPORTANT
// maxVelocityScalingFactor 0.2;  t_pause 1; is a working combination for this program
const double VELOCITY_SCALING_FACTOR = 0.2;
const double ACCELERATION_SCALING_FACTOR = 0.2;

const unsigned int PLANNING_ATTEMPTS = 2;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "youbot_move_group_tests");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ROS_INFO("Let Rviz come up");
    //   sleep(20.0);

    // Our handle to the world
     moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Publisher for visualizing plans in Rviz; Optional
    ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("move_group/display_planned_path", 1, true);
    //ros::Publisher pub = node_handle.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 1000);
    moveit_msgs::DisplayTrajectory display_trajectory;

    // Declare a planning group
    // Planning groups are stored in an object called JointModelGroup
    static const std::string PLANNING_GROUP = "arm_1";
    moveit::planning_interface::MoveGroup move_group(PLANNING_GROUP);
      
    // We can print the name of the reference frame for this robot.
    ROS_INFO("------PLANNING REFERENCE FRAME-------: %s", move_group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("------End effector link-------: %s", move_group.getEndEffectorLink().c_str());

    // Prints active joints; lu for long unsigned int
    ROS_INFO("Number of Activejoints: %lu", move_group.getActiveJoints().size());

    // Get default planner
    std::string default_planner = move_group.getDefaultPlannerId(PLANNING_GROUP);
    ROS_INFO("------Default planner------: %s", default_planner.c_str());

    // Set planner
    ROS_INFO("--------SETTING PLANNER, PLANNING TIME------");
    move_group.setPlannerId("RRTkConfigDefault");

    // move_group.setPlannerId("PRMkConfigDefault");

    // Planning with collision detection can be slow.  Lets set the planning time
    // to be sure the planner has enough time to plan around the box.  10 seconds
    move_group.setPlanningTime(10.0);

    //get goal position tolerance, goal joint tolerance, goal orientation tolerance
    ROS_INFO("Tolerances: pos, joint and orientation : %f, %f, %f", move_group.getGoalPositionTolerance()
             , move_group.getGoalJointTolerance(), move_group.getGoalOrientationTolerance());

    // Set Velocity within safe limits, better to set it under 0.7
    double maxVelocityScalingFactor = VELOCITY_SCALING_FACTOR;
    move_group.setMaxVelocityScalingFactor(maxVelocityScalingFactor);

    // Get all stored target poses
    std::vector<std::string> targetPoses;
    std::vector<std::string>::iterator it;
    targetPoses = move_group.getNamedTargets();

    for(it=targetPoses.begin(); it<targetPoses.end();it++)
    {
        ROS_INFO("Stored target poses are: %s", it->c_str());  //prints all stored target poses
    }


    // close gripper;
    move_gripper(gr_close, gr_close);

    // Add a collision object to the scene
    moveit_msgs::CollisionObject collision_object;
    collision_object.header.frame_id = move_group.getPlanningFrame();
    // Object id
    collision_object.id = "my_box";

    // Dimensions of the box
    // (0.5,0.3,0.2)approximately the size of mobile base of youbot!
    shape_msgs::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.5; // length along x
    primitive.dimensions[1] = 0.3; // width along y
    primitive.dimensions[2] = 0.3; // height along z

    // Pose for the box relative to frame_id
    // Right hand system; Z thumb; Curl from first axis (x) to second axis (y)
    // Keep the box to right side of the base;

    geometry_msgs::Pose box_pose;
    box_pose.orientation.w = 1.0;
    box_pose.position.x =  0; //0.6;
    box_pose.position.y =  -0.3; // 0.15 + 0.15
    box_pose.position.z =  0.10;  //

    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(box_pose);
    collision_object.operation = collision_object.ADD;

    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(collision_object);

    // Now, let's add the collision object into the world
    ROS_INFO("Add MY_BOX into the world");
    planning_scene_interface.addCollisionObjects(collision_objects);

    /* Sleep so we have time to see the object in RViz */
    sleep(2.0);


    ROS_INFO("---------- TRAJECTORY PLANNING ----------");
    // Move to namedTarget "erect_high"
    bool success;
    /*
    robot_state::RobotStatePtr current_state;
    current_state = move_group.getCurrentState();
    //success = move_group.getCurrentState(&current_state);
    ROS_INFO("------OBTAINED_CURRENT_STATE----- %s", success?"":"FAILED");
    move_group.setStartState(*current_state);
    */
    move_group.setStartStateToCurrentState();
    move_group.setNamedTarget("slide_in");
    moveit::planning_interface::MoveGroup::Plan my_plan;
    success = move_group.plan(my_plan);

    ROS_INFO("Visualizing plan slide_in ");
    display_trajectory.trajectory_start = my_plan.start_state_;
    display_trajectory.trajectory.push_back(my_plan.trajectory_);
    display_publisher.publish(display_trajectory);
    ROS_INFO("Visualizing plan slide_in %s", success?"":"FAILED");
    sleep(10.0);

    std::cout<< "Execute the trajectory (y/n) ?";
    char flag;
    std::cin>>flag;
    if (flag != 'y') {
        std::exit(0);
    }

    // open gripper();
    move_gripper(gr_open, gr_open);

    // holders for saved trajectories
    moveit::planning_interface::MoveGroup::Plan pick_to_place, place_to_pick;

    // move to the target pose
    ROS_INFO("-------ACTUAL EXECUTION STARTS for slide_in --------");
    moveit::planning_interface::MoveItErrorCode Err = move_group.execute(my_plan);//move_group.move();
    ROS_INFO("====EXECUTION DONE for slide_in====: %s",Err.SUCCESS? "":"FAILED");
    sleep(t_pause);
    move_gripper(gr_hold, gr_hold);

    // Place
    move_group.setStartStateToCurrentState();
    move_group.setNamedTarget("stoop_low");
    success = move_group.plan(my_plan);
    ROS_INFO("Visualizing plan stoop_low %s", success?"":"FAILED");
    sleep(t_pause*10.0);

    std::cout<< "Execute and save the trajectory (y/n) ?";
    std::cin>>flag;
    if (flag != 'y') {
        std::exit(0);
    }

    pick_to_place.trajectory_ = my_plan.trajectory_;
    Err = move_group.move();
    ROS_INFO("Executing plan stoop_low %s", Err.SUCCESS?"":"FAILED");

    // release /open gripper
    move_gripper(gr_open, gr_open);
    sleep(2*t_pause);

    move_group.setStartStateToCurrentState();
    move_group.setNamedTarget("slide_in");
    success = move_group.plan(my_plan);
    ROS_INFO("Visualizing plan slide_in %s", success?"":"FAILED");
    sleep(t_pause*10.0);

    std::cout<< "Execute and save the trajectory (y/n) ?";
    std::cin>>flag;
    if (flag != 'y') {
        std::exit(0);
    }

    place_to_pick.trajectory_=my_plan.trajectory_;
    Err = move_group.move();
    ROS_INFO("--------Actual move slide_in-------- %s", Err.SUCCESS?"":"FAILED");

    // hold position for the gripper
    move_gripper(gr_hold, gr_hold);
    sleep(t_pause);


    int loop_count = PICK_N_PLACE_LOOP_COUNT;
    do{

        move_group.setStartStateToCurrentState();
        my_plan.trajectory_=pick_to_place.trajectory_;
        success = move_group.plan(my_plan);
        ROS_INFO("Visualizing pick_to_place trajectory %s", success?"":"FAILED");
        sleep(t_pause*10.0);

        std::cout<< "Execute the trajectory (y/n) ?";
        std::cin>>flag; // = std::getchar();
        if (flag != 'y') {
            std::exit(0);
        }

        Err = move_group.move();
        sleep(t_pause);

        move_group.setStartStateToCurrentState();
        my_plan.trajectory_= place_to_pick.trajectory_;
        success = move_group.plan(my_plan);
        ROS_INFO("Visualizing pick_to_place trajectory %s", success?"":"FAILED");
        sleep(t_pause*10.0);

        std::cout<< "Execute the trajectory (y/n) ?";
        std::cin>>flag; // = std::getchar();
        if (flag != 'y') {
            std::exit(0);
        }

        Err = move_group.move();
        sleep(t_pause);


        //move_group.setStartStatetoCurrentState();
        // ROS_INFO("Activating forward_bend");
        // Move to namedTarget "forward_bend"
        //move_group.setNamedTarget("forward_bend");
        //success = move_group.plan(my_plan);
        //Err = move_group.move();
        //ROS_INFO("Visualizing plan forward_bend %s", Err.SUCCESS?"":"FAILED");
        //sleep(t_pause*10.0);


        //move_group.setStartState(*move_group.getCurrentState());
        //move_group.setNamedTarget("carry_load");
        //success = move_group.plan(my_plan);
        //Err = move_group.move();
        //ROS_INFO("Visualizing plan carry_load %s", success?"":"FAILED");
        //sleep(t_pause);

        //move_group.setStartState(*move_group.getCurrentState());
        //move_group.setNamedTarget("backward_bend");
        //success = move_group.plan(my_plan);
        //Err = move_group.move();
        //ROS_INFO("Visualizing plan backward_bend %s", success?"":"FAILED");
        //sleep(t_pause);


        --loop_count;

//        sleep(2*t_pause);
    } while(loop_count>0);

    // close gripper
    move_gripper(gr_close, gr_close);
    move_group.setStartStateToCurrentState();
    // Move to namedTarget "gazebo_home"
    move_group.setNamedTarget("home_gazebo");
    success = move_group.plan(my_plan);
    sleep(10*t_pause);
    ROS_INFO("Visualizing plan forward_bend %s", success?"":"FAILED");
    Err = move_group.move();
    ROS_INFO("Plan gazebo_home execution %s", Err.SUCCESS? "": "FAILED");

    ros::shutdown();
    return 0;
}

/*
 * TrajectoryJoint controller is not implemented in youbot_driver
 * Youbot gripper uses brics_actuator with position command
 * So, a method for gripper actuation is implemented as below;
 * TO DO: Develop a TrajectoryJoint Controller for gripper
*/

void move_gripper(float l_val, float r_val)
{
    //  ros::init(argc, argv, "node_pubvel1");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<brics_actuator::JointPositions>("/arm_1/gripper_controller/position_command", 1000);

    ROS_INFO("Finger opening pub node created");

    ros::Rate rate(1);
    //ROS_INFO_STREAM("enterring ros::ok loop");

    int count = 0;

    while (ros::ok() && count < 2) {
        brics_actuator::JointPositions msg;
        //ROS_INFO_STREAM("to resize joint names array");
        msg.positions.resize(2);
        msg.positions[0].joint_uri="gripper_finger_joint_r";
        msg.positions[0].unit="m";
        msg.positions[0].value=r_val;

        msg.positions[1].joint_uri="gripper_finger_joint_l";
        msg.positions[1].unit="m";
        msg.positions[1].value=l_val;

        pub.publish(msg);
        //   pub.publish(msg_l);
        ++count;
        //wait until its time
        rate.sleep();
        ROS_INFO_STREAM("Opened Fingers");
     }

 }
