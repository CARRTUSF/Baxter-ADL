// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

#include <geometry_msgs/PoseStamped.h>

// Baxter Utilities
#include <baxter_control/baxter_utilities.h>

namespace carrt_demo
{

class CARRTDemoProgram
{
public:
    // Our Interface with MoveIt
    boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

    // Baxter Helper
    baxter_control::BaxterUtilities baxter_util_;

    //NodeHandle
    ros::NodeHandle nh;

    CARRTDemoProgram()
    {
        // Create MoveGroup for the Right_Arm
        move_group_.reset(new move_group_interface::MoveGroup("right_arm"));
        move_group_->setPlanningTime(30.0);

        // Let Everything Load
        ros::Duration(1.0).sleep();

        // Enable Baxter
        if(!baxter_util_.enableBaxter())
            return;

        StartRoutine();

        //geometry_msgs::PoseStamped ee_pose;
        //while(ros::ok()) {
        //    ee_pose = move_group_->getCurrentPose("right_gripper_l_finger");
        //    ROS_INFO_STREAM_NAMED("carrt_demo", "Postion (X, Y, Z): (" << ee_pose.pose.position.x << ", " << ee_pose.pose.position.y << ", " << ee_pose.pose.position.z << ")");
        //    ros::Duration(4.0).sleep();
        //}

        // Move to gravity neutral position
        baxter_util_.positionBaxterNeutral();

        // Disable servos
        baxter_util_.disableBaxter();

    }

    bool StartRoutine() {
        while(ros::ok()) {
            geometry_msgs::Pose pos;
            pos.position.x = 1;
            pos.position.y = 1;
            pos.position.z = 1;
            pos.orientation.x = 0;
            pos.orientation.y = 0;
            pos.orientation.z = 0;
            pos.orientation.w = 0;
            move_group_->setRandomTarget();
            move_group_->setStartStateToCurrentState();
            moveit::planning_interface::MoveGroup::Plan plan;
            if(move_group_->plan(plan)) {
              ROS_INFO("Planning Successful!");
              move_group_->execute(plan);
            }
            else {
              ROS_WARN("Planned failed! Not moving");
            }
        }
    }
};

} //namespace

int main(int argc, char **argv) {
    //Start our ROS Node with an Async Thread Spinner (1 Thread)
    ros::init(argc, argv, "carrt_demo_program");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    carrt_demo::CARRTDemoProgram();

    ros::shutdown();

    return 0;
} //main
