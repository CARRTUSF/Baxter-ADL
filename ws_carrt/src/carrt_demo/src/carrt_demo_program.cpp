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

    CARRTDemoProgram()
    {
        ros::NodeHandle nh;

        // Create MoveGroup for the Right_Arm
        move_group_.reset(new move_group_interface::MoveGroup("right_arm"));
        move_group_->setPlanningTime(30.0);

        // Let Everything Load
        ros::Duration(1.0).sleep();

        // Enable Baxter
        if(!baxter_util_.enableBaxter())
            return;

        geometry_msgs::PoseStamped ee_pose;
        while(ros::ok()) {
            ee_pose = move_group_->getCurrentPose("right_gripper_l_finger");
            ROS_INFO_STREAM_NAMED("carrt_demo", "Postion (X, Y, Z): (" << ee_pose.pose.position.x << ", " << ee_pose.pose.position.y << ", " << ee_pose.pose.position.z << ")");
            ros::Duration(4.0).sleep();
        }

        // Move to gravity neutral position
        baxter_util_.positionBaxterNeutral();

        // Disable servos
        baxter_util_.disableBaxter();

    }

    bool StartRoutine() {
        while(ros::ok()) {
        }
        //Wait for User Input
        //Tuck Baxter
        //Navigate PowerBot to destination
        //Untuck Baxter
        //Laucnh OPE
        //Navigate to OPE
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
