//ROS
#include <ros/ros.h>

//ROS Transform
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "carrt_kinect_tf_broadcaster");
    ros::NodeHandle node;

    tf::TransformBroadcaster kinect_br_;
    tf::Transform kinect_tf_;

    //Execute Once, the Base does not change position
    tf::Matrix3x3 kinect_matrix;
    //Rotate to 27 Degrees Down
    kinect_matrix.setValue( 0 , -0.3987 ,  0.9171 ,
                           -1 ,  0      ,  0      ,
                            0 , -0.9171 , -0.3987 );

    ros::Rate rate(10.0);
    while (node.ok()) {
        kinect_tf_.setOrigin(tf::Vector3(0.2, 0.0, 0.25));
        //kinect_tf_.setRotation( tf::createQuaternionFromRPY(0.0, 2.04203522, -1.57079633) );
        kinect_tf_.setBasis(kinect_matrix); //Set Rotation Matrix
        kinect_br_.sendTransform(tf::StampedTransform(kinect_tf_, ros::Time::now(), "base", "kinect_head"));
    }

    return 0;
} //main
