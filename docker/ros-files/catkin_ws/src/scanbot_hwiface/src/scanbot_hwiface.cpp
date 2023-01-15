#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

class Scanbot : public hardware_interface::RobotHW
{
private:
    hardware_interface::JointStateInterface wheel_state_interface;
    hardware_interface::VelocityJointInterface wheel_vel_interface;
    double vel_cmd[2];
    double pos[2];
    double vel[2];
    double eff[2];

public:
    Scanbot()
    {
        hardware_interface::JointStateHandle left_wheel_state_handle("left_wheel_revolute", &pos[0], &vel[0], &eff[0]);
        hardware_interface::JointStateHandle right_wheel_state_handle("right_wheel_revolute", &pos[1], &vel[1], &eff[1]);
        wheel_state_interface.registerHandle(left_wheel_state_handle);
        wheel_state_interface.registerHandle(right_wheel_state_handle);
        registerInterface(&wheel_state_interface);

        hardware_interface::JointHandle left_wheel_vel_handle(wheel_state_interface.getHandle("left_wheel_revolute"), &vel_cmd[0]);
        hardware_interface::JointHandle right_wheel_vel_handle(wheel_state_interface.getHandle("right_wheel_revolute"), &vel_cmd[1]);
        wheel_vel_interface.registerHandle(left_wheel_vel_handle);
        wheel_vel_interface.registerHandle(right_wheel_vel_handle);
        registerInterface(&wheel_vel_interface);
    }

    void read()
    {
        ROS_INFO("------------------------");
        ROS_INFO("reading pos/vel/eff from robot");

        /* update those values here ? */

        ROS_INFO("pos[0](left): %.02f", pos[0]);
        ROS_INFO("pos[1](right): %.02f", pos[1]);
        ROS_INFO("vel[0](left): %.02f", vel[0]);
        ROS_INFO("vel[1](right): %.02f", vel[1]);
        ROS_INFO("eff[0](left): %.02f", eff[0]);
        ROS_INFO("eff[1](right): %.02f", eff[1]);
        ROS_INFO("------------------------");

    }

    void write()
    {
        ROS_INFO("------------------------");
        ROS_INFO("writing cmd to robot");

        ROS_INFO("vel_cmd[0](left): %.02f", vel_cmd[0]);
        ROS_INFO("vel_cmd[1](right): %.02f", vel_cmd[1]);

        /* apply those values here ? */
        ROS_INFO("------------------------");
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "scanbot_hwiface");
    Scanbot robot;
    ros::NodeHandle n;
    controller_manager::ControllerManager cm(&robot);
    ros::Time last_t = ros::Time::now();
    ros::Rate rate(10);

    while (true) {
       robot.read();
       cm.update(ros::Time::now(),
                 ros::Time::now() - last_t);
       last_t = ros::Time::now();
       robot.write();

       rate.sleep();
    }
}






//#include <ROBOT_hardware_interface/ROBOT_hardware_interface.h>
//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "ROBOT_hardware_interface");
//    ros::CallbackQueue ros_queue;
//
//    ros::NodeHandle nh;
//    nh.setCallbackQueue(&ros_queue);
//    ROBOT_hardware_interface::ROBOTHardwareInterface rhi(nh);
//
//    ros::MultiThreadedSpinner spinner(0);
//    spinner.spin(&ros_queue);
//    return 0;
//}


//int main(int argc, char **argv) {
//  ros::init(argc, argv, "scanbot_hwiface");
//  ros::NodeHandle n;
//  ros::spin();
//  return 0;
//}


