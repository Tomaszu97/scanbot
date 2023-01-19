#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>

extern "C"
{
    #include "serial.c.h"
}


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
        ROS_INFO("registering scanbot hardware interfaces");
        hardware_interface::JointStateHandle left_wheel_state_handle("scanbot_left_wheel_revolute", &pos[0], &vel[0], &eff[0]);
        hardware_interface::JointStateHandle right_wheel_state_handle("scanbot_right_wheel_revolute", &pos[1], &vel[1], &eff[1]);
        wheel_state_interface.registerHandle(left_wheel_state_handle);
        wheel_state_interface.registerHandle(right_wheel_state_handle);
        registerInterface(&wheel_state_interface);

        hardware_interface::JointHandle left_wheel_vel_handle(wheel_state_interface.getHandle("scanbot_left_wheel_revolute"), &vel_cmd[0]);
        hardware_interface::JointHandle right_wheel_vel_handle(wheel_state_interface.getHandle("scanbot_right_wheel_revolute"), &vel_cmd[1]);
        wheel_vel_interface.registerHandle(left_wheel_vel_handle);
        wheel_vel_interface.registerHandle(right_wheel_vel_handle);
        registerInterface(&wheel_vel_interface);
    }

    void send_drive_cmd(int left, int right)
    {
        sprintf(send_buf, "DR:%d,%d#\n", left, right);
        send_cmd();
        /* TODO according to previous drive directions and received encoder clicks update odometry pos[] buffer */
        recv_cmd();
    }

    void hw_read()
    {
        /* copy velocity command to state */
        vel[0] = vel_cmd[0];
        vel[1] = vel_cmd[1];

        /* update state position */
        pos[0] += 0.1;
        pos[1] += 0.1;

        /* ignore effort state */

    }

    void hw_write()
    {
        int left = vel_cmd[0]*100;
        int right = vel_cmd[1]*100;
        left *= -1;
        right *= -1;
        ROS_INFO("left: %d", left);
        ROS_INFO("right: %d", right);
        /* not sure why its reversed */
        send_drive_cmd(right, left);
    }

};

int main(int argc, char **argv)
{
    /* arg1 is serial port filename */
    if (argc != 2) {
        ROS_ERROR("provide serial port as first argument");
        return EXIT_FAILURE;
    }

    ros::init(argc, argv, "scanbot_hwiface");
    ros::NodeHandle n;
    Scanbot robot;
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner asyncSpinner(0);
    asyncSpinner.start();

    ros::Time last_t = ros::Time::now();
    ros::Rate rate(5);

    const char *serial_filename = argv[1];
    open_serial(serial_filename);

    while (ros::ok()) {
       flush_serial();
       robot.hw_read();

       cm.update(ros::Time::now(),
                 ros::Time::now() - last_t);
       last_t = ros::Time::now();

       robot.hw_write();

       ros::spinOnce();
       rate.sleep();
    }

    close_serial();
}
