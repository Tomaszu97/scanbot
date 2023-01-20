#include <ros/ros.h>
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <std_msgs/String.h>

#define ENCODER_PPR 20
#define LOOP_RATE_HZ 5
#define SCAN_ARRAY_SIZE 180
#define CMD_DELIMITER ":"
#define CMD_PARAM_SEPARATOR_DELIMITER ","
#define PI 3.141592

extern "C"
{
    #include <string.h>
    #include "serial.c.h"
}


class Scanbot : public hardware_interface::RobotHW
{
private:
    hardware_interface::JointStateInterface state_interface;
    hardware_interface::VelocityJointInterface vel_interface;
    double vel_cmd[3];
    double pos[3];
    double vel[3];
    double eff[3];

public:
    Scanbot()
    {
        ROS_INFO("registering scanbot hardware interfaces");
        hardware_interface::JointStateHandle left_wheel_state_handle("scanbot_left_wheel_revolute", &pos[0], &vel[0], &eff[0]);
        hardware_interface::JointStateHandle right_wheel_state_handle("scanbot_right_wheel_revolute", &pos[1], &vel[1], &eff[1]);
        hardware_interface::JointStateHandle tower_state_handle("scanbot_tower_revolute", &pos[2], &vel[2], &eff[2]);
        state_interface.registerHandle(left_wheel_state_handle);
        state_interface.registerHandle(right_wheel_state_handle);
        state_interface.registerHandle(tower_state_handle);
        registerInterface(&state_interface);

        hardware_interface::JointHandle left_wheel_vel_handle(state_interface.getHandle("scanbot_left_wheel_revolute"), &vel_cmd[0]);
        hardware_interface::JointHandle right_wheel_vel_handle(state_interface.getHandle("scanbot_right_wheel_revolute"), &vel_cmd[1]);
        hardware_interface::JointHandle tower_vel_handle(state_interface.getHandle("scanbot_tower_revolute"), &vel_cmd[2]);
        vel_interface.registerHandle(left_wheel_vel_handle);
        vel_interface.registerHandle(right_wheel_vel_handle);
        vel_interface.registerHandle(tower_vel_handle);
        registerInterface(&vel_interface);
    }

    void send_drive_cmd(int left, int right)
    {
        /* send drive raw command */
        sprintf(send_buf, "DR:%d,%d#\n", left, right);
        send_cmd();

        /* receive encoder data */
        const bool received = recv_cmd();
        char *data = recv_buf;
        const char *delims = CMD_DELIMITER CMD_PARAM_SEPARATOR_DELIMITER;
        if (received == false) {
            ROS_ERROR("receive failed");
            return;
        }
        char *left_encoder_str = strtok(data, delims);
        if (left_encoder_str == NULL) {
            ROS_ERROR("cannot read left encoder");
            return;
        }
        char *right_encoder_str = strtok(NULL, delims);
        if (right_encoder_str == NULL) {
            ROS_ERROR("cannot read right encoder");
            return;
        }

        /* unsafe, but robot has limits anyway */
        const int left_encoder = atoi(left_encoder_str);
        const int right_encoder = atoi(right_encoder_str);

        /* update state position */
        double left_rad = (double)left_encoder * (2 * PI / ENCODER_PPR);
        double right_rad = (double)right_encoder * (2 * PI / ENCODER_PPR);
        if (vel[0] < 0) left_rad *= -1;
        if (vel[1] < 0) right_rad *= -1;
        pos[0] += left_rad;
        pos[1] += right_rad;
        /* update velocity state */
        vel[0] = vel_cmd[0];
        vel[1] = vel_cmd[1];
        /* ignore effort state */
    }

    void update()
    {
        int left = vel_cmd[0]*100;
        int right = vel_cmd[1]*100;
        ROS_INFO("left: %d", left);
        ROS_INFO("right: %d", right);
        send_drive_cmd(left, right);
    }

    bool get_scan(int *scan_array)
    {
        //sprintf(send_buf, "GS#\n");
        //send_cmd();
        /* TODO fetch scan data and fill in scan_array*/
        //SCAN_ARRAY_SIZE
        //const bool received = recv_cmd();
        //char *data = recv_buf;
        //const char *delims = CMD_DELIMITER CMD_PARAM_SEPARATOR_DELIMITER;
        //if (received == false) {
        //    ROS_ERROR("receive failed");
        //    return;
        //}
        //char *left_encoder_str = strtok(data, delims);
        //if (left_encoder_str == NULL) {
        //    ROS_ERROR("cannot read left encoder");
        //    return;
        //}
        //char *right_encoder_str = strtok(NULL, delims);
        //if (right_encoder_str == NULL) {
        //    ROS_ERROR("cannot read right encoder");
        //    return;
        //}

        ///* unsafe, but robot has limits anyway */
        //const int left_encoder = atoi(left_encoder_str);
        //const int right_encoder = atoi(right_encoder_str);

        ///* update state position */
        //pos[0] += (double)left_encoder * (PI / ENCODER_PPR);
        //pos[1] += (double)right_encoder * (PI / ENCODER_PPR);
        ///* ignore velocity state */
        ///* ignore effort state */
        return false;
    }
};

int main(int argc, char **argv)
{
    /* arg1 is serial port filename */
    /* arg2 is topic to publish on */
    if (argc != 3) {
        ROS_ERROR("provide serial port as first argument");
        return EXIT_FAILURE;
    }

    ros::init(argc, argv, "scanbot_hwiface");

    ros::NodeHandle n;

    ros::Publisher scan_publisher;
    scan_publisher = n.advertise<std_msgs::String>(argv[2], 10);

    Scanbot robot;

    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner asyncSpinner(0);
    asyncSpinner.start();

    ros::Time last_t = ros::Time::now();
    ros::Rate rate(LOOP_RATE_HZ);

    const char *serial_filename = argv[1];
    open_serial(serial_filename);

    ///* enable scanning */
    //sprintf(send_buf, "SC#\n");
    //send_cmd();

    flush_serial();

    while (ros::ok()) {
        /* update controller time */
        cm.update(ros::Time::now(),
                  ros::Time::now() - last_t);
        last_t = ros::Time::now();

        /* update robot joint cmds and states */
        robot.update();

        /* fetch and publish scans  */
        static int scan_values[SCAN_ARRAY_SIZE];
        const bool scan_successful = robot.get_scan(scan_values);
        if (scan_successful == true) {
            /*TODO*/
            std_msgs::String str;
            str.data = "hello world";
            scan_publisher.publish(str);
        }
        else ROS_ERROR("scan failed");

        ros::spinOnce();
        rate.sleep();
    }

    close_serial();
}
