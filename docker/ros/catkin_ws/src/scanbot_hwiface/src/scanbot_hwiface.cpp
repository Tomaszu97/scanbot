#include <ros/ros.h>
#include <ros/node_handle.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <sstream>

#define ENCODER_PPR 20
#define SCAN_ARRAY_SIZE 180
#define LASER_RANGE_MIN_M 0.2
#define LASER_RANGE_MAX_M 8.0
#define LASER_DEG_MIN 1
#define LASER_DEG_MAX 180
#define LASER_DEG_STEP 1

#define CMD_DELIMITER ":"
#define CMD_PARAM_SEPARATOR_DELIMITER ","
#define CMD_TERMINATOR '#'

#define PI 3.141592
#define DEG_TO_RAD 0.0174532925
#define RAD_TO_DEG 57.2957795

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
        send_drive_cmd(left, right);
    }

    bool get_scan(float *scan_array)
    {
        sprintf(send_buf, "GS#\n");
        send_cmd();

        const bool received = recv_cmd();
        const char *delims = CMD_DELIMITER CMD_PARAM_SEPARATOR_DELIMITER;
        if (received == false) {
            ROS_ERROR("receive failed");
            return false;
        }

        std::string str(recv_buf);
        std::istringstream ss(str);
        std::string token;
        unsigned int i = 0;
        while (std::getline(ss, token, ',')) {
            if (i >= SCAN_ARRAY_SIZE) break;
            token.erase(remove(token.begin(), token.end(), CMD_TERMINATOR), token.end());
            token.erase(remove(token.begin(), token.end(), '\r'), token.end());
            token.erase(remove(token.begin(), token.end(), '\n'), token.end());
            token.erase(remove(token.begin(), token.end(), '\t'), token.end());
            token.erase(remove(token.begin(), token.end(), ' '), token.end());
            if (token.length() == 0) {
                scan_array[i] = 0;
            }
            else {
                scan_array[i] = std::stoi(token);
            }
            i++;
        }

        return true;
    }
};

int main(int argc, char **argv)
{
    /* get commandline parameters */
    if (argc != 5) {
        ROS_ERROR("provide serial port as first argument");
        return EXIT_FAILURE;
    }
    const char *serial_port_filename = argv[1];
    const char *laser_scan_publish_topic = argv[2];
    const char *laser_scan_frame_id = argv[3];
    const char *loop_rate_str = argv[4];
    const int loop_rate_hz = std::stoi(std::string(loop_rate_str));

    /* initialize ros node and get its handle */
    ros::init(argc, argv, "scanbot_hwiface");
    ros::NodeHandle n;

    /* create a laser scan publisher */
    ros::Publisher scan_publisher;
    scan_publisher = n.advertise<sensor_msgs::LaserScan>(laser_scan_publish_topic, 10);
    int last_seq = 0;

    /* instantiate scanbot class */
    Scanbot robot;

    /* init required ros objects */
    controller_manager::ControllerManager cm(&robot);
    ros::AsyncSpinner asyncSpinner(0);
    asyncSpinner.start();

    /* enable scanning and open serial */
    open_serial(serial_port_filename);
    sprintf(send_buf, "SC#\n");
    send_cmd();

    /* prepare to loop */
    flush_serial();
    ros::Time last_t = ros::Time::now();
    ros::Rate rate(loop_rate_hz);
    while (ros::ok()) {
        /* update controller's time delta */
        cm.update(ros::Time::now(),
                  ros::Time::now() - last_t);
        last_t = ros::Time::now();

        /* update robot joint cmds and states */
        robot.update();

        /* fetch and publish scans  */
        float scan_values[SCAN_ARRAY_SIZE] = {0};
        const bool scan_successful = robot.get_scan(scan_values);
        if (scan_successful == true) {
            /* prepare laser scan message */
            ROS_DEBUG("preparing laser scan");
            sensor_msgs::LaserScan laser_scan;
            laser_scan.header.seq = last_seq++;
            laser_scan.header.stamp = last_t;
            laser_scan.header.frame_id = std::string(laser_scan_frame_id);
            laser_scan.angle_min = LASER_DEG_MIN * DEG_TO_RAD;
            laser_scan.angle_max = LASER_DEG_MAX * DEG_TO_RAD;
            laser_scan.angle_increment = LASER_DEG_STEP * DEG_TO_RAD;
            laser_scan.time_increment = 0;
            laser_scan.scan_time = 0;
            laser_scan.range_min = LASER_RANGE_MIN_M;
            laser_scan.range_max = LASER_RANGE_MAX_M;
            for (int i = 0; i < SCAN_ARRAY_SIZE; i++) {
                laser_scan.ranges.push_back(scan_values[i]);
            }
            /* publish  laser scan message */
            ROS_DEBUG("publishing laser scan");
            scan_publisher.publish(laser_scan);
        }
        else {
            ROS_ERROR("scan failed");
        }

        ros::spinOnce();
        rate.sleep();
    }

    /* disable scanning and close serial */
    sprintf(send_buf, "SS#\n");
    send_cmd();
    close_serial();
}
