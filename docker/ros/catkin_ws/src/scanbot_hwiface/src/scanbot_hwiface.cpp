#include <ros/ros.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <sstream>
#include <vector>
#include <algorithm>
#include <async_comm/serial.h>

#define SCAN_MIN_DEG 0
#define SCAN_MAX_DEG 179
#define SCAN_BUF_LEN ( SCAN_MAX_DEG - SCAN_MIN_DEG + 1)
#define ENCODER_PPR 20
#define LASER_DEG_STEP 1
#define LASER_RANGE_MIN_M 0.2
#define LASER_RANGE_MAX_M 2.5
#define LASER_DUMMY_VAL 0
#define LASER_UNITS_PER_METER 100
#define LASER_DEFAULT_INTENSITY 0

#define CMD_DELIM ":"
#define CMD_PARAM_DELIM ","
#define CMD_TERM "#"
#define CMD_LEGAL_CHARSET " _-abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789" CMD_DELIM CMD_PARAM_DELIM CMD_TERM
#define CMD_RECEIVE_BUF_LEN 4096
#define CMD_SEND_BUF_LEN 1024
#define CMD_PING "PG"
#define CMD_START_SCAN "SC"
#define CMD_STOP_SCAN "SS"
#define CMD_DRIVE_RAW "DR"
#define CMD_NOTIFY_SCAN "NS"
#define CMD_NOTIFY_SCAN_ARGC SCAN_BUF_LEN
#define CMD_NOTIFY_ENCODERS "NE"
#define CMD_NOTIFY_ENCODERS_ARGC 2

#define PI 3.14159265358979323846
#define DEG_TO_RAD ( PI / 180 )
#define RAD_TO_DEG ( 180 / PI )

class
Scanbot :
    public hardware_interface::RobotHW
{
private:
    /* ros */
    ros::NodeHandle node_handle;
    ros::Publisher scan_publisher;
    std::string laser_scan_frame_id;
    hardware_interface::JointStateInterface state_interface;
    hardware_interface::VelocityJointInterface vel_interface;
    double vel_cmd[3];
    double pos[3];
    double vel[3];
    double eff[3];

    /* serial connection */
    async_comm::Serial *serial;
    uint8_t send_buffer[CMD_SEND_BUF_LEN] = { 0 };
    std::vector<uint8_t> receive_buffer;

public:
    Scanbot(std::string laser_scan_publish_topic,
            std::string laser_scan_frame_id_,
            std::string serial_port,
            const unsigned int baud_rate)
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

        ROS_INFO("initializing serial connection on %s with baud %d", serial_port.c_str(), baud_rate);
        serial = new async_comm::Serial(serial_port,
                                        baud_rate);
        if (!serial->init())
        {
            ROS_INFO("serial initialization failed");
            std::exit(-2);
        }
        serial->register_receive_callback(std::bind(&Scanbot::serial_callback, this, std::placeholders::_1, std::placeholders::_2));

        ROS_INFO("creating laser scan publisher");
        scan_publisher = node_handle.advertise<sensor_msgs::LaserScan>(laser_scan_publish_topic, 10);
        laser_scan_frame_id = laser_scan_frame_id_;

        send_start_scan_cmd();
    }

    ~Scanbot()
    {
        send_stop_scan_cmd();

        ROS_INFO("deinitializing serial");
        serial->close();
        delete serial;
    }

    void
    send_drive_cmd(int left,
                   int right)
    {
        int written = std::snprintf((char *)send_buffer,
                                    sizeof(send_buffer),
                                    CMD_PING CMD_TERM //FIXME
                                    CMD_DRIVE_RAW CMD_DELIM
                                    "%d" CMD_PARAM_DELIM
                                    "%d" CMD_TERM "\n",
                                    left,
                                    right);
        if (written < 0 || written >= sizeof(send_buffer)) {
            ROS_ERROR("error when constructing drive command");
            return;
        }

        ROS_DEBUG("sending drive command: %s, len:%d", send_buffer, written);
        serial->send_bytes(send_buffer, written);
    }

    void
    send_start_scan_cmd()
    {
        int written = std::snprintf((char *)send_buffer,
                                    sizeof(send_buffer),
                                    CMD_PING CMD_TERM //FIXME
                                    CMD_START_SCAN CMD_TERM "\n");
        if (written < 0 || written >= sizeof(send_buffer)) {
            ROS_ERROR("error when constructing start scan command");
            return;
        }

        ROS_INFO("sending start scan command");
        serial->send_bytes(send_buffer, written);
    }

    void
    send_stop_scan_cmd()
    {
        int written = std::snprintf((char *)send_buffer,
                                    sizeof(send_buffer),
                                    CMD_PING CMD_TERM //FIXME
                                    CMD_STOP_SCAN CMD_TERM "\n");
        if (written < 0 || written >= sizeof(send_buffer)) {
            ROS_ERROR("error when constructing stop scan command");
            return;
        }

        ROS_INFO("sending stop scan command");
        serial->send_bytes(send_buffer, written);
    }

    void
    work_steer()
    {
        int left = vel_cmd[0]*100;
        int right = vel_cmd[1]*100;
        send_drive_cmd(left, right);
    }

    std::vector<int>
    cmd_line_to_params_vector(std::string cmd_line,
                              std::size_t expected_len)
    {
        std::istringstream ss(cmd_line);
        std::string token;
        std::vector<int> values;

        while(std::getline(ss, token, CMD_PARAM_DELIM[0]))
        {
            const int value = std::stoi(token);
            values.push_back(value);
        }
        if (values.size() != expected_len) {
            ROS_ERROR("param vector size mismatch, expected: %lu, parsed: %lu", (long unsigned int)expected_len, (long unsigned int)values.size());
            return std::vector<int>();
        }
        return values;
    }

    bool
    handle_notify_scan(std::string cmd_line)
    {
        static uint32_t sequence_number_generator = 0;
        std::vector<int> laser_distances = cmd_line_to_params_vector(cmd_line, SCAN_BUF_LEN);

        if (laser_distances.size() != SCAN_BUF_LEN) {
            ROS_ERROR("incorrect scan length received");
            return false;
        }

        ROS_INFO_STREAM("laser scan received");

        /* construct laser scan message */
        sensor_msgs::LaserScan laser_scan;
        laser_scan.header.stamp = ros::Time::now();
        laser_scan.header.seq = sequence_number_generator++;
        laser_scan.header.frame_id = std::string(laser_scan_frame_id);
        laser_scan.angle_increment = LASER_DEG_STEP * DEG_TO_RAD;
        laser_scan.time_increment = 0;
        laser_scan.scan_time = 0;
        laser_scan.range_min = LASER_RANGE_MIN_M;
        laser_scan.range_max = LASER_RANGE_MAX_M;
        laser_scan.angle_min = (-PI / 2) + (SCAN_MIN_DEG * DEG_TO_RAD);
        laser_scan.angle_max = (-PI / 2) + (SCAN_MAX_DEG * DEG_TO_RAD);
        for (int i = 0; i < SCAN_BUF_LEN; i++) {
            const double distance = (double)laser_distances[i] / LASER_UNITS_PER_METER;
            laser_scan.ranges.push_back(distance);
            laser_scan.intensities.push_back(LASER_DEFAULT_INTENSITY);
        }

        /* publish message */
        scan_publisher.publish(laser_scan);
        return true;
    }

    bool
    handle_notify_encoders(std::string cmd_line)
    {
        std::vector<int> encoder_distances = cmd_line_to_params_vector(cmd_line, 2);
        const int left_encoder = encoder_distances[0];
        const int right_encoder = encoder_distances[1];
        ROS_DEBUG_STREAM("encoders: " << left_encoder << ", " << right_encoder);

        /* update position state */
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
        return true;
    }

    void
    handle_cmd_line(std::string cmd_line)
    {
        const size_t cmd_delim_pos = cmd_line.find(CMD_DELIM);
        if (cmd_delim_pos == std::string::npos) {
            ROS_ERROR("no CMD_DELIM found in received data");
            return;
        }

        std::string command = cmd_line.substr(0, cmd_delim_pos);
        cmd_line = cmd_line.substr(cmd_delim_pos + 1);

        if (command.compare(CMD_NOTIFY_SCAN) == 0) {
            handle_notify_scan(cmd_line);
        }
        else if (command.compare(CMD_NOTIFY_ENCODERS) == 0) {
            handle_notify_encoders(cmd_line);
        }
        else {
            ROS_INFO_STREAM("unknown command: " << command << ",params: " << cmd_line);
        }
    }

    std::string
    sanitize_string(std::string str)
    {
        const std::string legal_chars = CMD_LEGAL_CHARSET;
        std::string new_string;
        std::string::iterator it;

        for (it = str.begin(); it != str.end(); it++) {
            const bool found = legal_chars.find(*it) != std::string::npos;
            if (found == true) new_string.append(1,*it);
        }
        return new_string;
    }

    void
    serial_callback(const uint8_t *buf,
                    size_t len)
    {
        /* copy data to receive buffer */
        std::size_t i;
        for (i = 0; i < len; i++) {
            receive_buffer.push_back(buf[i]);
        }

        /* watch size of receive buffer vector */
        if ((receive_buffer.size() + len) > CMD_RECEIVE_BUF_LEN) {
            ROS_ERROR("receive buffer limit overflow");
            std::exit(-3);
        }

        /* find terminator character and process */
        while (true) {
            auto idx = std::find(receive_buffer.begin(),
                                 receive_buffer.end(),
                                 (uint8_t) CMD_TERM[0]);
            /* skip processing if no terminator found */
            if (idx == receive_buffer.end()) return;

            std::string cmd_line(receive_buffer.begin(), idx);
            receive_buffer.erase(receive_buffer.begin(), (idx + 1));

            cmd_line = sanitize_string(cmd_line);
            handle_cmd_line(cmd_line);
        }
    }
};

int
main(int argc,
     char **argv)
{
    /* get commandline parameters */
    if (argc != 5) {
        ROS_ERROR("USAGE: ./progname <serial_port> <laser_publish_topic> <laser_scan_frame_id> <loop_rate_hz>");
        std::exit(-1);
    }
    std::string serial_port_filename = std::string(argv[1]);
    std::string laser_scan_publish_topic = std::string(argv[2]);
    std::string laser_scan_frame_id = std::string(argv[3]);
    std::string loop_rate_str = std::string(argv[4]);
    const double loop_rate_hz = std::stod(std::string(loop_rate_str));

    /* initialize ros node */
    ros::init(argc, argv, "scanbot_hwiface");

    /* instantiate scanbot class */
    const unsigned int baud_rate = 38400;
    Scanbot robot(laser_scan_publish_topic,
                  laser_scan_frame_id,
                  serial_port_filename,
                  baud_rate);

    /* init required ros objects */
    controller_manager::ControllerManager cm(&robot);

    ros::AsyncSpinner asyncSpinner(0);
    asyncSpinner.start();

    ros::Time last_t = ros::Time::now();
    ros::Rate rate(loop_rate_hz);
    while (ros::ok() == true) {
        /* update controller's time delta */
        cm.update(ros::Time::now(),
                  ros::Time::now() - last_t);
        last_t = ros::Time::now();

        robot.work_steer();

        /* throttle */
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
