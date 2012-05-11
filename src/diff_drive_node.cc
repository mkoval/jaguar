#include <cmath>
#include <string>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <jaguar/diff_drive.h>
#include <jaguar/JaguarConfig.h>

using namespace can;
using namespace jaguar;

static ros::Subscriber sub_twist;
static ros::Publisher pub_odom;
static ros::Publisher pub_estop;
static ros::Publisher pub_temp_left, pub_temp_right;
static ros::Publisher pub_voltage_left, pub_voltage_right;
static ros::Publisher pub_vleft, pub_vright;

static DiffDriveSettings settings;
static boost::shared_ptr<DiffDriveRobot> robot;
static boost::shared_ptr<tf::TransformBroadcaster> pub_tf;
static std::string frame_parent;
static std::string frame_child;
static int heartbeat_rate;

static void callback_odom(double x, double y, double theta,
                          double velocity, double omega)
{
    ros::Time now = ros::Time::now();

    // odom TF Frame
    geometry_msgs::TransformStamped msg_tf;
    msg_tf.header.stamp = now;
    msg_tf.header.frame_id = frame_parent;
    msg_tf.child_frame_id  = frame_child;
    msg_tf.transform.translation.x = x;
    msg_tf.transform.translation.y = y;
    msg_tf.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
    pub_tf->sendTransform(msg_tf);

    // Odometry Message
    nav_msgs::Odometry msg_odom;
    msg_odom.header.stamp = now;
    msg_odom.header.frame_id = frame_parent;
    msg_odom.child_frame_id  = frame_child;
    msg_odom.pose.pose.position.x = x;
    msg_odom.pose.pose.position.y = y;
    msg_odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);
    msg_odom.twist.twist.linear.x = velocity;
    msg_odom.twist.twist.linear.y = 0;
    msg_odom.twist.twist.angular.z = omega;
    pub_odom.publish(msg_odom);
}

static void callback_estop(bool stopped)
{
    std_msgs::Bool msg;
    msg.data = stopped;
    pub_estop.publish(msg);
}

static void callback_diag_left(double voltage, double temperature)
{
    std_msgs::Float64 msg_voltage, msg_temperature;
    msg_voltage.data = voltage;
    msg_temperature.data = temperature;
    pub_voltage_left.publish(msg_voltage);
    pub_temp_left.publish(msg_temperature);
}

static void callback_diag_right(double voltage, double temperature)
{
    std_msgs::Float64 msg_voltage, msg_temperature;
    msg_voltage.data = voltage;
    msg_temperature.data = temperature;
    pub_voltage_right.publish(msg_voltage);
    pub_temp_right.publish(msg_temperature);
}

void callback_speed(DiffDriveRobot::Side side, double speed)
{
    std_msgs::Float64 msg;
    msg.data = speed;

    switch (side) {
    case DiffDriveRobot::kLeft:
        pub_vleft.publish(msg);
        break;

    case DiffDriveRobot::kRight:
        pub_vright.publish(msg);
        break;

    default:
        ROS_WARN_THROTTLE(10, "Invalid speed callback.");
    }
}

void callback_cmd(geometry_msgs::Twist const &twist)
{
    if (twist.linear.y  != 0.0 || twist.linear.z  != 0
     || twist.angular.x != 0.0 || twist.angular.y != 0) {
        ROS_WARN_THROTTLE(10.0, "Ignoring non-zero component of velocity command.");
    }
    robot->drive(twist.linear.x, twist.angular.z);
}

void callback_reconfigure(jaguar::JaguarConfig &config, uint32_t level)
{
    // Speed Control Gains
    if (level & 1) {
        robot->speed_set_p(config.gain_p);
        ROS_INFO("Reconfigure, P = %f", config.gain_p);
    }
    if (level & 2) {
        robot->speed_set_i(config.gain_i);
        ROS_INFO("Reconfigure, I = %f", config.gain_i);
    }
    if (level & 4) {
        robot->speed_set_d(config.gain_d);
        ROS_INFO("Reconfigure, D = %f", config.gain_d);
    }
    if (level & 8) {
        robot->drive_brake(config.brake);
        ROS_INFO("Reconfigure, Braking = %d", config.brake);
    }
    if (level & 16) {
        if (0 < config.cpr && config.cpr <= std::numeric_limits<uint16_t>::max()) {
            robot->odom_set_encoders(config.cpr);
            ROS_INFO("Reconfigure, CPR = %d", config.cpr);
        } else {
            ROS_WARN("CPR must be a positive 16-bit unsigned integer.");
        }
    }
    if (level & 32) {
        if (config.wheel_diameter <= 0) {
            ROS_WARN("Wheel diameter must be positive.");
        } else {
            robot->odom_set_circumference(M_PI * config.wheel_diameter);
            ROS_INFO("Reconfigure, Wheel Diameter = %f m", config.wheel_diameter);
        }
    }
    if (level & 64) {
        if (config.wheel_separation <= 0) {
            ROS_WARN("Wheel separation must be positive.");
        } else {
            robot->odom_set_separation(config.wheel_separation);
            ROS_INFO("Reconfigure, Wheel Separation = %f m", config.wheel_separation);
        }
    }
    if (level & 128) {
        if (config.odom_rate <= 0 || config.odom_rate > 255) {
            ROS_WARN("Odometry update rate must be positive.");
        } else {
            robot->odom_set_rate(config.odom_rate);
            ROS_INFO("Reconfigure, Odometry Update Rate = %d ms", config.odom_rate);
        }
    }
    if (level & 256) {
        if (config.diag_rate <= 0 || config.diag_rate > 255) {
            ROS_WARN("Diagnostics update rate must be positive.");
        } else {
            robot->diag_set_rate(config.diag_rate);
            ROS_INFO("Reconfigure, Diagnostics Update Rate = %d ms", config.diag_rate);
        }
    }
    if (level & 512) {
        if (config.heartbeat_rate <= 0 || config.heartbeat_rate > 255) {
            ROS_WARN("Heartbeat rate must be in the range 1-255.");
        } else if (config.heartbeat_rate > 100) {
            ROS_WARN("Heartbeat rate is dangerously high.");
        } else {
            heartbeat_rate = config.heartbeat_rate;
            ROS_INFO("Reconfigure, Heartbeat Rate = %d ms", config.heartbeat_rate);
        }
    }
    if (level & 1024) {
        robot->drive_raw(config.setpoint, config.setpoint);
        ROS_INFO("Reconfigure, Setpoint = %f", config.setpoint);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_drive_node");
    ros::NodeHandle nh;

    ros::param::get("~port", settings.port);
    ros::param::get("~id_left", settings.id_left);
    ros::param::get("~id_right", settings.id_right);
    ros::param::get("~frame_parent", frame_parent);
    ros::param::get("~frame_child", frame_child);
    ros::param::get("~accel_max", settings.accel_max_mps2);

    // TODO: Read this from a parameter.
    settings.brake = BrakeCoastSetting::kOverrideCoast;

    if (!(1 <= settings.id_left  && settings.id_left  <= 63)
     || !(1 <= settings.id_right && settings.id_right <= 63)) {
        ROS_FATAL("Invalid CAN device id. Must be in the range 1-63.");
        return 1;
    } else if (settings.id_left == settings.id_right){
        ROS_FATAL("Invalid CAN device ID. Left and right IDs must be unique.");
        return 1;
    }
    ROS_INFO("Communicating to IDs %d and %d over %s", settings.id_left,
             settings.id_right, settings.port.c_str());

    robot = boost::make_shared<DiffDriveRobot>(settings);

    // Use dynamic reconfigure for all remaining parameters.
    dynamic_reconfigure::Server<jaguar::JaguarConfig> server;
    dynamic_reconfigure::Server<jaguar::JaguarConfig>::CallbackType f;
    f = boost::bind(&callback_reconfigure, _1, _2);
    server.setCallback(f);

    sub_twist = nh.subscribe("cmd_vel", 1, &callback_cmd);
    pub_odom  = nh.advertise<nav_msgs::Odometry>("odom", 100);
    pub_estop = nh.advertise<std_msgs::Bool>("estop", 1, true);
    pub_vleft  = nh.advertise<std_msgs::Float64>("encoder_left", 100);
    pub_vright = nh.advertise<std_msgs::Float64>("encoder_right", 100);
    pub_temp_left  = nh.advertise<std_msgs::Float64>("temperature_left", 10);
    pub_temp_right = nh.advertise<std_msgs::Float64>("temperature_right", 10);
    pub_voltage_left  = nh.advertise<std_msgs::Float64>("voltage_left", 10);
    pub_voltage_right = nh.advertise<std_msgs::Float64>("voltage_right", 10);
    pub_tf = boost::make_shared<tf::TransformBroadcaster>();

    // These must be registered after the publishers are initialized. Otherwise
    // there is a race condition in the callbacks.
    robot->odom_attach(&callback_odom);
    robot->diag_attach(&callback_diag_left, &callback_diag_right);
    robot->estop_attach(&callback_estop);

    // TODO: Read this heartbeat rate from a parameter.
    ros::Rate heartbeat_rate(50);
    while (ros::ok()) {
        robot->drive_spin(1 / 50.);
        robot->heartbeat();
        ros::spinOnce();
        heartbeat_rate.sleep();
    }
    return 0;
}
