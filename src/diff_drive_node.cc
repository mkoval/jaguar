#include <cmath>
#include <string>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>

#include <jaguar/diff_drive.h>
#include <jaguar/JaguarConfig.h>

using namespace can;
using namespace jaguar;

static ros::Subscriber sub_twist;
static ros::Publisher pub_odom;
static ros::Publisher pub_vleft, pub_vright;

static DiffDriveSettings settings;
static boost::shared_ptr<DiffDriveRobot> robot;
static boost::shared_ptr<tf::TransformBroadcaster> pub_tf;
static std::string frame_parent;
static std::string frame_child;

void callback_odom(double x,  double y,  double theta,
                   double vx, double vy, double omega)
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
    msg_odom.twist.twist.linear.x = vx;
    msg_odom.twist.twist.linear.y = vy;
    msg_odom.twist.twist.angular.z = omega;
    pub_odom.publish(msg_odom);
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
        if (0 < config.ticks_per_rev && config.ticks_per_rev <= std::numeric_limits<uint16_t>::max()) {
            robot->robot_set_encoders(config.ticks_per_rev);
            ROS_INFO("Reconfigure, Ticks/Rev = %d", config.ticks_per_rev);
        } else {
            ROS_WARN("Ticks/rev must be a positive 16-bit unsigned integer.");
        }
    }
    if (level & 32) {
        if (config.wheel_radius <= 0) {
            ROS_WARN("Wheel radius must be positive.");
        } else if (config.robot_radius <= 0) {
            ROS_WARN("Robot radius must be positive.");
        } else {
            robot->robot_set_radii(config.wheel_radius, config.robot_radius);
            ROS_INFO("Reconfigure, Wheel Radius = %f m and Robot Radius = %f m",
                config.wheel_radius, config.robot_radius
            );
        }
    }
    if (level & 64) {
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
    ros::param::get("~heartbeat", settings.heartbeat_ms);
    ros::param::get("~status", settings.status_ms);
    ros::param::get("~frame_parent", frame_parent);
    ros::param::get("~frame_child", frame_child);
    ros::param::get("~accel_max", settings.accel_max_mps2);

    // TODO: Read this from a parameter.
    settings.brake = BrakeCoastSetting::kOverrideCoast;

    if (!(1 <= settings.id_left  && settings.id_left  <= 63)
     || !(1 <= settings.id_right && settings.id_right <= 63)) {
        ROS_FATAL("Invalid CAN device id. Must be in the range 1-63.");
        return 1;
    } else if (settings.heartbeat_ms <= 0 || settings.heartbeat_ms > 100) {
        ROS_FATAL("Heartbeat period invalid. Must be in the range 1-500 ms.");
        return 1;
    } else if (settings.status_ms <= 0 || settings.status_ms > std::numeric_limits<uint16_t>::max()) {
        ROS_FATAL("Status period invalid must be in the range 1-255 ms.");
        return 1;
    }

    ROS_INFO("Port: %s", settings.port.c_str());
    ROS_INFO("ID Left: %d", settings.id_left);
    ROS_INFO("ID Right: %d", settings.id_right);
    ROS_INFO("Heartbeat Rate: %d ms", settings.heartbeat_ms);
    ROS_INFO("Status Rate: %d ms", settings.status_ms);

    // This must be done first because the asynchronous encoder callbacks use
    // the transform broadcaster.
    sub_twist = nh.subscribe("cmd_vel", 1, &callback_cmd);
    pub_odom  = nh.advertise<nav_msgs::Odometry>("odom", 100);
    pub_vleft  = nh.advertise<std_msgs::Float64>("encoder_left", 100);
    pub_vright = nh.advertise<std_msgs::Float64>("encoder_right", 100);
    pub_tf = boost::make_shared<tf::TransformBroadcaster>();

    dynamic_reconfigure::Server<jaguar::JaguarConfig> server;
    dynamic_reconfigure::Server<jaguar::JaguarConfig>::CallbackType f;
    f = boost::bind(&callback_reconfigure, _1, _2);
    server.setCallback(f);

    robot = boost::make_shared<DiffDriveRobot>(settings);
    robot->odom_attach(&callback_odom);
    robot->speed_attach(&callback_speed);

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
