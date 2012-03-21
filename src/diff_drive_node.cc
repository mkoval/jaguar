#include <cmath>
#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <jaguar/diff_drive.h>

using namespace can;
using namespace jaguar;

static ros::Subscriber sub_twist;
static ros::Publisher pub_odom;
static tf::TransformBroadcaster pub_tf;

static DiffDriveSettings settings;
static boost::shared_ptr<DiffDriveRobot> robot;

void callback_odom(double x,  double y,  double theta,
                   double vx, double vy, double omega)
{
    ros::Time now = ros::Time::now();

    // odom TF Frame
    geometry_msgs::TransformStamped msg_tf;
    msg_tf.header.stamp = now;
    msg_tf.header.frame_id = "odom";
    msg_tf.child_frame_id  = "base_link";
    msg_tf.transform.translation.x = x;
    msg_tf.transform.translation.y = y;
    msg_tf.transform.rotation = tf::createQuaternionMsgFromYaw(theta);
    pub_tf.sendTransform(msg_tf);

    // Odometry Message
    nav_msgs::Odometry msg_odom;
    msg_odom.header.stamp = now;
    msg_odom.header.frame_id = "odom";
    msg_odom.child_frame_id  = "base_link";
    msg_odom.pose.pose.position.x = x;
    msg_odom.pose.pose.position.y = y;
    msg_odom.twist.twist.linear.x = vx;
    msg_odom.twist.twist.linear.y = vy;
    msg_odom.twist.twist.angular.z = omega;
    pub_odom.publish(msg_odom);
}

void callback_cmd(geometry_msgs::Twist const &twist)
{
    if (twist.linear.y  != 0.0 || twist.linear.z  != 0
     || twist.angular.x != 0.0 || twist.angular.y != 0) {
        ROS_WARN_THROTTLE(10.0, "Ignoring non-zero component of velocity command.");
    }
    robot->drive(twist.linear.x, twist.angular.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_drive_node");
    ros::NodeHandle nh;

    sub_twist = nh.subscribe("cmd_vel", 1, &callback_cmd);
    pub_odom  = nh.advertise<nav_msgs::Odometry>("odom", 100);

    ros::param::get("port", settings.port);
    ros::param::get("id_left", settings.id_left);
    ros::param::get("id_right", settings.id_right);
    ros::param::get("heartbeat", settings.heartbeat_ms);
    ros::param::get("status", settings.status_ms);
    ros::param::get("ticks_per_rev", settings.ticks_per_rev);
    ros::param::get("wheel_radius", settings.wheel_radius_m);
    ros::param::get("robot_radius", settings.robot_radius_m);

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
    } else if (settings.ticks_per_rev<= 0) {
        ROS_FATAL("Number of ticks per revolution must be positive");
        return 1;
    } else if (settings.wheel_radius_m <= 0) {
        ROS_FATAL("Wheel radius must be positive.");
        return 1;
    } else if (settings.robot_radius_m <= 0) {
        ROS_FATAL("Robot radius must be positive.");
        return 1;
    }

    robot = boost::make_shared<DiffDriveRobot>(settings);
    robot->odom_attach(&callback_odom);

    ros::spin();

    return 0;
}
