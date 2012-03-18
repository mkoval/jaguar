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

static boost::shared_ptr<DiffDriveRobot> robot;

static std::string const port = "/dev/ttyUSB0";
static int const left_id = 1;
static int const right_id = 2;
static int const rate_heartbeat = 100; // ms
static int const rate_odometry = 20; // ms
static double const v_max = 1000.0; // m/s
static double const omega_max = 1000.0; // m/s
static double const robot_radius = 0.3; // m

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
    if (fabs(twist.linear.x) > v_max) {
        ROS_WARN_THROTTLE(10.0, "Linear velocity exceeds maximum speed.");
    }
    if (fabs(twist.angular.z) > omega_max) {
        ROS_WARN_THROTTLE(10.0, "Angular velocity exceeds maximum turn rate.");
    }

    // Constraining the velocity is probably not necessary, but it may help
    // the stability of the control loop by limiting integral windup.
    double const v = std::min(std::max(twist.linear.x, -v_max), v_max);
    double const omega = std::min(std::max(twist.angular.z, -omega_max), omega_max);

    robot->drive(v, omega);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "diff_drive_node");
    ros::NodeHandle nh;

    sub_twist = nh.subscribe("cmd_vel", 1, &callback_cmd);
    pub_odom   = nh.advertise<nav_msgs::Odometry>("odom", 100);

    robot = boost::make_shared<DiffDriveRobot>(
        port,
        left_id, right_id,
        rate_heartbeat, rate_odometry,
        robot_radius
    );
    robot->odom_attach(&callback_odom);
    return 0;
}
