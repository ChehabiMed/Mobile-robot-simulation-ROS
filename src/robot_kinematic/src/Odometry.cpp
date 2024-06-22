/*
 * ROS Node for Differential Drive Robot Odometry
 * 
 * This node subscribes to joint state messages from the robot and computes odometry
 * based on the wheel velocities. It publishes the odometry information to the /wheel/odom topic.
 * Additionally, it publishes a TF between the world frame (odom) and base_link frame.
 * 
 * Maintainer: chehabi.mohammed@gmail.com
 * License: MIT
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <cmath>

// Global variables for motor rate and buffer size
int motor_rate;
int buffer_size;

class DifferentialDriveRobot
{
public:
    DifferentialDriveRobot()
    {
        // Initialize ROS node and publishers/subscribers
        ros::NodeHandle nh;
        odom_pub = nh.advertise<nav_msgs::Odometry>("/wheel/odom", buffer_size);
        joint_state_sub = nh.subscribe("robot/joint_states", buffer_size, &DifferentialDriveRobot::jointStateCallback, this);

        // Initialize parameters from ROS parameter server
        nh.param<double>("wheel_separation", wheel_separation, 0.138);
        nh.param<double>("wheel_radius", wheel_radius, 0.032);
        nh.param<int>("motor_rate", motor_rate, 100);
        nh.param<int>("buffer_size", buffer_size, 10);

        // Initialize odometry variables
        x = 0.0;
        y = 0.0;
        theta = 0.0;
        last_time = ros::Time::now();

        // Initialize TF broadcaster
        tf_broadcaster = new tf::TransformBroadcaster();
        last_tf_publish_time = ros::Time(0);  // Initialize last publish time to 0
    }

    // Callback function to handle joint state messages
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr &joint_state_msg)
    {
        double real_velocity_left = joint_state_msg->velocity[0];
        double real_velocity_right = joint_state_msg->velocity[1];

        double linear_velocity = (wheel_radius / 2.0) * (real_velocity_left + real_velocity_right);
        double angular_velocity = (wheel_radius / wheel_separation) * (real_velocity_right - real_velocity_left);

        computeOdometry(linear_velocity, angular_velocity);
    }

    // Function to compute and publish odometry
    void computeOdometry(double linear_velocity, double angular_velocity)
    {
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();
        last_time = current_time;

        // Update odometry
        x += linear_velocity * cos(theta) * dt;
        y += linear_velocity * sin(theta) * dt;
        theta += angular_velocity * dt;

        // Create quaternion from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        // Fill odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        // Set position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.orientation = odom_quat;

        // Set velocity
        odom.twist.twist.linear.x = linear_velocity;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = angular_velocity;

        // Publish the message
        odom_pub.publish(odom);

        // Publish TF transform if time difference is significant
        if ((current_time - last_tf_publish_time).toSec() > 0.1)  // Adjust threshold as needed
        {
            last_tf_publish_time = current_time;

            geometry_msgs::TransformStamped odom_tf;
            odom_tf.header.stamp = current_time;
            odom_tf.header.frame_id = "odom";
            odom_tf.child_frame_id = "base_link";
            odom_tf.transform.translation.x = x;
            odom_tf.transform.translation.y = y;
            odom_tf.transform.translation.z = 0.0;
            odom_tf.transform.rotation = odom_quat;

            tf_broadcaster->sendTransform(odom_tf);
        }
    }

private:
    ros::Publisher odom_pub;           // Publisher for odometry messages
    ros::Subscriber joint_state_sub;   // Subscriber for joint state messages
    tf::TransformBroadcaster* tf_broadcaster;  // TF broadcaster instance
    double wheel_radius;               // Radius of the wheels
    double wheel_separation;           // Distance between the wheels
    double x, y, theta;                // Odometry state
    ros::Time last_time, current_time; // Time variables for dt calculation
    ros::Time last_tf_publish_time;    // Last time TF transform was published
};

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "differential_drive_robot");

    // Create DifferentialDriveRobot object
    DifferentialDriveRobot robot;

    // Set the control loop rate
    ros::Rate rate(motor_rate);

    // Main control loop
    while (ros::ok())
    {
        ros::spinOnce(); // Process incoming messages
        rate.sleep();    // Sleep to maintain the loop rate
    }

    return 0;
}
