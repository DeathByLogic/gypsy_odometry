/*
 * gypsydriver.cpp
 *
 *  Created on: Sep 28, 2013
 *      Author: daniel
 */

#include <cmath>
#include <unistd.h>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>

#include "beagleIO.h"
#include "gypsy_odometry.h"
#include "encoders.h"

// Global constants
const double rate = 100.0;

// Global variables

int main (int argc, char** argv) {
	// Store current and previous time
	ros::Time current_time, last_time;

	// Current position and velocity
	geometry_msgs::Pose2D current_pos;
	geometry_msgs::Pose2D current_vel;

	// Left and Right tick count from encoders
	long left_count_delta;
	long right_count_delta;

	// Previous Left and Right tick count from encoders
	long prev_left_count;
	long prev_right_count;

	std_msgs::Float32 left_speed;
	std_msgs::Float32 right_speed;

	// Init ROS
	ros::init(argc, argv, "gypsy_odometry");

	// Create node and publication handlers
	ros::NodeHandle n;
	ros::Publisher left_speed_pub = n.advertise<std_msgs::Float32>("left_wheel_speed", 10);
	ros::Publisher right_speed_pub = n.advertise<std_msgs::Float32>("right_wheel_speed", 10);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
	tf::TransformBroadcaster odom_broadcaster;

	// Get and save current time
	current_time = ros::Time::now();
	last_time = ros::Time::now();

	ros::Rate r(rate);
	while (n.ok()) {
		// Check for incoming messages
		ros::spinOnce();
		current_time = ros::Time::now();

		//calculate change in time (dt)
		double dtt = (current_time - last_time).toSec();

		// Read the encoders
		SensorMemory left_sensor;
		SensorMemory right_sensor;

		// Read encoder values over I2C
		//read_encoder(&left_sensor, sizeof(left_sensor), LEFT_SENSOR_ADR);
		//read_encoder(&right_sensor, sizeof(right_sensor), RIGHT_SENSOR_ADR);

		// Update the location based on the count delta
		//current_pos = update_location(current_pos, left_sensor.tick_count - prev_left_count, right_sensor.tick_count - prev_right_count);
		current_pos = update_location(current_pos, 1, -1);

		// Update previous tick counts
		prev_left_count = left_sensor.tick_count;
		prev_right_count = right_sensor.tick_count;

		// Update the speed based on the periods
		//speed = update_speed(left_sensor.tick_period, right_sensor.tick_period);
		left_speed.data = (WHEEL_DIM * PI / ENC_COUNT) / (left_sensor.tick_period / PERIOD_DIVIDER);
		right_speed.data = (WHEEL_DIM * PI / ENC_COUNT) / (right_sensor.tick_period / PERIOD_DIVIDER);

		// Calculate velocities
		current_vel.x = (right_speed.data + left_speed.data) / 2;
		current_vel.y = 0.0;
		current_vel.theta = (right_speed.data - left_speed.data) / WHEEL_BASE;

		//since all odometry is 6DOF we'll need a quaternion created from yaw
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_pos.theta);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = current_time;
		odom_trans.header.frame_id = "odom";
		odom_trans.child_frame_id = "base_link";

		odom_trans.transform.translation.x = current_pos.x;
		odom_trans.transform.translation.y = current_pos.y;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = current_time;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = current_pos.x;
		odom.pose.pose.position.y = current_pos.y;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_link";
		odom.twist.twist.linear.x = current_vel.x;
		odom.twist.twist.linear.y = current_vel.y;
		odom.twist.twist.angular.z = current_vel.theta;

		//publish the message
		odom_pub.publish(odom);

		// Publish current left and right velocity
		left_speed_pub.publish(left_speed);
		right_speed_pub.publish(right_speed);

		// Update time
		last_time = current_time;

		// Wait for next period
		r.sleep();
	}

	return 0;
}

// Update the calculated position with new data
geometry_msgs::Pose2D update_location(const geometry_msgs::Pose2D current_position, const long left_delta, const long right_delta) {
	geometry_msgs::Pose2D new_position;

	float theta_l;
	float theta_r;
	float theta_sum;

	float delta_x = 0.0;
	float delta_y = 0.0;

	// Left Wheel
	theta_l = -((float)left_delta * WHEEL_DIM * PI / ENC_COUNT) / WHEEL_BASE;

	delta_x += 0.5 * WHEEL_BASE * (sin(current_position.theta) - sin(current_position.theta + theta_l));
	delta_y += 0.5 * WHEEL_BASE * (cos(current_position.theta + theta_l) - cos(current_position.theta));

	// Right Wheel
	theta_r = ((float)right_delta * WHEEL_DIM * PI / ENC_COUNT) / WHEEL_BASE;

	delta_x += 0.5 * WHEEL_BASE * (sin(current_position.theta + theta_r) - sin(current_position.theta));
	delta_y += 0.5 * WHEEL_BASE * (cos(current_position.theta) - cos(current_position.theta + theta_r));

	// Sum up current theta and delta
	theta_sum = current_position.theta + (theta_r + theta_l);

	// Keep theta within +- PI
	if (theta_sum > PI) {
		theta_sum -= 2 * PI;
	}

	if (theta_sum < -PI) {
		theta_sum += 2 * PI;
	}

	// Update variables
	new_position.x = current_position.x + delta_x;
	new_position.y = current_position.y + delta_y;
	new_position.theta = theta_sum;

	return new_position;
}

float calculate_speed(const unsigned long left_period, const unsigned long right_period) {
	float rightSpeed;
	float leftSpeed;

	leftSpeed = (WHEEL_DIM * PI / ENC_COUNT) / (left_period / PERIOD_DIVIDER);
	rightSpeed = (WHEEL_DIM * PI / ENC_COUNT) / (right_period / PERIOD_DIVIDER);

	return (leftSpeed + rightSpeed) / 2.0;
}

