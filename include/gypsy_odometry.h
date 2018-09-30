/*
 * gypsy_odometry.h
 *
 *  Created on: Sep 28, 2013
 *      Author: daniel
 */

#ifndef GYPSY_ODOMETRY_H_
#define GYPSY_ODOMETRY_H_

// Constant Definitions
#define PI					3.14159

#define LEFT_SENSOR_ADR		0x72
#define RIGHT_SENSOR_ADR	0x71

#define NUM_SENSORS	2

//#define WHEEL_BASE			13.25      	// Distance between wheels
//#define WHEEL_DIM			4.875      	// Diameter of wheels
#define WHEEL_BASE			0.33655      	// Distance between wheels
#define WHEEL_DIM			0.123825      	// Diameter of wheels

#define ENC_COUNT			100        	// Number of encoder counts per revolution
#define PERIOD_DIVIDER		1E6			// Microseconds per second

// Type Definitions
struct SensorMemory {
	float			raw_sensor[NUM_SENSORS];
	float			avg_sensor[NUM_SENSORS];
	unsigned long	tick_period;
	long			tick_count;
	unsigned short	crc;
} __attribute__((packed));

//Function Prototypes
geometry_msgs::Pose2D update_location(const geometry_msgs::Pose2D current_position, const long left_delta, const long right_delta);
float update_speed(const unsigned long left_period, const unsigned long right_period);

#endif /* GYPSY_ODOMETRY_H_ */
