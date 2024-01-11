/****************************************************************************
 *
 * Copyright 2020 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @brief Offboard control example
 * @file offboard_control.cpp
 * @addtogroup examples
 * @author Mickey Cowden <info@cowden.tech>
 * @author Nuno Marques <nuno.marques@dronesolutions.io>
 */

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>

#include <stdint.h>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

#define NOFDRONES 16

class OffboardControl : public rclcpp::Node
{
public:
	// std::string sel_ns = "";
	float vel_received[NOFDRONES][3] = {{0.0, 0.0, 0.0}};
	float loc_received[NOFDRONES][3] = {{0.0, 0.0, 0.0}};
	OffboardControl() : Node("offboard_control")
	{
		// sel_ns = px4_ns;
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
		
		for (int i = 0; i < NOFDRONES; i++) {
			// subscribe to every drones' local position/velocity data
			subscription_[i] = this->create_subscription<px4_msgs::msg::VehicleLocalPosition>(
				"/px4_"+std::to_string(i+1)+"/fmu/out/vehicle_local_position", qos,
			[this, i](const px4_msgs::msg::VehicleLocalPosition::UniquePtr msg) {
				vel_received[i][0] = msg->vx;
				vel_received[i][1] = msg->vy;
				vel_received[i][2] = msg->vz;
				loc_received[i][0] = msg->x;
				loc_received[i][1] = msg->y;
				loc_received[i][2] = msg->z;
			}
			);
			

			// create publisher for every drones' offboard control mode, trajectory setpoint, and vehicle command
			offboard_control_mode_publisher_[i] = this->create_publisher<OffboardControlMode>(
				"/px4_"+std::to_string(i+1)+"/fmu/in/offboard_control_mode", 10);
			trajectory_setpoint_publisher_[i] = this->create_publisher<TrajectorySetpoint>(
				"/px4_"+std::to_string(i+1)+"/fmu/in/trajectory_setpoint", 10);
			vehicle_command_publisher_[i] = this->create_publisher<VehicleCommand>(
				"/px4_"+std::to_string(i+1)+"/fmu/in/vehicle_command", 10);
		}
		
		offboard_setpoint_counter_ = 0;

		auto timer_callback = [this]() -> void {

			if (offboard_setpoint_counter_ == 10) {
				// Change to Offboard mode after 10 setpoints
				for (int i = 0; i < NOFDRONES; i++)
				{
					this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6, 1+i);
				}
				
				// Arm the vehicle
				this->arm();
			}

			// offboard_control_mode needs to be paired with trajectory_setpoint
			publish_offboard_control_mode();
			publish_trajectory_setpoint();

			// stop the counter after reaching 11
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}
		};
		timer_ = this->create_wall_timer(100ms, timer_callback);
	}

	void arm();
	void disarm();

private:
	rclcpp::TimerBase::SharedPtr timer_;

	rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_[NOFDRONES];
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_[NOFDRONES];
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_[NOFDRONES];
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_[NOFDRONES];

	std::atomic<uint64_t> timestamp_;   //!< common synced timestamped

	uint64_t offboard_setpoint_counter_;   //!< counter for the number of setpoints sent

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0, int target = 0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void OffboardControl::arm()
{
	for (int i = 0; i < NOFDRONES; i++)
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0, 1+i);
		/* code */
	}
	

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void OffboardControl::disarm()
{
	for (int i = 0; i < NOFDRONES; i++)
	{
		publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0, 1+i);
	}

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Publish the offboard control mode.
 *        For this example, only position and altitude controls are active.
 */
void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = nan;
	msg.velocity = true;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	for (int i = 0; i < NOFDRONES; i++)
	{
		offboard_control_mode_publisher_[i]->publish(msg);
	}
}

/**
 * @brief Publish a trajectory setpoint
 *        For this example, it sends a trajectory setpoint to make the
 *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
 */
void OffboardControl::publish_trajectory_setpoint()
{
	RCLCPP_INFO(this->get_logger(), 
		"\n\n\n\n\n1: [%f\t%f\t%f]\n2: [%f\t%f\t%f]\n3: [%f\t%f\t%f]\n4: [%f\t%f\t%f]\n\n\n\n", 
		loc_received[0][0], loc_received[0][1], loc_received[0][2], 
		loc_received[1][0], loc_received[1][1], loc_received[1][2], 
		loc_received[2][0], loc_received[2][1], loc_received[2][2], 
		loc_received[3][0], loc_received[3][1], loc_received[3][2]
	);

	
	TrajectorySetpoint msg{};
	msg.velocity = {0.0, 0.0, -1.0};
	msg.yaw = -3.14; // [-PI:PI]
	uint64_t tstimestamp = this->get_clock()->now().nanoseconds() / 1000;
	msg.timestamp = tstimestamp;
	if (tstimestamp % 10000000 < 5000000) {
		msg.velocity = {0.0, 1.0, -1.0};
	} else {
		msg.velocity = {1.0, 0.0,  -1.0};
	}
	for (int i = 0; i < NOFDRONES; i++){
		trajectory_setpoint_publisher_[i]->publish(msg);
	}
	
	/* // print next trajectory setpoint
	RCLCPP_INFO(this->get_logger(), "Trajectory has been set to... {%f, %f, %f}", 
		msg.position[0], msg.position[1], msg.position[2]); 
	*/

}

/**
 * @brief Publish vehicle commands
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2, int target)
{
	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 0;
	msg.target_component = 0;
	msg.source_system = 2;
	msg.source_component = 2;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	vehicle_command_publisher_[target-1]->publish(msg);
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());

	rclcpp::shutdown();
	return 0;
}
