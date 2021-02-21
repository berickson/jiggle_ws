#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "car_msgs/Speedometer.h"
#include "car_msgs/RcCommand.h"
#include "car_msgs/DriveDistanceAction.h"
#include "car_msgs/SteerCommand.h"
#include "car_msgs/SpeedCommand.h"
#include "std_srvs/Trigger.h"
#include "math.h"
#include "geometry.h"
#include "lookup-table.h"

float steering_for_curvature(Angle theta_per_meter) {
  static const LookupTable t({{-85.1, 1929},
                              {-71.9, 1839},
                              {-58.2, 1794},
                              {-44.1, 1759},
                              {-29.6, 1678},
                              {-14.8, 1599},
                              {0, 1521},
                              {14.8, 1461},
                              {29.6, 1339},
                              {44.0, 1306},
                              {58.2, 1260},
                              {71.9, 1175},
                              {85.1, 1071}

  });
  static float offset = 1463-1521; // as of 2/19, center was about 1463us todo: add calibrating

  return t.lookup(theta_per_meter.degrees())+offset;
}

class CarController {
public:
  ros::Publisher rc_command_publisher_;


  ros::Subscriber speed_command_subscriber_;
  ros::Subscriber steer_command_subscriber_;
  ros::Subscriber motor_speedometer_subscriber_;

  ros::NodeHandle node_;
  car_msgs::Speedometer motor_speedometer_;
  car_msgs::SpeedCommand last_speed_command_;
  car_msgs::SteerCommand last_steer_command_;

  float esc_us_float = 1500;

  CarController() {
    speed_command_subscriber_ = node_.subscribe("car/speed_command", 1, &CarController::speed_command_callback, this);
    steer_command_subscriber_ = node_.subscribe("car/steer_command", 1, &CarController::steer_command_callback, this);
    motor_speedometer_subscriber_ = node_.subscribe("car/speedometers/motor", 1, &CarController::motor_speedometer_callback, this);
    rc_command_publisher_ = node_.advertise<car_msgs::RcCommand>( "car/rc_command", 1);
  }

  void motor_speedometer_callback(const car_msgs::Speedometer::ConstPtr & motor_speedometer) {
    motor_speedometer_ = *motor_speedometer;
  }


  void steer_command_callback(const car_msgs::SteerCommand::ConstPtr& steer_command) {
    ROS_INFO("set steer command");
    last_steer_command_ = *steer_command;

  }

  void speed_command_callback(const car_msgs::SpeedCommand::ConstPtr& speed_command) {

    // reset if it's been a long time since the last call
    const float timeout = 0.5;
    if(abs(speed_command->header.stamp.toSec() - last_speed_command_.header.stamp.toSec()) > timeout) {
      esc_us_float = 1543;
    }
    float k_v = 0.3;
    float k_a = 0.1;

    float lag = 0.15;
    float speed_ahead = speed_command->velocity + lag * speed_command->acceleration;
    float v_error = speed_ahead - motor_speedometer_.v_smooth;
    float a_error = speed_command->acceleration - motor_speedometer_.a_smooth;

    esc_us_float += k_v * v_error + k_a * a_error;

    car_msgs::RcCommand rc_command;

    rc_command.header = speed_command->header;
    rc_command.esc_us = esc_us_float;
    rc_command.str_us = steering_for_curvature(Angle::radians(last_steer_command_.curvature)); // center steer, TODO: get somewhere else

    rc_command_publisher_.publish(rc_command);
    last_speed_command_ = *speed_command;
    ROS_INFO("published rc");
  }

};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "car_controller");

    {
        // Driver driver;
        CarController car_controller;

        ros::Rate rate(100);
        while(ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;

}