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

struct VelocityAndAcceleration{ float v; float a; };
VelocityAndAcceleration triangle_wave(double t, double amplitude, double period ) {
    //  of amplitude a and period 2*p

    VelocityAndAcceleration rv;
    rv.a = 2*amplitude / period;

    float x = fmod(t, period) - period/2;
    if(x<0) {
        rv.a = -rv.a;
    }
    rv.v = abs(x) * 2*amplitude/period;
    

    return rv;
}

VelocityAndAcceleration sin_wave(double t, double amplitude, double period ) {
    VelocityAndAcceleration rv;
    rv.v = (sin( (t/period) * M_2_PI)+1) * amplitude / 2.;
    rv.a = (cos( (t/period) * M_2_PI)) * amplitude / (2. * period);
    return rv;
}


// action model based on
// http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29
class DriveDistanceAction {
public:
    ros::NodeHandle node_;
    actionlib::SimpleActionServer<car_msgs::DriveDistanceAction> action_server_;
    std::string action_name_;
    float start_meters_ = NAN;
    car_msgs::DriveDistanceGoal goal_;
    car_msgs::DriveDistanceFeedback feedback_;
    car_msgs::DriveDistanceResult result_;
    ros::Subscriber drive_distance_subscriber_;
    ros::Publisher enable_rc_mode_publisher_;

    ros::Publisher speed_command_publisher_;
    ros::Publisher steer_command_publisher_;

    ros::ServiceClient disable_rc_mode_service_;
    ros::ServiceClient enable_rc_mode_service_;

    car_msgs::Speedometer motor_speedometer_;

    
  DriveDistanceAction(std::string name) : 
    action_server_(node_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    action_server_.registerGoalCallback(boost::bind(&DriveDistanceAction::goal_callback, this));
    action_server_.registerPreemptCallback(boost::bind(&DriveDistanceAction::preempt_callback, this));

    //subscribe to the data topic of interest
    drive_distance_subscriber_ = node_.subscribe("car/speedometers/motor", 1, &DriveDistanceAction::speedometer_callback, this);
    speed_command_publisher_ = node_.advertise<car_msgs::SpeedCommand>("car/speed_command", 1);
    steer_command_publisher_ = node_.advertise<car_msgs::SteerCommand>("car/steer_command", 1);
    disable_rc_mode_service_ = node_.serviceClient<std_srvs::Trigger>("car/disable_rc_mode");
    enable_rc_mode_service_ = node_.serviceClient<std_srvs::Trigger>("car/enable_rc_mode");

    action_server_.start();
  }

  ~DriveDistanceAction(void)
  {
  }

  void enable_rc_mode() {
    std_srvs::Trigger trigger;
    if (enable_rc_mode_service_.call(trigger))
    {
        ROS_INFO("enable rc mode ok");
    }
    else
    {
        ROS_ERROR("enable rc mode failed");
    }
  }

  void disable_rc_mode() {
    std_srvs::Trigger trigger;
    if (disable_rc_mode_service_.call(trigger))
    {
        ROS_INFO("enable rc mode ok");
    }
    else
    {
        ROS_ERROR("enable rc mode failed");
    }
  }


  void goal_callback()
  {
    // accept the new goal
    start_meters_ = NAN;
    goal_ = *(action_server_.acceptNewGoal());
    ROS_INFO(
      "%s: new goal. distance: %f  max_v: %f max_accel: %f max_decel: %f", 
      action_name_.c_str(), 
      goal_.distance,
      goal_.max_v,
      goal_.max_accel,
      goal_.max_decel
    );

  }

  void preempt_callback()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    action_server_.setPreempted();
  }

  void speedometer_callback(const car_msgs::Speedometer::ConstPtr& msg)
  {
    motor_speedometer_ = *msg;

    // make sure that the action hasn't been canceled
    if (!action_server_.isActive())
      return;
    
    if(isnan(start_meters_)) {
        start_meters_ = motor_speedometer_.meters;
        enable_rc_mode();
    }
    
    feedback_.distance = motor_speedometer_.meters - start_meters_;

    car_msgs::SpeedCommand speed_command;
    speed_command.header = motor_speedometer_.header;
    speed_command.acceleration = 0.0;

    bool done = feedback_.distance > goal_.distance;

    
    speed_command.velocity = 0; // start with zero
    if(!done) {
      auto elapsed_secs = msg->header.stamp.toSec() - goal_.header.stamp.toSec();
      auto ramp_up_v = elapsed_secs * goal_.max_accel;
// returns velocity at position x with acceleration a, initial velocity v0 and
// initial position x0
      auto remaining_distance = goal_.distance - feedback_.distance;
      auto ramp_down_v = velocity_at_position(remaining_distance, goal_.max_decel, 0.0, 0.0);

      if(ramp_up_v < goal_.max_v && ramp_up_v < ramp_down_v) {
        // ramping up, keep accelerating
        speed_command.velocity = ramp_up_v;
        speed_command.acceleration = goal_.max_accel;
      } else if( ramp_down_v < ramp_up_v && ramp_down_v < goal_.max_v) {
        // ramping down, deccelerate
        speed_command.velocity = ramp_down_v;
        speed_command.acceleration = -abs(acceleration_for_distance_and_velocities(remaining_distance, motor_speedometer_.v_smooth, 0));
      } else {
        speed_command.velocity = goal_.max_v;
        speed_command.acceleration = 0;
      }
    }



    speed_command_publisher_.publish(speed_command);

    car_msgs::SteerCommand steer_command;
    steer_command.header = motor_speedometer_.header;
    steer_command.curvature = 0;
    steer_command.curvature_rate = 0;
    steer_command_publisher_.publish(steer_command);

    action_server_.publishFeedback(feedback_);

    if(done)
    {
        disable_rc_mode();
        action_server_.setSucceeded(result_);
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // action_server_.setAborted(result_);
    } 
  }


};

int main(int argc, char ** argv) {
    ros::init(argc, argv, "driver");

    {
        // Driver driver;
        DriveDistanceAction drive_distance_action(ros::this_node::getName());

        ros::Rate rate(100);
        while(ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

    return 0;

}