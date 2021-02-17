#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include "car_msgs/speedometer.h"
#include "car_msgs/rc_command.h"
#include "car_msgs/drive_distanceAction.h"
#include "std_srvs/Trigger.h"
#include "math.h"

struct VelocityAndAccleration{ float v; float a; };
VelocityAndAccleration triangle_wave(double t, double amplitude, double period ) {
    //  of amplitude a and period 2*p

    VelocityAndAccleration rv;
    rv.a = 2*amplitude / period;

    float x = fmod(t, period) - period/2;
    if(x<0) {
        rv.a = -rv.a;
    }
    rv.v = abs(x) * 2*amplitude/period;
    

    return rv;
}

VelocityAndAccleration sin_wave(double t, double amplitude, double period ) {
    VelocityAndAccleration rv;
    rv.v = (sin( (t/period) * M_2_PI)+1) * amplitude / 2.;
    rv.a = (cos( (t/period) * M_2_PI)) * amplitude / (2. * period);
    return rv;
}


// action model based on
// http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionServer%28GoalCallbackMethod%29
class DriveDistanceAction {
public:
    
  DriveDistanceAction(std::string name) : 
    as_(nh_, name, false),
    action_name_(name)
  {
    //register the goal and feeback callbacks
    as_.registerGoalCallback(boost::bind(&DriveDistanceAction::goal_callback, this));
    as_.registerPreemptCallback(boost::bind(&DriveDistanceAction::preempt_callback, this));

    //subscribe to the data topic of interest
    sub_ = nh_.subscribe("/car/speedometers/motor", 1, &DriveDistanceAction::speedometer_callback, this);
    as_.start();
  }

  ~DriveDistanceAction(void)
  {
  }

  void goal_callback()
  {
    ROS_INFO("%s: new goal");
    // accept the new goal
    start_meters_ = NAN;
    goal_distance_ = as_.acceptNewGoal()->distance;
  }

  void preempt_callback()
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
  }

  void speedometer_callback(const car_msgs::speedometer::ConstPtr& msg)
  {
    // make sure that the action hasn't been canceled
    if (!as_.isActive())
      return;
    
    if(isnan(start_meters_)) {
        start_meters_ = msg->meters;
    }
    
    feedback_.distance = msg->meters - start_meters_;

    as_.publishFeedback(feedback_);

    if(feedback_.distance > goal_distance_) 
    {
        as_.setSucceeded(result_);
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // as_.setAborted(result_);
    } 
  }

protected:
    
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<car_msgs::drive_distanceAction> as_;
  std::string action_name_;
  float start_meters_ = NAN;
  float goal_distance_;
  car_msgs::drive_distanceFeedback feedback_;
  car_msgs::drive_distanceResult result_;
  ros::Subscriber sub_;
};




class Driver {
    ros::Subscriber motor_speedometer_sub_;
    car_msgs::speedometer motor_speedometer_;
    bool speedomoter_ok_ = false;
    ros::NodeHandle node_;
    ros::Publisher rc_command_publisher_;
    ros::Publisher enable_rc_mode_publisher_;
    ros::Publisher setpoint_v_publisher_;
    ros::Publisher setpoint_a_publisher_;
    car_msgs::rc_command rc_command;

public:
    
    Driver() {
        rc_command_publisher_ = node_.advertise<car_msgs::rc_command>("car/rc_command", 1);
        setpoint_v_publisher_ = node_.advertise<std_msgs::Float64>("driver/setpoint/v", 1);
        setpoint_a_publisher_ = node_.advertise<std_msgs::Float64>("driver/setpoint/a", 1);

        // enable_rc_mode_publisher_ = node_.advertise<std_msgs::Bool>("car/enable_rc_mode", 1, true);
        motor_speedometer_sub_ = node_.subscribe<car_msgs::speedometer> ("car/speedometers/motor", 1, &Driver::motor_speedometer_callback_, this);

        ros::ServiceClient client = node_.serviceClient<std_srvs::Trigger>("car/enable_rc_mode");
        std_srvs::Trigger srv;
        if (client.call(srv))
        {
            ROS_INFO("enable rc mode ok");
        }
        else
        {
            ROS_ERROR("enable rc mode failed");
        }


        std_msgs::Bool enable_msg;
        enable_msg.data = true;
        enable_rc_mode_publisher_.publish(enable_msg);

        rc_command.str_us = 1500;
        rc_command.esc_us = 1500;

    }

    ~Driver() {
        std_msgs::Bool enable_msg;
        enable_msg.data = false;
        enable_rc_mode_publisher_.publish(enable_msg);
    }



    
    void motor_speedometer_callback_(const car_msgs::speedometer::ConstPtr& motor_speedometer) {
        motor_speedometer_ = *motor_speedometer;
        
        // std_msgs::Bool enable_msg;
        // enable_msg.data = true;
        // enable_rc_mode_publisher_.publish(enable_msg);



        float k_v = 0.3;
        float k_a = 0.1;

        double secs =ros::Time::now().toSec();
        float max_speed = 2;
        float period = 6;
        // auto setpoint = triangle_wave(secs, max_speed, period);
        auto setpoint = sin_wave(secs, max_speed, period);
        float lag = 0.15;
        float speed_ahead = setpoint.v + lag * setpoint.a;
        float v_error = speed_ahead - motor_speedometer_.v_smooth;
        float a_error = setpoint.a - motor_speedometer_.a_smooth;


        static float esc_us_float = 1540;

        esc_us_float += k_v * v_error + k_a * a_error;
        rc_command.header.stamp = motor_speedometer->header.stamp;
        rc_command.esc_us = esc_us_float;

        ROS_INFO("secs: %f setpoint.v: %f current speed: %f  setpoint.a: %f accel: %f esc_us: %f v_error: %f a_error: %f", 
            secs, 
            setpoint.v,
            motor_speedometer_.v_smooth,
            setpoint.a,
            motor_speedometer_.a_smooth,
            esc_us_float,
            v_error,
            a_error
            );

        std_msgs::Float64 setpoint_a_msg, setpoint_v_msg;
        setpoint_a_msg.data = setpoint.a;
        setpoint_v_msg.data = setpoint.v;

        setpoint_a_publisher_.publish(setpoint_a_msg);
        setpoint_v_publisher_.publish(setpoint_v_msg);

        rc_command_publisher_.publish(rc_command);

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