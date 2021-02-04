#include <ros/ros.h>

#include "car_msgs/update.h"
#include "car_msgs/speedometer.h"

#include "speedometer.h"



class CarInstruments {
     public:
        CarInstruments();
        void update_callback(const car_msgs::update::ConstPtr& update);

        Speedometer front_right_wheel_;
        Speedometer front_left_wheel_;
        Speedometer motor_;

     private:
        int32_t update_count_ = 0;

        // car constants for blue-crash
        // todo: get from parameter server
        const float front_meters_per_odometer_tick = 0.002528;
        const float rear_meters_per_odometer_tick = 0.00146;
        const float motor_meters_per_odometer_tick = 0.00292;
        const float front_wheelbase_width_in_meters = 0.2413;
        const float rear_wheelbase_width_in_meters = 0.2667;
        const float wheelbase_length_in_meters = 0.33655;

        ros::NodeHandle node_;

        ros::Publisher fl_speedometer_publisher_;
        ros::Publisher fr_speedometer_publisher_;
        ros::Publisher motor_speedometer_publisher_;

        ros::Subscriber update_sub_;
};

CarInstruments::CarInstruments(){
    front_right_wheel_.meters_per_tick = front_meters_per_odometer_tick;
    front_left_wheel_.meters_per_tick = front_meters_per_odometer_tick;
    motor_.meters_per_tick = motor_meters_per_odometer_tick;

    bool latch = true;
    fl_speedometer_publisher_ = node_.advertise<car_msgs::speedometer> ("/car/speedometers/fl", 10, latch);
    fr_speedometer_publisher_ = node_.advertise<car_msgs::speedometer> ("/car/speedometers/fr", 10, latch);
    motor_speedometer_publisher_ = node_.advertise<car_msgs::speedometer> ("/car/speedometers/motor", 10, latch);
    const int queue_length=1; // 1 ensures latest message
    update_sub_ = node_.subscribe<car_msgs::update> ("/car/update", queue_length, &CarInstruments::update_callback, this);
}

void CarInstruments::update_callback(const car_msgs::update::ConstPtr& d){
    // ROS_INFO("got car update ms: %d us: %d", d->ms, d->us);
    ++update_count_;

    front_left_wheel_.update_from_sensor(d->us, d->odo_fl_a, d->odo_fl_a_us,
                                        d->odo_fl_b, d->odo_fl_b_us);
    motor_.update_from_sensor(d->us, d->spur_odo, d->spur_us);
    front_right_wheel_.update_from_sensor(d->us, d->odo_fr_a, d->odo_fr_a_us, 
                                         d->odo_fr_b, d->odo_fr_b_us);

    fl_speedometer_publisher_.publish(front_left_wheel_.get_speedometer_message());
    fr_speedometer_publisher_.publish(front_right_wheel_.get_speedometer_message());
    motor_speedometer_publisher_.publish(motor_.get_speedometer_message());

}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "car_instruments");

    CarInstruments car_instruments;

    ros::spin();

    return 0;
}