#include "ros/ros.h"
#include "std_msgs/String.h"

#include "car_controller/car_update.h"
#include "car_controller/car_rc_command.h"


#include <sstream>
#include <thread>

#include "logger.h"
#include "work-queue.h"
#include "usb.h"
#include "string-utils.h"
#include "CarMessages.h"

Usb usb;
WorkQueue<StampedString> usb_queue;
bool quit = false;



using namespace std;

bool get_dynamics_from_line(Dynamics2 &d, const StampedString& msg ) {
  auto split_message = split(msg.message, ',', false);
  if (split_message.size() < 2) return false;

  string message_type = split_message[0];
  string body = trimmed(split_message[1]);
  if (message_type != "TD2") {
    return false;
  }
  
  StringInTransfer stream(body.c_str());
  d.transfer(stream);
  if (stream.ok) {
    return  true;

  } else {
    // ++usb_error_count;
    log_warning((string) "bad dynamics: " + msg.to_string());
    log_warning((string) "message: " + stream.error_message);
  }
  return stream.ok;
}

void rc_command_callback(const car_controller::car_rc_command::ConstPtr& msg)
{
  ROS_INFO("command str_us: [%d] esc_us: [%d]", msg->str_us, msg->esc_us);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_controller");
  ros::NodeHandle n;
  ros::Publisher car_raw_pub = n.advertise<car_controller::car_update>("car_raw", 1000);
  ros::Subscriber sub = n.subscribe("car_rc", 2, rc_command_callback);
  
  ros::Rate loop_rate(100);
  int count = 0;
  car_controller::car_update msg;

  {
  try {
    log_entry_exit w("usb thread");
    usb.add_line_listener(&usb_queue);
    usb.write_on_connect("\ntd+\n");

    // simlink to /dev/ttyA* enabled by adding the
    // following to the end of /etc/udev/rules.d/49-teensy.rules
    // ATTRS{idVendor}=="16c0", , ATTRS{idProduct}=="04[789B]?",
    // SYMLINK+="teensy$attr{serial}"

    string device_path = "/dev/car-controller";
    usb.run(device_path);
    
    while (ros::ok()) {
      ros::spinOnce();
      try {
        static uint32_t processed_count = 0;
        StampedString line;
        if (usb_queue.try_pop(line, 1)) {
          log_warning_if_duration_exceeded w("processing usb line", 10ms);
          size_t remaining = usb_queue.size();
          if(remaining > 0) {
            static uint32_t count = 0;
            count++;
            if(count%100 == 1) {
              log_warning("car usb queue not empty after pop, remaining:  "+to_string(remaining) + " count: " + to_string(count));
            }
            //while(usb_queue.size() > 0) {
            //  // take off excess lines
            //  StampedLine l2;
            //  usb_queue.try_pop(l2, 0);
            //}
          }
          Dynamics2 d;
          if(get_dynamics_from_line(d, line)) {
            car_controller::car_update msg;
            msg.header.stamp = ros::Time::now();

            msg.ms = d.ms;
            msg.us = d.us;
            msg.v_bat = d.v_bat;
            msg.mode = d.mode;
            msg.rx_str = d.rx_str;
            msg.rx_esc = d.rx_esc;
            msg.ax = d.ax;
            msg.ay = d.ay;
            msg.az = d.az;
            msg.spur_us = d.spur_us;
            msg.spur_odo = d.spur_odo;
            msg.odo_fl_a = d.odo_fl_a;
            msg.odo_fl_a_us = d.odo_fl_a_us;
            msg.odo_fl_b = d.odo_fl_b;
            msg.odo_fl_b_us = d.odo_fl_b_us;
            msg.odo_fl_ab_us = d.odo_fl_ab_us;
            msg.odo_fr_a = d.odo_fr_a;
            msg.odo_fr_a_us = d.odo_fr_a_us;
            msg.odo_fr_b = d.odo_fr_b;
            msg.odo_fr_b_us = d.odo_fr_b_us;
            msg.odo_fr_ab_us = d.odo_fr_ab_us;
            msg.mpu_deg_yaw = d.mpu_deg_yaw;
            msg.mpu_deg_pitch = d.mpu_deg_pitch;
            msg.mpu_deg_roll = d.mpu_deg_roll;
            msg.mpu_deg_f = d.mpu_deg_f;
            msg.go = d.go;
          
            car_raw_pub.publish(msg);
          }
          ++processed_count;
        }

      } catch (string error_string) {
        log_error("error caught in usb loop" + error_string);
      } catch (std::exception& e) {
        log_error((string) "std::exception caught in usb loop" + e.what());
      } catch (...) {
        log_error("error caught in usb loop");
      }
    }
  } catch (string error_string) {
    log_error("error caught in usb_thread_start" + error_string);
  } catch (...) {
    log_error("unknown error caught in usb_thread_start");
  }
}


  return 0;
}
