#include <ros/ros.h>
// #include <tf2_ros/static_transform_broadcaster.h>
#include "tf2/transform_datatypes.h"
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

#include "car_msgs/ResetOdometer.h"


// based on https://answers.ros.org/question/11232/how-to-turn-laser-scan-to-point-cloud-map/

class OdomTracker {
     public:
        OdomTracker();

        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        bool reset_odom(car_msgs::ResetOdometer::Request  &req, car_msgs::ResetOdometer::Response &res) {
            ROS_INFO("reset odometer");

            try{
                odom_transform_ = tf_buffer_.lookupTransform(
                    "base_link", 
                    "odom",  
                    ros::Time(0));

                odom_transform_.header.frame_id = "map";
                odom_transform_.child_frame_id = "odom";
            }
            catch (tf::TransformException ex){
                ROS_ERROR("could not find map to base_link");
            }

            try{
                lidar_odom_transform_= tf_buffer_.lookupTransform(
                    "lidar_odom_base_link",
                    "lidar_odom",
                    ros::Time(0)
                    );
                lidar_odom_transform_.header.frame_id = "map";
                lidar_odom_transform_.child_frame_id = "lidar_odom";
            }
            catch (tf::TransformException ex){
                ROS_ERROR("could not find map to lidar_base_link");
            }

            return true;
        }

        void send_transforms() {
            auto now = ros::Time::now();

            odom_transform_.header.stamp = now;
            lidar_odom_transform_.header.stamp = now;

            odom_broadcaster_.sendTransform(odom_transform_);
            odom_broadcaster_.sendTransform(lidar_odom_transform_);
        }
        
     // private:
        tf2_ros::TransformBroadcaster odom_broadcaster_;
        // tf2_ros::StaticTransformBroadcaster lidar_odom_broadcaster_;

        geometry_msgs::TransformStamped odom_transform_;
        geometry_msgs::TransformStamped lidar_odom_transform_;

        ros::NodeHandle node_;
        //tf::TransformListener tf_listener_;

        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;

        ros::ServiceServer reset_odom_service_;
};

OdomTracker::OdomTracker()
: tf_listener_(tf_buffer_)
{
    reset_odom_service_ = node_.advertiseService("reset_odom", &OdomTracker::reset_odom, this);

    auto now = ros::Time::now();

    {
        odom_transform_.header.stamp = now;
        odom_transform_.header.frame_id = "map";
        odom_transform_.child_frame_id = "odom";
        odom_transform_.transform.translation.x = 0.0;
        odom_transform_.transform.translation.y = 0.0;
        odom_transform_.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0.0);
        odom_transform_.transform.rotation.x = quat.x();
        odom_transform_.transform.rotation.y = quat.y();
        odom_transform_.transform.rotation.z = quat.z();
        odom_transform_.transform.rotation.w = quat.w();
    }
    {
        lidar_odom_transform_.header.stamp = now;
        lidar_odom_transform_.header.frame_id = "map";
        lidar_odom_transform_.child_frame_id = "lidar_odom";
        lidar_odom_transform_.transform.translation.x = 0.0;
        lidar_odom_transform_.transform.translation.y = 0.0;
        lidar_odom_transform_.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0.0, 0.0, 0.0);
        lidar_odom_transform_.transform.rotation.x = quat.x();
        lidar_odom_transform_.transform.rotation.y = quat.y();
        lidar_odom_transform_.transform.rotation.z = quat.z();
        lidar_odom_transform_.transform.rotation.w = quat.w();
    }

    send_transforms();
}


int main(int argc, char** argv)
{

    ros::init(argc, argv, "odom_tracker");

    OdomTracker tracker;

    ros::Rate rate(10);
    while(ros::ok()) {
        tracker.send_transforms();
        ros::spinOnce();
        rate.sleep();
    }

    

    return 0;
}
