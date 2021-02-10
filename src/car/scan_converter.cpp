#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>


// based on https://answers.ros.org/question/11232/how-to-turn-laser-scan-to-point-cloud-map/

class My_Filter {
     public:
        My_Filter();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
     //private:
        ros::NodeHandle node_;
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;

        ros::Publisher point_cloud_publisher_;
        ros::Subscriber scan_sub_;
};

My_Filter::My_Filter(){
    const int queue_length = 1; // ensure we only process the latest message
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", queue_length, &My_Filter::scanCallback, this);
    point_cloud_publisher_ = node_.advertise<sensor_msgs::PointCloud2> ("/cloud", 1, false);
}

void My_Filter::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    sensor_msgs::PointCloud2 cloud;
    projector_.transformLaserScanToPointCloud("base_link", *scan, cloud, tfListener_);
    point_cloud_publisher_.publish(cloud);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_converter");

    My_Filter filter;

    ros::spin();

    return 0;
}
