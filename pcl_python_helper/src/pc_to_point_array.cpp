
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_python_helper/PointArray2d.h>


using namespace std;

ros::Publisher point_array_pub;

 void cloud_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input){

    pcl_python_helper::PointArray2d points;
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input,pcl_pc2);
    pcl::PointCloud<pcl::PointXY>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXY>);
    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);  
    vector<vector<double>> xyzVal;
    vector<double> xVal,yVal;
    for (size_t i =0; i < temp_cloud -> size()/2; ++i){
        xVal.push_back(temp_cloud->points[i].x);
        yVal.push_back(temp_cloud->points[i].y);
    }
    
    points.header.stamp = ros::Time::now();
    points.x = xVal;
    points.y = yVal;
    point_array_pub.publish(points);
 }

int main (int argc, char** argv) {
     ros::init (argc, argv, "pcl_point_array_converter");
     ros::NodeHandle nh;
     ros::Rate loop_rate(10);
     ros::Subscriber sub;
     sub = nh.subscribe ("/scan_matched_points2", 1, cloud_cb);
     point_array_pub = nh.advertise<pcl_python_helper::PointArray2d>("points", 100);
     ROS_INFO("publishing points\r");
     ros::spin();
 }
 