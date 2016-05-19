#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#define ANGLE_SPREAD 180
#define BOT_REFERENCE_X 500
#define BOT_REFERENCE_Y 100   //100 pixels with respect to cartesian coordinates
#define LARGE_VAL 10000

pcl::PointCloud<pcl::PointXYZ>::Ptr addtocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg, cv::Point p)
{
	cloud_msg->points.push_back(pcl::PointXYZ(((p.y-1000)+2*BOT_REFERENCE_Y)*(0.01) + 5.82,(BOT_REFERENCE_X-p.x)*0.01,0));
	return cloud_msg;
}

//cloud_msg->points.push_back(pcl::PointXYZ(((i-img.rows)+2*BOT_REFERENCE_Y)*(0.01) + 5.82,(BOT_REFERENCE_X-j)*0.01,0));