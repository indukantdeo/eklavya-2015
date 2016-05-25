#include <cmath>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <queue>
#include <tf/transform_datatypes.h>
#include <ros/package.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

#include <vector>
#include <sensor_msgs/LaserScan.h>
#include <stack>

struct co_ord_ {int x;int y;};
typedef struct co_ord_ co_ord;

cv::Point laneStart(cv::Mat& output_lane);

pcl::PointCloud<pcl::PointXYZ>::Ptr addtocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg, cv::Point p);


class quadratic_curve
{
public:
	float a;
	float b;
	float c;
	int matches;
	int x_intercept(cv::Mat img);
	cv::Mat mark_points();
	cv::Mat draw_curve(cv::Mat);
	void compute_quadratic(co_ord p1, co_ord p2, co_ord p3);
	bool on_curve(co_ord p);
	void ransac(std::vector<co_ord>& points, cv::Mat &img);
	int point(int y);
};