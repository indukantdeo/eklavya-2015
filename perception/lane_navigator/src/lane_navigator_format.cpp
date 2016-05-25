#include <lane_navigator.hpp>
//#pragma_once

#include <opencv2/highgui/highgui.hpp>
#include <ros/package.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ros/conversions.h>

float center_height=0.50;   //when 2 lanes are present, distance of target from bot=center_height*9 meters

#define ANGLE_SPREAD 180
#define BOT_REFERENCE_X 500
#define BOT_REFERENCE_Y 100   //100 pixels with respect to cartesian coordinates
#define LARGE_VAL 10000

pcl::PointCloud<pcl::PointXYZ>::Ptr addtocloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_msg, cv::Point p)
{
    cloud_msg->points.push_back(pcl::PointXYZ(((p.y-1000)+2*BOT_REFERENCE_Y)*(0.01) + 5.82,(BOT_REFERENCE_X-p.x)*0.01,0));
    return cloud_msg;
}
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
PointCloud::Ptr lane_cloud(new PointCloud);

ros::Publisher cloud_pub;


const int bot_x = 500, bot_y = 900;
int step_move = -700;
const cv::Point origin(0, 480); //Wrt top left corner
int count = 0;
int debug=1;
ros::Publisher pub_point;
geometry_msgs::Pose2D msge;
geometry_msgs::PoseStamped new_msg;
//queue<geometry_msgs::Pose2D> temp;
int counter=0;
int mid_point=500;

int pre_left_intercept, pre_right_intercept;
bool is_previous_single=false, is_previous_left;
bool start_frame=true;


geometry_msgs::Pose2D* temp=new geometry_msgs::Pose2D [3];

geometry_msgs::PoseStamped convert_Pose2D_to_PoseStamped(geometry_msgs::Pose2D pose2d){
    geometry_msgs::PoseStamped pose_stamp;


    pose_stamp.pose.position.x = pose2d.x;
    pose_stamp.pose.position.y = pose2d.y;
    pose_stamp.pose.position.z = 0;
    pose_stamp.header.frame_id = "base_link";
    tf::Quaternion frame_quat;
    frame_quat=tf::createQuaternionFromYaw(pose2d.theta);

    pose_stamp.pose.orientation.x=frame_quat.x();
    pose_stamp.pose.orientation.y=frame_quat.y();
    pose_stamp.pose.orientation.z=frame_quat.z();
    pose_stamp.pose.orientation.w=frame_quat.w();

    return pose_stamp;
}

void update_pointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  lane_cloud=cloud;

  std::cout << "Entered update cloud" << std::endl << std::endl << std::endl;
}

float laneAngle(cv::Mat lane)
{
    cv::Point lane_start, lane_end;
    lane_start=laneStart(lane);
    cv::flip(lane, lane, 0);
    lane_end=laneStart(lane);
    lane_end.y=lane.rows-lane_end.y;

    return std::atan2(lane_start.y - lane_end.y, lane_start.x - lane_end.x);
}

cv::Point laneStart(cv::Mat& output_lane)
{
    int error_margin=35;

    int i, j, counter=0;
    cv::Point lanestart;
    lanestart.x=0;
    lanestart.y=0;

    for(i=output_lane.rows-1;i>=0;i--)
    {
        if(counter>=error_margin) break;

        for(j=0;j<output_lane.cols;j++)
        {
            if(counter>=error_margin) break;
            if(output_lane.at<uchar>(i,j)==255)
            {
                counter+;
                lanestart.x+=j;
                lanestart.y+=i;
            }
        }
    }

    lanestart.x=lanestart.x/error_margin;
    lanestart.y=lanestart.y/error_margin;

    return lanestart;
}

int quadratic_curve::point(int y)
{
    return a*y*y+b*y+c;
}

cv::Mat quadratic_curve::draw_curve(cv::Mat img)
{
    for(size_t i=0;i<img.rows;i++)
        for(size_t j=0;j<img.cols;j++)
            if(on_curve({j,i})==true) img.at<uchar>(i,j)=255;

    return img;
}

void quadratic_curve::compute_quadratic(co_ord p1, co_ord p2, co_ord p3)
{
    if(p1.y==p2.y || p1.y==p3.y || p2.y==p3.y)    //error check for denominator in expression below
    {
        a=0;
        b=0;
        c=0;
        return;
    }

    
    a=( ((float)p1.x-p2.x)/(p1.y-p2.y)- ((float)p2.x-p3.x)/(p2.y-p3.y) )/((float)p1.y-p3.y);
    b=( ((float)p1.x-p2.x)- a*(p1.y*p1.y- p2.y*p2.y) )/((float)p1.y-p2.y);
    c=p1.x-a*p1.y*p1.y-b*p1.y;
    return ;
        
}

bool quadratic_curve::on_curve(co_ord p) //checks if given point lies on curve
{
    float error_margin=35;
    if(abs(a*p.y*p.y+b*p.y+c-p.x)<error_margin) return true;
    else return false;
}

int quadratic_curve::x_intercept(cv::Mat img)
{
    return (a*img.rows*img.rows+b*img.rows+c);
}

void quadratic_curve::ransac(std::vector<co_ord>& points, cv::Mat &img)
{
	//fit with quadratic model
    quadratic_curve lane, temp_lane, line, temp_line;
    lane.matches=-1;
    co_ord p1, p2, p3;
    int iterations=200;
    srand(12);

    while(iterations-- && points.size()!=0) 
    {
        //randomly select three distinct points
        do
        {
            p1=points[rand()%points.size()];
            p2=points[rand()%points.size()];
            p3=points[rand()%points.size()];
        }
        while(p1.y==p2.y || p1.y==p3.y || p2.y==p3.y);
        //compute curve corresponding to this three points
        temp_lane.compute_quadratic(p1,p2,p3);
        //find number of points lying on this curve
        temp_lane.matches=0;
        for(size_t i=0;i<points.size();i++)
            if(temp_lane.on_curve(points[i])==true) temp_lane.matches++;
        //if curve is better than current best, update current best
        if(temp_lane.matches>=lane.matches)
        {
            lane=temp_lane;
        }
    }

    iterations=100;
    int error_margin=25;
    line.matches=-1;
    while(iterations-- && points.size()!=0) 
    {
        //randomly select three distinct points
        do
        {
            p1=points[rand()%points.size()];
            p2=points[rand()%points.size()];
        }
        while(p1.y==p2.y);
        //compute curve corresponding to these 2 points
        temp_line.a=0;
        temp_line.b=(p1.x-p2.x)/(p1.y-p2.y);
        temp_line.c=p1.x-temp_line.b*p1.y;
        //find number of points lying on this curve
        temp_line.matches=0;
        for(size_t i=0;i<points.size();i++)
            if(abs(points[i].x-temp_line.b*points[i].y-temp_line.c)<error_margin) temp_line.matches++;

        //if line is better than current best, update current best
        if(temp_line.matches>=line.matches)
        {
            line=temp_line;
        }
    }

    if(line.matches>0.8*lane.matches)
    {
        a=line.a;
        b=line.b;
        c=line.c;
        matches=line.matches;
    }
    else
    {
        a=lane.a;
        b=lane.b;
        c=lane.c;
        matches=lane.matches;
    }

}


geometry_msgs::Pose2D findTarget(cv::Mat img) {

    geometry_msgs::Pose2D target_pose;
    cv::Point target_point, lane_start;
    float heading;

    cv:: Point first_lane_center, second_lane_center, lane_center;
    bool single_lane;
    bool is_single_lane_left;
    cv:: Point lane_intercept;
	float angle;

    cv::Mat head(img.rows,img.cols,CV_8UC3,cv::Scalar(0,0,0));

    cv::Mat output_first_lane=img-img;
    cv::Mat output_second_lane=img-img;

    //find if image has a single lane

    cv::Mat final_output=img.clone();
    cv::Mat original= img,left_lines=img-img, right_lines=img-img;

    
    //extract white pixels in vector points
    std::vector<co_ord> points;
    int i, j, k;
    for(i=0;i<img.rows;i++)
        for(j=0;j<img.cols;j++)
        {
            if(img.at<uchar>(i,j)>=20) points.push_back({j,i});
        }

    int white_points=points.size();

    if(points.size()==0)    //no white pixel in image- give target at 2.25	 meters straight ahead
    {
        target_pose.x=img.cols/2;
        target_pose.y=img.rows/2;
        target_pose.theta = 1.57;
        cv::circle(final_output, {target_point.x,target_point.y}, 40, cv::Scalar(255,0,0), 1, 8, 0);
        imshow("waypoint", final_output);
        return target_pose;
    }

    quadratic_curve first_lane;
    first_lane.ransac(points, img);


    //mark the points which lie on the best curve and remove these points from original image
    cv::Mat first_lane_removed=img.clone();
    for(i=0;i<points.size();i++)
        if(first_lane.on_curve(points[i])==true)
        {
            output_first_lane.at<uchar>(points[i].y, points[i].x)=255;
            first_lane_removed.at<uchar>(points[i].y,points[i].x)=0;
        }

    //imshow("image after removing first lane", first_lane_removed);
    cv::imshow("first lane", output_first_lane);

    cv::Mat first_curve=img-img;
    first_curve=first_lane.draw_curve(first_curve);
    cv::imshow("first curve", first_curve);

    first_lane_center.x=first_lane.point((1-center_height)*img.rows);
    first_lane_center.y=(1-center_height)*img.rows;

    cv::circle(final_output, first_lane_center, 30, cv::Scalar(122,0,0), 1, 8, 0);

    points.clear();
    for(i=0;i<first_lane_removed.rows;i++)
        for(j=0;j<first_lane_removed.cols;j++)
        {
            if(first_lane_removed.at<uchar>(i,j)>=20) points.push_back({j,i});
        }

    quadratic_curve second_lane;
    second_lane.ransac(points, first_lane_removed);

    int threshold=4000;
    if(second_lane.matches<=0.1*white_points) //pixels detected in second lane below certain threshold i.e. only one lane in image
    {
        single_lane=true;
        
        
        //if(laneStart(output_first_lane)>img.cols/2) is_single_lane_left=false;
        //else is_single_lane_left=true;

        if(start_frame==true)
        {
            if(first_lane.x_intercept(img)<img.cols/2) is_single_lane_left=true;
            else is_single_lane_left=false;
        }
        else if(is_previous_single==true)
        {
            if(is_previous_left==true) is_single_lane_left=true;
            else is_single_lane_left=false;
        }
        else
        {
            if(abs(pre_left_intercept-first_lane.x_intercept(img))<abs(pre_right_intercept-first_lane.x_intercept(img)))
                is_single_lane_left=true;
            else false;
        }

        is_previous_single=true;
        if(is_single_lane_left==true) is_previous_left=true;
        else is_previous_left=false;

        cv::Mat output=output_first_lane;
        //imshow("Lanes", output);
    }
    else    //two lanes have been detected: draw best fit curve on second_curve and mark the height=point at center_height*img.rows as second_lane center
    {
    		single_lane=false;

            //mark the points which lie on the best curve
            for(i=0;i<points.size();i++)
                if(second_lane.on_curve(points[i])==true)
                {
                    output_second_lane.at<uchar>(points[i].y, points[i].x)=255;
                }
            //imshow("second lane", output_second_lane);

            //cv::Mat output=output_first_lane+output_second_lane;
            //imshow("Lanes", output);

            cv::Mat second_curve=img-img;
    		second_curve=second_lane.draw_curve(second_curve);
    		cv::imshow("second curve", second_curve);

            second_lane_center.x=second_lane.point((1-center_height)*img.rows);
    		second_lane_center.y=(1-center_height)*img.rows;
    		cv::circle(final_output, second_lane_center, 30, cv::Scalar(122,0,0), 1, 8, 0);

            //lane center is midpoint of first lane center and second lane center i.e target position
    		lane_center.x=(first_lane_center.x+second_lane_center.x)/2;
    		lane_center.y=(first_lane_center.y+second_lane_center.y)/2;
    		cv::circle(final_output, lane_center, 50, cv::Scalar(122,0,0), 1, 8, 0);

            if(first_lane.x_intercept(img)>second_lane.x_intercept(img))
            {
                pre_right_intercept=first_lane.x_intercept(img);
                pre_left_intercept=second_lane.x_intercept(img);
            }
            else
            {
                pre_left_intercept=first_lane.x_intercept(img);
                pre_right_intercept=second_lane.x_intercept(img);
            }
            is_previous_left=false;

            angle=(laneAngle(output_first_lane)+laneAngle(output_second_lane))/2;
    }

    //std::cout << "beginning part" << std::endl;


    
    if(single_lane==true)    //if single lane is present
    {

        angle=laneAngle(output_first_lane);

        int shift=200, target_height=130;

        if(is_single_lane_left)
        {
            //target_point.x=(lane_intercept.x+shift)+abs(target_height*cos(lane_angle));
            //target_point.y=(lane_intercept.y)-abs(target_height*sin(lane_angle));

            //target_point.x=(lane_start.x+shift)+abs(target_height*cos(lane_angle));
            //target_point.y=(lane_start.y)-abs(target_height*sin(lane_angle));

            target_point.x=(lane_start.x+abs(shift*sin(lane_angle)))+abs(target_height*cos(lane_angle));
            target_point.y=(lane_start.y+shift*cos(lane_angle))-abs(target_height*sin(lane_angle));

        }
        else
        {
            //target_point.x=(lane_intercept.x-shift)-abs(target_height*cos(lane_angle));
            //target_point.y=(lane_intercept.y)-abs(target_height*sin(lane_angle));

            //target_point.x=(lane_start.x-shift)-abs(target_height*cos(lane_angle));
            //target_point.y=(lane_start.y)-abs(target_height*sin(lane_angle));

            target_point.x=(lane_start.x-abs(shift*sin(lane_angle)))+abs(target_height*cos(lane_angle));
            target_point.y=(lane_start.y-shift*cos(lane_angle))-abs(target_height*sin(lane_angle));
        }

        //std::cout<<"lane intercept: "<<lane_intercept.x<<" "<<lane_intercept.y<<std::endl;
        // std::cout<<"target point: "<<target_point.x<<" "<<target_point.y<<std::endl;
        //cv::circle(final_output, lane_intercept, 30, cv::Scalar(122,0,0), 1, 8, 0);
        cv::circle(final_output, lane_start, 30, cv::Scalar(122,0,0), 1, 8, 0);
        cv::circle(final_output, target_point, 50, cv::Scalar(122,0,0), 1, 8, 0);
        //cv::line(final_output, lane_intercept, target_point, 255, 10, 8, 0);
        cv::line(final_output, lane_start, target_point, 255, 10, 8, 0);
        lane_center.x=target_point.x;
        lane_center.y=target_point.y;
        angle=lane_angle;


    }

   // cout<<target_point.x<<" "<<target_point.y<<endl;
   //cv::circle(final_output, {target_point.x,target_point.y}, 50, cv::Scalar(122,0,0), 1, 8, 0);
    imshow("right_lines",right_lines);
    imshow("left_lines",left_lines);
    imshow("heading", head);


    //std::cout << "exit" << std::endl;

    
   // waitKey(0);
   // return 0;

    //target_pose.x = target_point.x;
    //target_pose.y = target_point.y;
    target_pose.x=lane_center.x;
    target_pose.y=lane_center.y;
    target_pose.theta = angle-CV_PI/2;
    //target_pose.theta = CV_PI+angle;

    std::cout<<target_pose.x<<" "<<target_pose.y<<std::endl;

    start_frame=false;

    //
    //Add additional lanes
    // 
    if(single_lane==false)
    {
        cv::Point left_start, right_start;

        if(first_lane.x_intercept(img)>second_lane.x_intercept(img))
        {
            left_start=laneStart(output_second_lane);
            right_start=laneStart(output_first_lane);
        }
        else
        {
            right_start=laneStart(output_second_lane);
            left_start=laneStart(output_first_lane);
        }

        for(i=left_start.y;i<img.rows;i++)
            {
                lane_cloud=addtocloud(lane_cloud, {left_start.x-(i-left_start.y),i});
                //std::cout<<lane_cloud->points.size()<<std::endl;
            }

        for(i=right_start.y;i<img.rows;i++)
            lane_cloud=addtocloud(lane_cloud, {right_start.x+(i-right_start.y),i});

        cv::circle(final_output, left_start, 30, cv::Scalar(255,0,0), 1, 8, 0);
        cv::circle(final_output, right_start, 30, cv::Scalar(255,0,0), 1, 8, 0);

        cv::circle(final_output, {left_start.x-(img.rows-left_start.y),img.rows}, 70, cv::Scalar(255,0,0), 1, 8, 0);
        cv::circle(final_output, {right_start.x+(img.rows-right_start.y),img.rows}, 70, cv::Scalar(255,0,0), 1, 8, 0);
        imshow("waypoint", final_output);

        for(i=img.rows;i<1.5*img.rows;i++)
        {
            lane_cloud=addtocloud(lane_cloud, {left_start.x-(img.rows-left_start.y),i});
            lane_cloud=addtocloud(lane_cloud, {right_start.x+(img.rows-right_start.y),i});
        }

        for(j=left_start.x-(img.rows-left_start.y);j<right_start.x+(img.rows-right_start.y);j++)
        {
            lane_cloud=addtocloud(lane_cloud, {j,1.5*img.rows});
        }

        /*for(i=0;i<img.rows;i++)
            for(j=0;j<img.cols;j++)
                lane_cloud=addtocloud(lane_cloud, {i,j});*/
    }


    return target_pose;
   

}

void publishTarget(const sensor_msgs::ImageConstPtr msg ) {
    if (debug){ROS_INFO("Listened for the %d time\n", count++);
    cv::namedWindow("listening", CV_WINDOW_AUTOSIZE);}
    cv::Mat img;


    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
    // img= cv_bridge.imgMsgToCv(msge, "mono8");
    // cv::cvtColor(msg.image,img,CV_BGR2GRAY);
    img = cv_ptr->image;

    std::cout<<"before findTarget"<<lane_cloud->points.size()<<std::endl;
    msge = findTarget(img);

    std::cout<<lane_cloud->points[lane_cloud->points.size()-1].x<<" "<<lane_cloud->points[lane_cloud->points.size()-1].y<<std::endl;

    std::cout<<"before publish: "<<lane_cloud->points.size()<<" "<<lane_cloud->height<<" "<<lane_cloud->width<<std::endl;
    lane_cloud->width = lane_cloud->points.size();
    cloud_pub.publish(lane_cloud);
    std::cout<<"after publish"<<lane_cloud->points.size()<<std::endl;
    geometry_msgs::Pose2D temp1,temp2;
    temp1=msge;
    for(int i=0;i<3;i++)
    {
        temp2=temp[i];
        temp[i]=temp1;
        temp1=temp2;
    }
    if(debug==5)
    {
        msge.x=500;
        msge.y=250;
        msge.theta=0;
    }
    double temp11=msge.x/100;
    //msge.x=msge.y/100;
    msge.x=(900-msge.y)/100;
    msge.y= 5 -temp11;
    new_msg = convert_Pose2D_to_PoseStamped(msge);

  std::cout << "Entered publish target" << std::endl << std::endl << std::endl;

    pub_point.publish(new_msg);
    if(debug)
    {
       cv::waitKey(33);
       ROS_INFO("%f %f %f ", msge.x, msge.y, msge.theta);
    }
    if(counter<3)
        counter++;
}



int main(int argc, char **argv) {
    std::string node_name= "lane_navigator";
    ros::init(argc, argv, node_name);
    ros::NodeHandle node_handle;
    msge.x=0;
    msge.y=0;
    msge.theta=0;
    pub_point = node_handle.advertise<geometry_msgs::PoseStamped>("/lane_navigator/intermediate_target", 10);
    ros::Subscriber lanes_subscriber = node_handle.subscribe("/lane_detector1/lanes", 10, &publishTarget);

    ros::Subscriber point_cloud_subscriber = node_handle.subscribe("cloud_data", 10, update_pointcloud);
    cloud_pub = node_handle.advertise<pcl::PointCloud<pcl::PointXYZ> >("/stitched_cloud_data", 10);

    while(ros::ok()){
    node_handle.getParam(node_name + "/debug", debug);
    ros::spinOnce();
    //cloud_pub.publish(lane_cloud);
    }
    return 0;
}
