#include <lane_navigator.hpp>
//#pragma_once

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

cv::Point laneStart(cv::Mat& output_lane)
{
    int error_margin=35;

    int i, j, flag=0;
    cv::Point lanestart;
    lanestart.x=0;
    lanestart.y=0;

    for(i=output_lane.rows-1;i>=0;i--)
    {
        if(flag>=error_margin) break;

        for(j=0;j<output_lane.cols;j++)
        {
            if(flag>=error_margin) break;
            if(output_lane.at<uchar>(i,j)==255)
            {
                flag++;
                lanestart.x+=j;
                lanestart.y+=i;
            }
        }
    }

    lanestart.x=lanestart.x/error_margin;
    lanestart.y=lanestart.y/error_margin;

    return lanestart;
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
    int iterations=100;
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

    //fit with linear model
    /*std::vector<cv::Vec2f> lines;

    int high=1000, low=1, mid;
    bool stop=false;
    while(high>low)
    {
    	mid=(high+low+1)/2;
    	cv::HoughLines(img, lines, 1, CV_PI/180, mid, 0, 0 );

    	if(lines.size()==0)	high=mid-1;
    	else if(lines.size()>0)	low=mid;
    }

    float rho = lines[0][0], theta = lines[0][1];
    cv::Point pt1, pt2;
    double a = cos(theta), b = sin(theta);
    double x0 = a*rho, y0 = b*rho;
    pt1.x = cvRound(x0 + 1000*(-b));
    pt1.y = cvRound(y0 + 1000*(a));
    pt2.x = cvRound(x0 - 1000*(-b));
    pt2.y = cvRound(y0 - 1000*(a));
     
    line.a=a;
    if(pt1.x-pt2.x==0) line.b=99999;
    else line.b=(pt1.y-pt2.y)/(pt1.x-pt2.x);
    line.c=pt1.y-line.b*pt1.x;


    if(mid>0.95*lane.matches)
    {
        a=line.a;
        b=line.b;
        c=line.c;
        matches=mid;
    }
    else
    {
        a=lane.a;
        b=lane.b;
        c=lane.c;
        matches=lane.matches;
    }*/

    a=lane.a;
    b=lane.b;
    c=lane.c;
    matches=lane.matches;

}


geometry_msgs::Pose2D findTarget(cv::Mat img) {

    geometry_msgs::Pose2D target_pose;
    cv::Point target_point, lane_start;
    float heading;

    float center_height=0.50;	//when 2 lanes are present, distance of target from bot=center_height*9 meters

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

    first_lane_center.x=first_lane.a*(1-center_height)*img.rows*(1-center_height)*img.rows+first_lane.b*(1-center_height)*img.rows+first_lane.c;
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
        
        int flag=0;
        lane_start.x=0;
        lane_start.y=0;
        for(i=img.rows-1;i>=0;i--)
        {
            if(flag>=35) break;
            for(j=0;j<img.cols;j++)
            {
                if(flag>=35) break;
                if(output_first_lane.at<uchar>(i,j)==255)
                {
                    flag++;
                    lane_start.x+=j;
                    lane_start.y+=i;
                    cv::rectangle(final_output, {j-25,i-25}, {j+25,i+25}, 255, 1, 8, 0);
                }
            }
        }

        lane_start.x=lane_start.x/35;
        lane_start.y=lane_start.y/35;
        cv::rectangle(final_output, {lane_start.x-150,lane_start.y-150}, {lane_start.x+150,lane_start.y+150}, 255, 1, 8, 0);
        if(lane_start.x>img.cols/2) is_single_lane_left=false;
        else is_single_lane_left=true;

        /*if(start_frame==true)
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
        }*/

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

            cv::Mat output=output_first_lane+output_second_lane;
            //imshow("Lanes", output);


    		cv::Mat second_curve=img-img;
    		second_curve=second_lane.draw_curve(second_curve);
    		cv::imshow("second curve", second_curve);

    		second_lane_center.x=second_lane.a*(1-center_height)*img.rows*(1-center_height)*img.rows+second_lane.b*(1-center_height)*img.rows+second_lane.c;
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
    }

    //std::cout << "beginning part" << std::endl;


    if(single_lane==false)  //is 2 lanes are present
    {

        std::vector<cv::Vec4i>lines1, lines2;

        int points_thres=500;
        cv::Mat hlines1(img.rows,img.cols,CV_8UC3,cv::Scalar(0,0,0));
        cv::Mat hlines2(img.rows,img.cols,CV_8UC3,cv::Scalar(0,0,0));
        while(lines1.size()==0 && points_thres>=50)
        {
            cv::HoughLinesP(output_first_lane, lines1, 1, CV_PI / 180, points_thres, 15, 5);
            points_thres-=50;
        }
        points_thres=500;
        while(lines2.size()==0 && points_thres>=50)
        {
            cv::HoughLinesP(output_second_lane, lines2, 1, CV_PI / 180, points_thres, 15, 5);
            points_thres-=50;
        }
        

        //hough line tester
        for( size_t i = 0; i < lines1.size(); i++ )
            {
                cv::Vec4i p = lines1[i];
                cv::Point pt1,pt2;
                pt1.x=p[0];
                pt1.y=p[1];
                pt2.x=p[2];
                pt2.y=p[3];

                cv::line( hlines1, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA);
            }

        for( size_t i = 0; i < lines2.size(); i++ )
            {
                cv::Vec4i p = lines2[i];
                cv::Point pt1,pt2;
                pt1.x=p[0];
                pt1.y=p[1];
                pt2.x=p[2];
                pt2.y=p[3];
                cv::line( hlines2, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA);
            }   


        cv::Point center_point1,center_point2,center_point;
        center_point1.x=0;
        center_point1.y=0;
        center_point2.x=0;
        center_point2.y=0;
        center_point.x;
        center_point.y;

        double lane_angle1=0.0, lane_angle2=0.0, lane_angle;

        
        for (int i = 0; i < lines1.size(); i++)
        {
            cv::Vec4i p = lines1[i];
            cv::line(left_lines, cv::Point(p[0], p[1]), cv::Point(p[2], p[3]), cv::Scalar(255, 255, 255), 3, CV_AA);
            center_point1.x += (p[0] + p[2]) / 2;
            center_point1.y += (p[1] + p[3]) / 2;
            if ((p[1] - p[3]) != 0) 
            {
                double theta = std::atan2((1.0*p[1] - p[3]), (p[0] - p[2]));
                if (theta < 0)
                    theta = 3.14 + theta;
                lane_angle1 += (theta);
            }
            else
            {
                lane_angle1+=1.57;
            }
        }
        if(lines1.size()!=0)
        {
        	center_point1.x/=lines1.size();
        	center_point1.y/=lines1.size();
        	lane_angle1/=lines1.size();
    	}
    	else 
    	{
    		center_point1.x=img.cols/2;
        	center_point1.y=img.rows/2;
        	lane_angle1=1.57;
    	}

        for (int i = 0; i < lines2.size(); i++)
        {
            cv::Vec4i p = lines2[i];
            cv::line(right_lines, cv::Point(p[0], p[1]), cv::Point(p[2], p[3]), cv::Scalar(255, 255, 255), 3, CV_AA);
            center_point2.x += (p[0] + p[2]) / 2;
            center_point2.y += (p[1] + p[3]) / 2;
            if ((p[1] - p[3]) != 0) 
            {
                double theta = std::atan2((1.0*p[1] - p[3]), (p[0] - p[2]));
                if (theta < 0)
                    theta = 3.14 + theta;
                lane_angle2 += (theta);
            }
            else
            {
                lane_angle2+=1.57;
            }
        }
        if(lines2.size()!=0)
        {
        	center_point2.x/=lines2.size();
        	center_point2.y/=lines2.size();
        	lane_angle2/=lines2.size();
    	}
    	else 
    	{
    		center_point2.x=img.cols/2;
        	center_point2.y=img.rows/2;
        	lane_angle2=1.57;
    	}


        center_point.x=(center_point1.x+center_point2.x)/2;
        center_point.y=(center_point1.y+center_point2.y)/2;
        lane_angle=(lane_angle1+lane_angle2)/2;
        
        
        cv::Point pt1,pt2;
        pt1.y= center_point.x;
        pt1.x=center_point.y;
        pt2.y=200;
        pt2.x=(int)(center_point.y-((center_point.x-200)/tan(lane_angle)));
        cv::line( head, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA);

        //-------
        //cv::circle(final_output, {center_point.x,center_point.y}, 20, cv::Scalar(255,0,0), 1, 8, 0);
        
        cv::Point bot;
        bot.x=img.cols/2;
        bot.y=900;

        int p2=700;
        //target_point.x=bot.x+p2*cos(atan2(center_point.y-bot.y,center_point.x-bot.x));
        //target_point.y=bot.y+p2*sin(atan2(center_point.y-bot.y,center_point.x-bot.x));
        //cv::circle(final_output, {target_point.x,target_point.y}, 40, cv::Scalar(255,0,0), 1, 8, 0);
        angle = lane_angle;




    }
    else    //if single lane is present
    {

        std::vector<cv::Vec4i>lines;
        cv::Mat hlines(img.rows,img.cols,CV_8UC3,cv::Scalar(0,0,0));
        cv::HoughLinesP(output_first_lane, lines, 1, CV_PI / 180, 25, 15, 5);
        double lane_angle=0.0;
            
        for( size_t i = 0; i < lines.size(); i++ )
            {
                cv::Vec4i p = lines[i];
                cv::Point pt1,pt2;
                pt1.x=p[0];
                pt1.y=p[1];
                pt2.x=p[2];
                pt2.y=p[3];
                cv::line( hlines, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA);
            }   



        //----------code for target perpendicular to lane at some distance

        cv::Point center_point, displaced_point;

        for (int i = 0; i < lines.size(); i++)
        {
            cv::Vec4i p = lines[i];
            center_point.x += (p[0] + p[2]) / 2;
            center_point.y += (p[1] + p[3]) / 2;
            if ((p[1] - p[3]) != 0) 
            {
                double theta = std::atan2((1.0*p[1] - p[3]), (p[0] - p[2]));
                if (theta < 0)
                    theta = 3.14 + theta;
                lane_angle += (theta);
            }
            else
            {
                lane_angle+=1.57;
            }
        }
        if(lines.size()!=0)
        {
        	center_point.x/=lines.size();
        	center_point.y/=lines.size();
        	lane_angle/=lines.size();
    	}
    	else 
    	{
    		center_point.x=img.cols/2;
        	center_point.y=img.rows/2;
        	lane_angle=1.57;
    	}

        

        int p1=200, p2=500;
        if(is_single_lane_left)
        {
            displaced_point.x=center_point.x+p1*cos(3.14/2-lane_angle);
            displaced_point.y=center_point.y+p1*sin(3.14/2-lane_angle);

        }
        else
        {
            displaced_point.x=center_point.x-p1*cos(3.14/2-lane_angle);
            displaced_point.y=center_point.y-p1*sin(3.14/2-lane_angle);
        }

        cv::Point bot;
        bot.x=img.cols/2;
        bot.y=100;

        heading=(1.0*displaced_point.y-bot.y)/(displaced_point.x-bot.x);
        //target_point.x=bot.x+p2*cos(heading);
        //target_point.y=p2*sin(heading);
        //target_point.theta = lane_angle;
        angle = lane_angle;

        //  imshow("houghp",hlines);

        for (int i = 0; i < lines.size(); i++)
        {
            cv::Vec4i p = lines[i];
            if ((p[1] - p[3]) != 0) 
            {
                double theta = std::atan2((1.0*p[1] - p[3]), (p[0] - p[2]));
                if (theta < 0)
                    theta = 3.14 + theta;
                lane_angle += (theta);
            }
            else
            {
                lane_angle+=1.57;
            }
        }
        if(lines.size()!=0)
        {
            lane_angle/=lines.size();
        }
        else 
        {
            lane_angle=1.57;
        }

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
