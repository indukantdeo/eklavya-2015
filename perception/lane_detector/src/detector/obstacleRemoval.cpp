#include <laneDetector.hpp>

static int obstacle_removal_dilation_size = 30; //variable used for dilating and eroding.. to be changed only if dimension of image changes.
static int obstacle_removal_hue = 25; //used to remove obstacle, change only after calibration.
static int obstacle_removal_saturation = 100; //used to remove obstacle, change only after calibration.

int min1threshold=0, max1threshold=10, min2threshold=160, max2threshold=360;

int rgb2hsv(float &h, float &s, float &v, int r, int g, int b) {
    int rgbMin, rgbMax;

    rgbMin = r < g ? (r < b ? r : b) : (g < b ? g : b);
    rgbMax = r > g ? (r > b ? r : b) : (g > b ? g : b);

    v = rgbMax;
    if (v == 0) {
        h = 0;
        s = 0;
        return -1;
    }
    s = 255 * long(rgbMax - rgbMin) / v;
    if (s == 0) {
        h = 0;
        return -2;
    }

    if (rgbMax == r) {
        h = 0 + 43 * (g - b) / (rgbMax - rgbMin);
    } else if (rgbMax == g) {
        h = 85 + 43 * (b - r) / (rgbMax - rgbMin);
    } else {
        h = 171 + 43 * (r - g) / (rgbMax - rgbMin);
    }

    return 0;
}

cv::Mat ObstacleRemovedBinary(cv::Mat &image) {
    cv::Mat binary_after_HSV_thresholding(image.rows, image.cols, CV_8UC1, cv::Scalar(0, 0, 0));
    float h = 0, s = 0, v = 0;
    for (int i = 0; i < image.rows; i++) {
        for (int j = 0; j < image.cols; j++) {
            if (image.at<cv::Vec3b>(i, j)[0] && image.at<cv::Vec3b>(i, j)[1] && image.at<cv::Vec3b>(i, j)[2]) {
                rgb2hsv(h, s, v, (int) image.at<cv::Vec3b>(i, j)[2], (int) image.at<cv::Vec3b>(i, j)[1], (int) image.at<cv::Vec3b>(i, j)[0]);
                if (h < obstacle_removal_hue && s > obstacle_removal_saturation) {
                    binary_after_HSV_thresholding.at<uchar>(i, j) = 255;
                }
            }
        }
    }
    return binary_after_HSV_thresholding;
}

cv::Mat LaneDetector::obstacleRemoval(cv::Mat &image) {
    /*(cv::Mat img_HSV(image.rows, image.cols, CV_8UC3);
    cv::Mat binary_after_HSV_thresholding(image.rows, image.cols, CV_8UC1);
    cv::Mat binary_dialated(image.rows, image.cols, CV_8UC1);

    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                cv::Size(2 * obstacle_removal_dilation_size + 1, 2 * obstacle_removal_dilation_size + 1),
                                                cv::Point(obstacle_removal_dilation_size, obstacle_removal_dilation_size));
    binary_after_HSV_thresholding = ObstacleRemovedBinary(image);
    cv::dilate(binary_after_HSV_thresholding, binary_dialated, element);
    for (int i = 0; i < binary_dialated.rows; i++) {
        for (int j = 0; j < binary_dialated.cols; j++) {
            image.at<cv::Vec3b>(i, j)[0] = (255 - binary_dialated.at<uchar>(i, j)) & image.at<cv::Vec3b>(i, j)[0];
            image.at<cv::Vec3b>(i, j)[1] = (255 - binary_dialated.at<uchar>(i, j)) & image.at<cv::Vec3b>(i, j)[1];
            image.at<cv::Vec3b>(i, j)[2] = (255 - binary_dialated.at<uchar>(i, j)) & image.at<cv::Vec3b>(i, j)[2];
        }
    }
    return image;*/

    //return image;

    cv::Mat img;
    int i, j;

    cv::cvtColor(image,img,CV_BGR2HSV);
    cv::imshow("before obstacles removal",img);

    cv::Mat channels[3];
    cv::split(img, channels);
    imshow("hue", channels[0]);

    cv::Mat cones(img.rows, img.cols, CV_8UC1, cv::Scalar(0));

    cv::namedWindow("hue_control_box", 1);
    cv::createTrackbar("min1threshold", "hue_control_box", &min1threshold, 255);
    cv::createTrackbar("max1threshold", "hue_control_box", &max1threshold, 360);

    cv::createTrackbar("min2threshold", "hue_control_box", &min2threshold, 255);
    cv::createTrackbar("max2threshold", "hue_control_box", &max2threshold, 360);
 
    for(i=0;i<img.rows;i++)
        for(j=0;j<img.cols;j++)
            {
                if(img.at<cv::Vec3b>(i,j)[0]>min1threshold && img.at<cv::Vec3b>(i,j)[0]<max1threshold)
                    cones.at<uchar>(i,j)=255;

                if(img.at<cv::Vec3b>(i,j)[0]>min2threshold && img.at<cv::Vec3b>(i,j)[0]<max2threshold)
                    cones.at<uchar>(i,j)=255;
            }

    cv::imshow("cones before median blur", cones);
    cv::medianBlur(cones,cones, 15);

    cv::imshow("cones before erosion", cones);

    //cv::erode(cones, cones, cv::Mat(), cv::Point(-1,-1), 20, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue() );
    cv::dilate(cones, cones, cv::Mat(), cv::Point(-1,-1), 10, cv::BORDER_CONSTANT, cv::morphologyDefaultBorderValue());

    cv::imshow("cones", cones);

    cv::SimpleBlobDetector::Params params;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = 100;
    params.maxArea=1000000000;

    // Change thresholds
    params.minThreshold = 50;
    params.maxThreshold = 255;
    params.thresholdStep= 10;

    //merge close by blobs
    params.minDistBetweenBlobs=100;

    params.filterByInertia = false;
    params.filterByConvexity = false;
    params.filterByColor = false;
    params.filterByCircularity = false;

    cv::SimpleBlobDetector detector(params);
    std::vector<cv::KeyPoint> obs;

    detector.detect( cones, obs);

    std::cout<<obs.size()<<std::endl;

    //cv::Mat blobs=image-image,circle;
    //cv::drawKeypoints( blobs, obs, circle, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DEFAULT );
    //cv::imshow("blobs", circle);

    for(size_t i=0;i<obs.size();i++)
    {
        cv::Point corners[1][20];
        corners[0][0]=cv::Point(obs[i].pt.x-obs[i].size, obs[i].pt.y-2*obs[i].size);
        corners[0][1]=cv::Point(obs[i].pt.x-obs[i].size, obs[i].pt.y+2*obs[i].size);
        corners[0][2]=cv::Point(obs[i].pt.x+obs[i].size, obs[i].pt.y+2*obs[i].size);
        corners[0][3]=cv::Point(obs[i].pt.x+obs[i].size, obs[i].pt.y-2*obs[i].size);

        /*corners[0][0]=cv::Point(obs[i].pt.x-obs[i].size, obs[i].pt.y-obs[i].size);
        corners[0][1]=cv::Point(obs[i].pt.x-obs[i].size, obs[i].pt.y+obs[i].size);
        corners[0][2]=cv::Point(obs[i].pt.x+obs[i].size, obs[i].pt.y+obs[i].size);
        corners[0][3]=cv::Point(obs[i].pt.x+obs[i].size, obs[i].pt.y-obs[i].size);*/

        const cv::Point* rect[1]={corners[0]};
        int num_corners[] = { 4 };

        cv::fillPoly(image, rect, num_corners, 1, cv::Scalar(0,0,0), 8);
    }

    //channel[0]=img_hue;
    //cv::merge(channel,3, image);
    //cv::cvtColor(img_hsv,image,CV_HSV2BGR);

    cv::imshow("after obstacles removal",image);

    return image;

}

void ReadParameterFromFile() {
    static bool read_parameters = false;
    if (!read_parameters) {
        std::cout << "Reading Obstacle Removal Parameters from file" << std::endl;
    }
    return;
}
