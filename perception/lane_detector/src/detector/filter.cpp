//
//Removes random noise and large white obstacles like cones/barricades
//
#include <laneDetector.hpp>

cv::Mat LaneDetector::filter(cv::Mat &image)
{
	cv::Mat img=image;

	//cv::imshow("lanes", img);


	img=perspective_transform(img);
	//cv::imshow("before filter", img);
	

	int smoothing_threshold=11, obs_threshold=40;
	cv::medianBlur(img,img,smoothing_threshold);

	int cont_white_pixels=0;
	int i, j, k;
	for(i=0;i<img.rows;i++)
	{
		cont_white_pixels=0;
		for(j=0;j<img.cols;j++)
		{
			if(cont_white_pixels>=obs_threshold && (img.at<uchar>(i,j)==0 || j==img.cols-1))
			{	
				for(k=j-cont_white_pixels+1;k<=j;k++)	
					{img.at<uchar>(i,k)=0;}
			}

			if(img.at<uchar>(i,j)==255) cont_white_pixels++;
			else cont_white_pixels=0;
		}
	}

	//cv::medianBlur(img,img,smoothing_threshold); 

	//cv::imshow("after filter", img);
	return inversePerspectiveTransform(img);


}
