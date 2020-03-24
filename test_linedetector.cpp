#include <iostream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "def_assert.h"
#include "TIMER.h"
#include "LineDetector.hpp"

int main(int argc, char* argv[]){
    // my method compare with my method.
    cv::Mat src, src_gray, dst;
    std::string image_path = "/home/lut/Pictures/lut_dataset/sc_m0.jpg";
	src = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    while(src.rows >=1080){
        cv::resize(src, src, src.size()/2);
    }
	if (!src.data) 
	{
		std::cerr << "无法打开图像文件！" << std::endl;
		return -1;
	}

	cv::namedWindow("input image", CV_WINDOW_AUTOSIZE);
	cv::namedWindow("hough-line-detection", CV_WINDOW_AUTOSIZE);
	cv::imshow("input image", src);

	//边缘检测
	TIMER time_canny("canny cost.");
	time_canny.start_timer();
	cv::Canny(src, src_gray, 150, 200);
	time_canny.end_timer();
	std::cout<<time_canny<<std::endl;// 检测2.5-3.0ms
	//灰度化
	TIMER time_lutmethod("lutMethod cost.");
	time_lutmethod.start_timer();
	std::vector<lut_detector::Point> pxes;
	pxes.reserve(2000);
	float max_dis = 0;
	float min_dis = std::sqrt(src.rows*src.rows+src.cols*src.cols);

	for(size_t r =0; r < src_gray.rows; r++){
		for(size_t c=0; c<src_gray.cols; c++){
			if(src_gray.at<uchar>(r, c) > 200){
				pxes.push_back(lut_detector::Point(c, r));
				float cur_dis = std::sqrt(c*c+r*r);
				if( cur_dis > max_dis)
					max_dis = cur_dis;
				if( cur_dis < min_dis)
					min_dis = cur_dis;
			}
		}
	}
	// pxes.clear();
	// pxes.push_back(lut_detector::Point(500,500));
	std::cout<<"px size is: "<<pxes.size()<<std::endl;
	lut_detector::LineDetector line_detector;
	std::vector<size_t> rt_rect;
	size_t w, h;
	line_detector.detector(pxes, -max_dis, max_dis, 0, rt_rect, w, h);
	time_lutmethod.end_timer();
	std::cout<<time_lutmethod<<std::endl;
	// return 0;s
	int max_counter = 0;
	cv::Mat myIMG(h, w, CV_8UC1);
	for(int i=0; i<h; i++){
		for(int j=0; j<w; j++){
			if(max_counter < rt_rect[i*w+j])
				max_counter = rt_rect[i*w+j];
		}
	}
	float scale = 255.f/max_counter;
	// change to cv::Mat
	for(int i=0; i<h; i++){
		for(int j=0; j<w; j++){
			// std::cout<<rt_rect[i*w+j]<<", ";
			rt_rect[i*w+j] *= scale;
			// std::cout<<scale<<std::endl;
			myIMG.at<uchar>(i, j) = rt_rect[i*w+j];
		}
		// std::cout<<std::endl;
	}
	// rt_rect[500*10+100] = 255;
	// std::memcpy(myIMG.data, rt_rect.data(), rt_rect.size());
	cv::imshow("lutImg", myIMG);
	cv::waitKey(0);
	return 0;
	cv::cvtColor(src_gray, dst, CV_GRAY2BGR);
	cv::imshow("src_gray", src_gray);
	std::vector<cv::Vec2f> lines;
	//霍夫曼直线检测
	cv::Point pt1, pt2;
	TIMER time_hough("hough line cost.");
	time_hough.start_timer();
	cv::HoughLines(src_gray, lines, 1, CV_PI / 180, 150, 0, 0);
	time_hough.end_timer();
	std::cout<<time_hough<<std::endl;// time cost 18ms, cost too much.
	for (size_t i = 0; i < lines.size(); i++)
	{
		// 极坐标中的r长度detector
		float rho = lines[i][0]; 
		// 极坐标中的角度
		float theta = lines[i][1]; 
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		// 转换为平面坐标的四个点
		pt1.x = cvRound(x0 + 1000 * (-b));
		pt1.y = cvRound(y0 + 1000 * (a));
		pt2.x = cvRound(x0 - 1000 * (-b));
		pt2.y = cvRound(y0 - 1000 * (a));
		cv::line(dst, pt1, pt2, cv::Scalar(0, 0, 255), 1, CV_AA);
	}
	
	// 然后对交点进行聚类的时间
	cv::imshow("matt", src);
	cv::imshow("hough-line-detection", dst);
	cv::waitKey(0);

	cv::destroyAllWindows();
    return 0;
}