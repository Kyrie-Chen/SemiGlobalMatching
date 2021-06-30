#pragma once
#include "sgm_types.h"
#include <opencv2/opencv.hpp>


namespace new_util
{
	/*������ά����*/
	void saveXYZ(const char* filename, const float32* disp, int height, int width);

	/*�Ӳ�ͼת���ͼ*/
	void disp2Depth(const float32* disp, float32* depth, int height, int width, cv::Mat K, float baseline);

	/*float����ת��ΪMat��������ʾ����һ����*/
	void float2MatForDisplay(const float32* disparity, cv::Mat& disp_mat, int height, int width);
	
	/*float����ת��ΪMat*/
	void float2Mat(const float32* disp, cv::Mat& disp_mat, int height, int width);
	
	/*RGBͼ+���ͼת��Ϊ����ͼ*/
	void ConvertToPointCloud(const cv::Mat& rgb, const cv::Mat& depth, 
		PointCloud_color::Ptr cloud, const cv::Mat& cam_in, float cam_factor);
};

