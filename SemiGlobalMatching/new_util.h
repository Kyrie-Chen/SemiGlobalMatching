#pragma once
#include "sgm_types.h"
#include <opencv2/opencv.hpp>


namespace new_util
{
	/*保存三维坐标*/
	void saveXYZ(const char* filename, const float32* disp, int height, int width);

	/*视差图转深度图*/
	void disp2Depth(const float32* disp, float32* depth, int height, int width, cv::Mat K, float baseline);

	/*float数组转换为Mat并用于显示（归一化）*/
	void float2MatForDisplay(const float32* disparity, cv::Mat& disp_mat, int height, int width);
	
	/*float数组转换为Mat*/
	void float2Mat(const float32* disp, cv::Mat& disp_mat, int height, int width);
	
	/*RGB图+深度图转换为点云图*/
	void ConvertToPointCloud(const cv::Mat& rgb, const cv::Mat& depth, 
		PointCloud_color::Ptr cloud, const cv::Mat& cam_in, float cam_factor);
};

