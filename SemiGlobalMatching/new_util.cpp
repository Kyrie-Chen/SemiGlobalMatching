#include "stdafx.h"
#include "new_util.h"


/*保存三维坐标*/
void new_util::saveXYZ(const char* filename, const float32* disp, int height, int width)
{
	//const double max_z = 16.0e4;
	FILE* fp = fopen(filename, "w");
	//printf("%d %d \n", mat.rows, mat.cols);
	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int id = i * height + j;
			fprintf(fp, "%.3f,", disp[id]);

			//Vec3f point = mat.at<Vec3f>(y, x);
			//if (fabs(point[2] - max_z) < FLT_EPSILON || fabs(point[2]) > max_z) 
			//	continue;
			//fprintf(fp, "%f %f %f\n", point[0], point[1], point[2]);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}


/*
	函数作用：视差图转深度图
	输入：　　
		disp ----视差图构成的数组　　
		K    ----内参矩阵，float类型
		baseline ----基线距离
	输出：　　
		depth ----深度图数组

	计算公式：
		depth = (f * baseline) / disp
		其中，f表示归一化的焦距，也就是内参中的fx； baseline是两个相机光心之间的距离，称作基线距离
*/
void new_util::disp2Depth(const float32* disp, float32* depth, int height, int width, cv::Mat cam_in, float baseline)
{	
	//float baseline = 178.089;	//基线距离65mm
	//const float PI = 3.14159265358;
	float fx = cam_in.at<float>(0, 0);
	float fy = cam_in.at<float>(1, 1);
	float cx = cam_in.at<float>(0, 2);
	float cy = cam_in.at<float>(1, 2);

	for (int i = 0; i < height; i++) {
		for (int j = 0; j < width; j++) {
			int id = i * width + j;
			if (!disp[id]) {
				depth[i] = 0;
				continue;  //防止0除
			}
			depth[id] = (float)fx * baseline / disp[id];
		}
	}

}


/*float数组转换为Mat并用于显示*/
void new_util::float2Mat(const float32* disparity, cv::Mat& disp_mat, int height, int width)
{
	//找到数组的最大视差值和最小视差值
	float min_disp = width, max_disp = -width;
	for (sint32 i = 0; i < height; i++) {
		for (sint32 j = 0; j < width; j++) {
			const float32 disp = disparity[i * width + j];
			if (disp != Invalid_Float) {
				min_disp = std::min(min_disp, disp);
				max_disp = std::max(max_disp, disp);
			}
		}
	}

	//将视差值放入Mat，斌归一化至0-255
	for (sint32 i = 0; i < height; i++) {
		for (sint32 j = 0; j < width; j++) {
			const float32 disp = disparity[i * width + j];
			if (disp == Invalid_Float) {
				disp_mat.data[i * width + j] = 0;
			}
			else {
				disp_mat.data[i * width + j] = static_cast<uchar>((disp - min_disp) / (max_disp - min_disp) * 255);
			}
		}
	}
}


/*
	函数作用：RGB图+深度图转换为点云图
	输入：
		rgb ---- 一般为左侧相机的rgb图，Mat类型，CV_8UC3
		depth ---- 深度图，Mat类型
		cam_in ---- 相机内参矩阵
	输出：
		cloud ---- 点云图，PointCloud<PointXYZRGBA>的指针类型
*/
void new_util::ConvertToPointCloud(const cv::Mat& rgb, const cv::Mat& depth, 
	PointCloud_color::Ptr cloud, const cv::Mat& cam_in, float cam_factor)
{
	float fx = cam_in.at<float>(0, 0);		//焦距
	float fy = cam_in.at<float>(1, 1);
	float cx = cam_in.at<float>(0, 2);		//图像的中心像素坐标与图像原点像素坐标间距
	float cy = cam_in.at<float>(1, 2);

	// loop the mat
	for (int m = 0; m < depth.rows; m++) {
		for (int n = 0; n < depth.cols; n++) {
			// get depth value at (m, n)
			//std::uint16_t d = depth.ptr<std::uint16_t>(m)[n];
			uint16 d = depth.ptr<uint16>(m)[n];
			// when d is equal 0 or 4096 means no depth
			if (d == 0 || d == 4096)
				continue;

			Point_color p;

			// get point x y z —— 图像坐标转换为世界坐标系（相似三角形 p.x / p.z = n / fx，见CSDN）
			p.z = static_cast<float>(d) / cam_factor;
			p.x = (n - cx) * p.z / fx;
			p.y = (m - cy) * p.z / fy;

			// get colors
			p.b = rgb.ptr<uchar>(m)[n * 3];
			p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
			p.r = rgb.ptr<uchar>(m)[n * 3 + 2];

			// add point to cloud
			cloud->points.push_back(p);
		}
	}
}