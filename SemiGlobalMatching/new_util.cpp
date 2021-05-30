#include "stdafx.h"
#include "new_util.h"


/*������ά����*/
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
	�������ã��Ӳ�ͼת���ͼ
	���룺����
		disp ----�Ӳ�ͼ���ɵ����顡��
		K    ----�ڲξ���float����
		baseline ----���߾���
	���������
		depth ----���ͼ����

	���㹫ʽ��
		depth = (f * baseline) / disp
		���У�f��ʾ��һ���Ľ��࣬Ҳ�����ڲ��е�fx�� baseline�������������֮��ľ��룬�������߾���
*/
void new_util::disp2Depth(const float32* disp, float32* depth, int height, int width, cv::Mat cam_in, float baseline)
{	
	//float baseline = 178.089;	//���߾���65mm
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
				continue;  //��ֹ0��
			}
			depth[id] = (float)fx * baseline / disp[id];
		}
	}

}


/*float����ת��ΪMat��������ʾ*/
void new_util::float2Mat(const float32* disparity, cv::Mat& disp_mat, int height, int width)
{
	//�ҵ����������Ӳ�ֵ����С�Ӳ�ֵ
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

	//���Ӳ�ֵ����Mat�����һ����0-255
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
	�������ã�RGBͼ+���ͼת��Ϊ����ͼ
	���룺
		rgb ---- һ��Ϊ��������rgbͼ��Mat���ͣ�CV_8UC3
		depth ---- ���ͼ��Mat����
		cam_in ---- ����ڲξ���
	�����
		cloud ---- ����ͼ��PointCloud<PointXYZRGBA>��ָ������
*/
void new_util::ConvertToPointCloud(const cv::Mat& rgb, const cv::Mat& depth, 
	PointCloud_color::Ptr cloud, const cv::Mat& cam_in, float cam_factor)
{
	float fx = cam_in.at<float>(0, 0);		//����
	float fy = cam_in.at<float>(1, 1);
	float cx = cam_in.at<float>(0, 2);		//ͼ�����������������ͼ��ԭ������������
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

			// get point x y z ���� ͼ������ת��Ϊ��������ϵ������������ p.x / p.z = n / fx����CSDN��
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