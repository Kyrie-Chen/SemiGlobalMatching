/* -*-c++-*- SemiGlobalMatching - Copyright (C) 2020.
* Author	: Yingsong Li(Ethan Li) <ethan.li.whu@gmail.com>
* https://github.com/ethan-li-coding/SemiGlobalMatching
* Describe	: header of sgm_types
*/

#pragma once

#include <cstdint>
#include <limits>
#include <pcl/visualization/pcl_visualizer.h>
#include <unordered_map>

/** \brief float��Чֵ */
constexpr auto Invalid_Float = std::numeric_limits<float>::infinity();

/** \brief �������ͱ��� */
typedef int8_t			sint8;		// �з���8λ����
typedef uint8_t			uint8;		// �޷���8λ����
typedef int16_t			sint16;		// �з���16λ����
typedef uint16_t		uint16;		// �޷���16λ����
typedef int32_t			sint32;		// �з���32λ����
typedef uint32_t		uint32;		// �޷���32λ����
typedef int64_t			sint64;		// �з���64λ����
typedef uint64_t		uint64;		// �޷���64λ����
typedef float			float32;	// �����ȸ���
typedef double			float64;	// ˫���ȸ���

typedef pcl::PointXYZRGBA Point_color;		
typedef pcl::PointCloud<Point_color> PointCloud_color;

struct 2DTemp {
	
	float temperature;
};

