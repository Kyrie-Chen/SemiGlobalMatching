#include "stdafx.h"
#include "TField.h"
#include <string>


TField::TField(std::string filename)
{

	//每个角度作为一个元素
	for (int angle = 0; angle < 360; angle += 5) {
		std::unordered_map<float, float> T_rule;	//{起点距离：对应温度值}

		//读取文件，将距离和温度对应值放入T_rule
		

		plane_filed.insert({ angle, T_rule });
	}
}
