#pragma once
#include <unordered_map>



class TField
{
public:
	TField(std::string filename);		//根据温度图文件构造得到离散的二维温度图


	//{角度值：{起点距离：对应温度值}}
	std::unordered_map<int, std::unordered_map<float, float>> plane_filed;		//离散的二维温度图

private:


};

