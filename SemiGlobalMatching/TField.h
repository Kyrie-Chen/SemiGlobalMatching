#pragma once
#include <unordered_map>



class TField
{
public:
	TField(std::string filename);		//�����¶�ͼ�ļ�����õ���ɢ�Ķ�ά�¶�ͼ


	//{�Ƕ�ֵ��{�����룺��Ӧ�¶�ֵ}}
	std::unordered_map<int, std::unordered_map<float, float>> plane_filed;		//��ɢ�Ķ�ά�¶�ͼ

private:


};

