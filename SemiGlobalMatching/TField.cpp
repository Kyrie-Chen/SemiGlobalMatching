#include "stdafx.h"
#include "TField.h"
#include <string>


TField::TField(std::string filename)
{

	//ÿ���Ƕ���Ϊһ��Ԫ��
	for (int angle = 0; angle < 360; angle += 5) {
		std::unordered_map<float, float> T_rule;	//{�����룺��Ӧ�¶�ֵ}

		//��ȡ�ļ�����������¶ȶ�Ӧֵ����T_rule
		

		plane_filed.insert({ angle, T_rule });
	}
}
