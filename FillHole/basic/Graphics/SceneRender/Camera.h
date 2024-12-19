#ifndef _CAMERA_
#define _CAMERA_

/*************** sn3DCore::Camera ******************
*--------------------------------------------------*
* ����ʵ���˵�ǰ��������ʾ��Χ���ƹ���,��Ҫ����:   *
*  1. �Ӿ��� ͶӰ��������,֧��͸�ӡ��������ַ�ʽ   *
*  2. ��ת��ƽ�ơ����ţ�ģ���ӵ��������           *
*  3. ��������任                                 *
*  4. �����任��ʼ��                               *
***************************************************/


#include "Common/point3.h"
#include "Common/plane3.h"
#include "Common/line3.h"
#include "Common/matrix44.h"
#include "Common/matrix33.h"

namespace sn3DCore
{
	class Camera
	{
	public:

		void GetView();
		void SetView();


		//======��������任
		Point3f Project(const Point3f &p) const;
		Point3f UnProject(const Point3f &p) const;
		Point3f ViewPoint() const;


		Plane3f ViewPlaneFromModel(const Point3f &p); //����ͨ����P�д�ֱ�������ƽ��
		Line3f ViewLineFromModel(const Point3f &p);   //����ͨ����P������
		Line3f ViewLineFromWindow(const Point3f &p);  //����ͨ����P�͹۲��ߵ���
		Point3f NormDevCoordToWindowCoord(const Point3f &p) const;//����һ���豸����ת��Ϊ��������
		Point3f WindowCoordToNormDevCoord(const Point3f &p) const;//����������ת��Ϊ��һ���豸����

	public:

		int viewport[4];		//���½ǣ�����
		float ratio;			//�Ӿ���߶�
		float fov;				//�ӽ�

		float clipRatioFar;		//Զ���ü�����
		float clipRatioNear;	//���ü�����
		float nearPlane;		//���ü�ƽ��
		float farPlane;			//Զ�ü�ƽ��

		//======Camera����
		Point3f m_Eye;
		Point3f m_Up;
		Point3f m_Target;

		Matrix44f proj;//ͶӰ����
		Matrix44f model;//ģ����ͼ����
		Matrix44f matrix;//�任����
		Matrix44f inverse;//�任�������  

	};
}

#endif
