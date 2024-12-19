#ifndef _MOTIONWIDGET_
#define _MOTIONWIDGET_

#include "../../../Common/point3.h"
//#include "../../SceneRender/SceneData.h"
#include "../../Entity/SceneObject.h"
#include "../../../Common/Quaternion.h"
#include "../../SceneModel/sn3DObject.h"

namespace sn3DCore
{
class MotionWidget
{
public:

   void  SetData(sn3DObject * obj);
   void  Transform();
   void  Initialize(Point3f mainAxis,Point3f secoAxis,Point3f thirAxis,Point3f center); //��ʼ���ֲ�����ϵ
   void  HitTest(); //���Կؼ��Ƿ񱻼���
   
   void  Motion(Point3f & mainAxis,Point3f & secoAxis,Point3f & thirAxis,Point3f & center);//�����˶�����
   void  Move(float distance ,int axis);//ƽ�ƺ���
   void  Rotate(float degree,int axis);//��ת����
   
   void  Multiply(float & newS,float & newQX,float & newQY,float & newQZ,float  S,float  QX, 
	             float  QY , float  QZ,float  s, float  qx, float qy , float  qz);//��Ԫ���˷�
   
   float UnProjectDistance(Point3f dir); //���û�����Ļ�ϵ�������ת��Ϊ�����е�λ��
   
   void  DrawWidget();//���ƾֲ�����ϵ�ؼ�
   void  DrawPoint();//���Ʋ��Ե�
   void  DrawCircle ();//����Բ��
   void  DrawSegment();//�����߶�

public:

      MotionWidget();
	  ~MotionWidget();
   
public:

   sn3DObject * m_object;

   //�ֲ�����ϵ����
   float m_Mat1[9];//�����������������ת����
   float m_Mat2[9];
   float m_Mat3[9];
   
   float m_Size; //�ֲ��˶�ϵ��ܴ�С

   Quaternion<float> m_Quaternion;//�˶���Ԫ��
   
   Point3f m_Center;    //�˶�������
   Point3f m_OriginMainAxis;   //��ʼ���᷽�� 
   Point3f m_OriginSecoAxis;   //��ʼ���᷽�� 
   Point3f m_OriginThirAxis;   //��ʼ�����᷽�� 

   Point3f m_CurMainAxis;   //��ǰ���᷽�� 
   Point3f m_CurSecoAxis;   //��ǰ���᷽�� 
   Point3f m_CurThirAxis;   //��ǰ�����᷽�� 

   float m_S,m_QX,m_QY,m_QZ; //��ǰ��ת��Ԫ��,�����ԭ�᷽��

   float m_Matrix[16];//����ת����
   
   Point3f m_RotatePoint; //��ת������

   bool m_IsDragging;
   bool m_IsActive;
   bool m_Active[6];
   
   SceneObject m_SceneObject; //��������
   float m_Degree1,m_Degree2,m_Degree3;
   Point3f m_Axis1,m_Axis2,m_Axis3;
   
  //OpenGL ����֧��
   Point3f m_CurPoint; //�������ǰ��
   Point3f m_PrevPoint;//�����ǰһ��
   double m_MvMatrix[16]; //ģ���ӵ����
   double m_ProjMatrix[16]; //ͶӰ����
   int  m_Viewport[4]; //��ǰ�ӿ�
   unsigned int * m_SelectBuf;//select-buffer
   void UpdateMatrix();

};
}
#endif