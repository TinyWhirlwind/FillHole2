#ifndef _SN3DPICK_
#define _SN3DPICK_

#include <vector>

#include"../../SceneGraph/sn3DObject.h"
#include"../../SceneRender/SceneData.h"
#include"../../../Common/Point3.h"
#include"../../../Common/Quaternion.h"

/*       sn3DPick��

ʵ���˶�ά��Ļ����������ά������ѡ�Ľ����㷨*/

namespace sn3DCore
{

class sn3DPick
{
public:

	virtual void PaintVertex(sn3DObject * obj,Quaternionf rot,int flag);  //��ˢѡ��
	
	virtual void RectVertex(sn3DObject * obj,Quaternionf rot,int flag);   // ����ѡ��
	
	virtual void SelectPolygonVertex(sn3DObject * obj,Quaternionf rot,int flag); // �����ѡ��

	virtual	void SelectRefPolygonVertex(sn3DObject * m,Quaternionf rot,int flag);//��־��ѡ��

	virtual void UpdateMatrix();        //  ���³���������Ϣ,����Ҫ������Ϣ֮ǰ���ã���ʱ������µĳ����任����

	virtual void DrawXORPaintCircle();  //  ��ˢѡ��ԲȦ����
	
	virtual void DrawPolygon();         //  �����ѡ�����λ���
	
	virtual void DrawRect();            //  ����ѡ����λ���

	virtual void Clear();

public:
	
	sn3DPick();
	virtual ~sn3DPick();

public:

	std::vector<Point2i> m_Polygon; 
	
	std::vector<Point2i> m_Rect;

	bool m_IsDragging;		//�Ƿ������϶���
	
	bool m_IsSelected;		//�Ƿ�ʰȡ

	Point2f m_CurPoint;	//�������ǰ��
	
	Point2f m_PrevPoint;	//�����ǰһ��

	double	m_MvMatrix[16]; //ģ���ӵ����

	double	m_ProjMatrix[16]; //ͶӰ����
	
	int	m_Viewport[4];	//��ǰ�ӿ�
	
	float * m_Pixels;		// ��ǰz-buffer
	
    unsigned int*	m_SelectBuf;  // ѡ�񻺴�
	
	double  m_PaintRadius;  // ƽ�滭ˢ�뾶
	
	double  m_PaintRadius2; // ƽ�滭ˢ�뾶��ƽ��

	int     m_SelectRadius;   // ����ʹ�õİ뾶ֵ    

	std::vector<Point3f> points; //������

};
}
#endif
