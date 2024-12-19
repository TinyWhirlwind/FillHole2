#ifndef _SN3DSELECT_
#define _SN3DSELECT_



/*   sn3DSelect ��

   ��ά����ѡ����ʵ����

*/

#include "sn3DMeshPick.h"
#include "sn3DRangePick.h"
#include "../../SceneRender/sn3DSceneGL3D.h"
#include "../../SceneGraph/sn3DObject.h"
namespace sn3DCore
{

// ѡ��ģʽ��
#define PAINT_SELECT  1
#define POLYGON_SELECT 2
#define RECT_SELECT 3
#define REFPOINT_SELECT 4

class sn3DSelect 
{
public:

	void   Draw(sn3DSceneGL3D *,int feature);               //  �����ӿ�,���������Լ�����ʱʹ��  
	
	bool   MouseMove(sn3DSceneGL3D *,unsigned int,int,int); //  ����ƶ���Ϣ,ѡ���϶�
	
	bool   LeftClick(sn3DSceneGL3D*,unsigned int,int,int);  //  ��������Ϣ,����ѡ���϶�״̬ 
	
	bool   LeftUp(sn3DSceneGL3D *,int,int);                 //  ������������Ϣ��ѡ���϶�����

	void   SetMode(int mode);                      //   ����ѡ��ģʽ

	void   SetMesh(sn3DObject * mesh);              //   ���ô�ѡ�����

	void   SetScene(sn3DSceneGL3D * scene);          //   ���ô�ѡ�񳡾�

	void   SetPick(sn3DPick * pick);                //   ����ѡ���ܶ���

	bool   IsDragging();                           // �Ƿ����϶�

	void   Store(){m_LastState = m_SelectMode;}

	void   Restore(){if(m_LastState!=0)m_SelectMode = m_LastState;}

	void   SetSelectRadius(int selectRadius);

	void   Clear();

public:

	sn3DSelect();
	
	~sn3DSelect();

	int m_feature;

	int	m_State;

	int	m_SelectMode;

	bool isPicking;

	sn3DObject * m_object;

	sn3DPick *		m_pick;

protected:	

	int	m_LastState;
	
	

	
};

}
#endif