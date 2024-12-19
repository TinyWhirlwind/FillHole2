#ifndef _SN3D_SCENE_MANAGER_
#define _SN3D_SCENE_MANAGER_

#include"../../sn3DInterface/Isn3DScene.h"

/* ���������� */
namespace sn3DCore
{

	class sn3DSceneManager : public Isn3DScene
	{
	public:

		bool ShareDisplay(sn3DSceneGL * scene1,sn3DSceneGL * scene2); // ������ʾ�б�

		void AddScene(sn3DSceneGL * scene);                           // ���볡��

		sn3DSceneGL * GetScene(int id);                               // ��ȡ�����ӿ�     

		void HideScene(int id);                                       // �رճ�����ʾ

		void ShowScene(int id);                                       // ��ʾ����

		void SetDrawMode(int id,DrawGL::DrawMode dm);

		void CheckShowBox(int id);

		void CheckShowModelBox(int id);

		//void SetColorMode(int id,int flag);


	public:

		std::vector<sn3DSceneGL *> m_scenes;                         // ���г���

	};
}


#endif