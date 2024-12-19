#ifndef _SN3D_SCENEGL2D_
#define _SN3D_SCENEGL2D_

#include "SceneGL.h" 
#include "SceneInteract/Navigation/Trackball.h" 

namespace sn3DCore
{
	class sn3DSceneGL2D : public SceneGL
	{
	public:

		/////////////////  ���Ʒ���   ///////////////

		virtual void PaintStatic();

		virtual void PaintScene();

		virtual void PaintMotion();

		void Initialize(float width, float height, HWND hWnd);

		void Destroy() {};

		virtual void Resize(int x, int y, int w, int h);
		virtual void UpdateSceneBox() {}
		virtual int  GetCount() { return 0; };

		virtual void MouseDown(int x, int y, int button); //����������
		virtual void MouseMove(int x, int y);
		virtual void MouseUp(int x, int y, int button);
		virtual void MouseWheel(float notch);

	public:
		void InitSceneParameter();// ��ʼ��������������
		void InitCamera(int width, int height); //  ��ʼ�������
		void InitSceneCamera();    //���ó��������
		void InitViewPoint();      //�����ӵ�λ��
		void ScaleSceneModel();    // ���ų����е�ģ������׼��Χ(CADģʽ)
	public:

		sn3DSceneGL2D();
		virtual ~sn3DSceneGL2D();

		Box3f			m_sceneBox;		   // �����Զ���ģ�Ͱ�Χ��
		Trackball		m_trackball;	   // ARCBALL
	};
}

#endif