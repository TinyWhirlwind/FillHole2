#ifndef _OPENGL_DEVICE_
#define _OPENGL_DEVICE_

#include "Common/Point3.h"
#include <windows.h> 

/*  OpenGL �豸��

����ʵ����OpenGL������ص���Ϣ������
*/

namespace sn3DCore
{
	class OpenGLDevice
	{
	public:

		////////////////////////////   �豸����   /////////////////////////////////

		bool InitializeGL(float width, float height, HDC hDC);  //  ��ʼ���豸

		void DestroyGL();                                     //  �����豸

		BOOL SetWindowPixelFormat(HDC hDC);  //  �豸���ظ�ʽ                   

		BOOL CreateViewGLContext(HDC hDC);   //  ����OpenGL�豸


		///////////////////////////    ���ݷ���   /////////////////////////////////

		HGLRC GetContext()                  //  ��ȡ���ƻ���
		{
			return m_hGLContext;
		}

		void GetMvMatrix(double* matrix);   //  ��ȡģ�ͱ任����

		void GetProjMatrix(double* matrix); //  ��ȡͶӰ�任����

		void GetViewport(int * viewport);   //  ��ȡ�ӿ���Ϣ

		float* GetDepthBuffer()             //  ��ȡ��Ȼ���
		{
			return (float*)m_depthBuffer;
		}

		unsigned int*  GetSelectBuffer()    //  ��ȡѡ�񻺴�
		{
			return (unsigned int*)m_selectBuf;
		}


		///////////////////////////////////////////////////////////////////////////////////

	public:
		OpenGLDevice();

		~OpenGLDevice();

	public:

		HWND	     m_hWnd;            //  ���ھ��

		HDC			 m_hDC;             //  ������Դ

		int			 m_iGLPixelIndex;   //  ���ظ�ʽ����

		HGLRC		 m_hGLContext;      //  �豸����

		float*		 m_depthBuffer;		//  ��Ȼ���

		unsigned int*m_selectBuf;		//  ѡ�񻺴�

		double		 m_mvMatrix[16];    //  ģ�ͱ任����

		double		 m_projMatrix[16];	//  ͶӰ�任����

		int			 m_viewport[4];     //  �ӿ�
	};
}
#endif