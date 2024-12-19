#ifndef  _FACE_
#define  _FACE_

/*        Face ��

���ඨ���˸��༸�������е��浥Ԫ  

�����������˽ṹ             

*/


#include "GeometryData.h"
#include "Vertex.h"
#include "Edge.h"

namespace sn3DCore
{
	class FaceTopology
	{
	public:

		int m_ffp[3]; // ���ڸ���������߹��������е���һ����   
		int m_ffi[3];    // ������һ���иñ��ڸ����е�����
		int m_vfp[3]; // ���ڸ����������Ĺ��������е���һ����
		int m_vfi[3];    // ������һ�����иõ��ڸ����е����� 

		FaceTopology() {m_vfp[0]=m_vfp[1]=m_vfp[2]=-1;
		m_ffp[0]=m_ffp[1]=m_ffp[2]=-1;}
	};

	class Face
	{

	public:	

		typedef float ScalarType;
		//////////////////////////////  �����Է��� ///////////////////////////////////

		Vertex *  V(int j)              //   ��ȡ��j�����㶥��  
		{return &m_data->m_vertices[m_v[j]];}

		const Vertex* cV(int j)
		{return &m_data->m_vertices[m_v[j]];}

		Vertex& refV(int j)
		{return m_data->m_vertices[m_v[j]];}

		Vertex *  V1( int j )            //   ��ȡ��j������ĺ�һ������
		{ return &m_data->m_vertices[m_v[(j+1)%3]];}

		Vertex *  V2( int j )            //   ��ȡ��j������ĺ���������
		{ return &m_data->m_vertices[m_v[(j+2)%3]];}

		void SetV(int j,int index)        //   ���ö�������
		{m_v[j] = index;}

		int  E(int j){return m_data->m_pEdges[m_index*3+j];}

		int & Flag() {return m_flag;}     //  ��ȡ��־

		int & Index(){return m_index;}    //  ���������

		Point3f & P(int j )               //   ��õ�j������ļ�������
		{return m_data->m_vertices[m_v[j]].m_coord;}

		Point3f & N()                    //   ��ȡ�淨��
		{return m_data->m_faceNormal[m_index];}

		Point3f Color()                  //   ��ȡ����ɫ
		{return Point3f(m_data->m_faceColor[m_index].X(),m_data->m_faceColor[m_index].Y(),m_data->m_faceColor[m_index].Z());}

		void SetColor(Point3f _color)    //   ��������ɫ
		{m_data->m_faceColor[m_index].X() = _color.X();m_data->m_faceColor[m_index].Y() = _color.Y();
		m_data->m_faceColor[m_index].Z() = _color.Z();}

		Point4f ColorA()                 //   ��ȡ��alphaֵ������ɫ
		{return Point4f(m_data->m_faceColor[m_index].X(), m_data->m_faceColor[m_index].Y(), m_data->m_faceColor[m_index].Z(), m_data->m_faceColor[m_index].W());}

		void SetColor(Point4f _color)    //   ���ô�alphaֵ������ɫ
		{m_data->m_faceColor[m_index] = Color4f(_color.X(), _color.Y(), _color.Z(), _color.W());}

		Point2f  TexCoord(int j)                      //  ��ȡ��������
		{return m_data->m_faceTexCoord[m_index*3+j];}

		void     SetTexCoord(Point2f _tex,int j)      //  ������������
		{m_data->m_faceTexCoord[m_index*3+j] = _tex;}

		Face* FFp(int j)                //   ��ȡ��������ָ��
		{
		  if(m_data->m_faceTopology[m_index].m_ffp[j]==-1)
				return 0;
		  else
			return &(m_data->m_faces[m_data->m_faceTopology[m_index].m_ffp[j]]);
		}

		void  SetFFp(int j,Face * fi)
		{
		  if(fi==0)m_data->m_faceTopology[m_index].m_vfp[j]=-1;
			else
			m_data->m_faceTopology[m_index].m_ffp[j] = fi->Index();
	     }

		int &FFi(int j)                  //   ��ȡ������������
		{return m_data->m_faceTopology[m_index].m_ffi[j];}

		Face* VFp(int j)                //   ��ȡ��������ָ��
		{
			if(m_data->m_faceTopology[m_index].m_vfp[j]==-1)
				return 0;
			else
			return &(m_data->m_faces[m_data->m_faceTopology[m_index].m_vfp[j]]);}

		void  SetVFp(int j,Face * fi)
		{
			if(fi==0)m_data->m_faceTopology[m_index].m_vfp[j]=-1;
			else
			m_data->m_faceTopology[m_index].m_vfp[j] = fi->Index();
		}

		int &VFi(int j)                  //   ��ȡ������������
		{return m_data->m_faceTopology[m_index].m_vfi[j];}	

		int & Identity(){return m_data->m_FaceIdentity[m_index];}

	    int & Identity1(){return m_data->m_FaceIdentity1[m_index];}

	    int & Identity2(){return m_data->m_FaceIdentity2[m_index];}

		void InitIMark()                  //  ��ʼ��������־
		{m_data->m_faceMark[m_index] = 0;}

		int &IMark()                      //  ��ȡ������־
		{return m_data->m_faceMark[m_index];}

		void SetGeometry(sn3DGeometryData * data){m_data =  data;}  //  ���ü������ݶ���	

		////////////////////////////    ��־λ   /////////////////////////////////

		bool  IsD()  {return (this->Flag() & DELETED) != 0;}	// �Ƿ�ɾ��
		void  SetD() {this->Flag() |= DELETED;}						
		void  ClearD() {this->Flag() &=(~DELETED);}				

		bool  IsV() {return (this->Flag() & VISITED) != 0;}  	// ����λ
		void  SetV(){this->Flag() |= VISITED;}						
		void  ClearV()	{this->Flag() &= ~VISITED;}				

		bool  IsM() {return (this->Flag() & MODIFIED) != 0;}	// �޸�λ
		void  SetM(){this->Flag() |= MODIFIED;}						
		void  ClearM(){this->Flag() &= ~MODIFIED;}					

		bool  IsM1() {return (this->Flag() & MODIFIED1) != 0;}	// �޸�λ
		void  SetM1(){this->Flag() |= MODIFIED1;}						
		void  ClearM1(){this->Flag() &= ~MODIFIED1;}				

		bool  IsM2() {return (this->Flag() & MODIFIED2) != 0;}	// �޸�λ
		void  SetM2(){this->Flag() |= MODIFIED2;}					
		void  ClearM2(){this->Flag() &= ~MODIFIED2;}			

		bool  IsS() {return (this->Flag() & SELECTED) != 0;}	//ѡ��λ
		void  SetS() {this->Flag() |= SELECTED;}						
		void  ClearS(){this->Flag() &=(~SELECTED);}					

		bool IsB(int i)  {return (this->Flag() & (BORDER0<<i)) != 0;} // �߽�λ
		void SetB(int i)  {this->Flag() |=(BORDER0<<i);}		
		void ClearB(int i)	{this->Flag() &= (~(BORDER0<<i));}	

		bool IsR()  {return (this->Flag() & NOTREAD) == 0;}	//��ȡλ
		void SetR() {this->Flag() &=(~NOTREAD);}					
		void ClearR() {this->Flag() |=NOTREAD;}						

		bool IsW()  {return (this->Flag() & NOTWRITE)== 0;}	//д��λ
		void SetW() {this->Flag() &=(~NOTWRITE);}				
		void ClearW() {this->Flag() |=NOTWRITE;}			

		bool  IsMultPolygon() {return (this->Flag() & MULTPOLYGON) != 0;}	// �޸�λ
		void  SetMultPolygon(){this->Flag() |= MULTPOLYGON;}						
		void  ClearMultPolygon(){this->Flag() &= ~MULTPOLYGON;}		

		bool  IsFeature() { return (this->Flag() & FEATURE) != 0; }	// �޸�λ
		void  SetFeature() { this->Flag() |= FEATURE; }
		void  ClearFeature() { this->Flag() &= ~FEATURE; }

		void  SetFlag(int flag) {this->Flag()=flag;}				// ���ñ�־	
		void  ClearFlag() {this->Flag()=0;}							// �����־


		enum { 		
			DELETED     = 0x01,		   // ɾ��λ
			VISITED     = 0x02,        // ����λ
			SELECTED    = 0x04,        // ѡ��λ		
			MODIFIED    = 0x08,        // �޸�λ  	
			MODIFIED1   = 0x10,        // �޸�λ
			MODIFIED2   = 0x40,        // �޸�־
			BORDER0     = 0x80,        // �߽�λ
			NOTWRITE    = 0x1000,	   // д��λ
			NOTREAD     = 0x2000,	   // ��ȡλ
			MULTPOLYGON = 0x4000,       // �����
			FEATURE = 0x8000       // ����
		};

	public:

		Face(){m_data = 0;m_flag=0;m_index=0;m_v[0]=0;m_v[1]=0;m_v[2]=0;}

		Face(sn3DGeometryData * data,int index){m_data=data;m_flag=0;m_index=index;m_v[0]=0;m_v[1]=0;m_v[2]=0;}

		Face(int p1,int p2, int p3,sn3DGeometryData * data,int index)
		{m_v[0]=p1;m_v[1]=p2;m_v[2]=p3;m_flag=0;m_index=index;m_data = data;}

		Face  operator = (Face face)
		{
			m_flag = face.m_flag;
			m_data = face.m_data;
			m_v[0] = face.m_v[0];
			m_v[1] = face.m_v[1];
			m_v[2] = face.m_v[2];
			return face;
		}
	public:

		sn3DGeometryData * m_data; // �������ݶ���

		int m_index;				// ���������

		int  m_v[3];               // ���㼯

		int  m_flag;               // ���־λ
	};
}
#endif