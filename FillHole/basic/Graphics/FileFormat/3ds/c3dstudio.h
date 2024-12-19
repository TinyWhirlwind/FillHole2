#ifndef _C3DSTUDIO_
#define _C3DSTUDIO_

#include<list>
#include <stdio.h>
#include<string.h>
#include"3dsfile.h"
#include"../BaseReader.h"
#include"../../../Common/Point3.h"

using namespace std;

#define	POINT_LIGHT			1
#define SPOT_LIGHT			2
#define	DIRECTIONAL_LIGHT	3


class C3DStudio 
{
public:

  class Light 
 {
  public:
     	
    string	 m_Name;//�ƹ�����
	unsigned int	 m_LightGL; //OpenGL ��Դ���
	int		 m_Type;  //��Դ����
	bool	 m_Enable;	
	Point3f  m_Color;	//��Դ��ɫ
	float    m_C1, m_C2, m_C3;	//˥��ϵ��
	Point3f  m_Location;	//��Դλ��	
    float	 m_NearRange;/* ����׷��֧�ֵ�˥������ */	
	float    m_FarRange;
	bool	 m_AttnOn;
	Point3f  m_Target;//�۹��Ŀ��λ��
	float	 m_Falloff; //GL_SPOT_CUTOFF
	float    m_Hotspot; //GL_SPOT_EXPONENT 
  };

 class Camera 
 {
  public:
	string   m_Name;	
	Point3f m_Eye;//���λ��
	Point3f m_Target;//���Ŀ��
	Point3f m_Up;//������
    float m_ScreenWidth;//������(���ڻ��ƴ��ڿ�ȣ����������ͶӰƽ���ȿɼ���)
	float m_ScreenHeight;//����߶�(���ڻ��ƴ��ڸ߶�)
	float m_Focal_length; //����?��̫���������ĸ���
	float m_Fov;	//�Ӿ�������(�Ƕȷǻ���)
	float m_RangeNear; //�Ӿ�����ü�ƽ�棬ʵ�ʶ�ȡ����3ds�ļ��Ĵ�������Ӱ��ֵ!!
	float m_RangeFar; //�Ӿ���Զ�ü�ƽ��
	float m_Roll;	//3ds��һ���Ƕȣ��ݲ���
  };

  class Surface
  {
   public:
    
	Surface(){m_Ambient = m_Specular = m_Reflected = m_Transmitted = m_Diffuse = Point3f(0.0,0.0,0.0);m_Index=1.0;}  
	string	  m_Name;			//��������
	string    m_BumpName;    //��͹�����ļ���
	string    m_TextureName; //�����ļ���
    float	  m_Index;		//���������
	Point3f	  m_Ambient;		
	Point3f	  m_Diffuse;		 
	Point3f	  m_Specular;		
	float	  m_Shininess;		// specularity factor
	float	  m_Shin_strength;   // the presence of the specular color in the hilight <not variable>
	Point3f	  m_Transmitted;	//͸����ɫ
	Point3f	  m_Reflected;	//������ɫ		
	short int m_PolygonMode;    
	short int m_ShadeModel;       
	bool	  m_TwoSided;
   };

  struct chunk3ds 
  {
   unsigned short int  tag;		
   unsigned long int   len;        
  };

	bool   Read3DSFile(std::string & file_name,BaseImporter * _bi);

	bool   Is3DSFile();

	FILE * fp;
    string 			m_Filename;
	float			m_Scale;
	unsigned long	m_Version;
	Point3f	backcolor;
	Point3f	ambient;

public:

	C3DStudio();

	~C3DStudio();

   std::list<Light *>      m_lightlist;
   std::list<Camera *>     m_cameralist;
   std::list<Surface *>    m_surfacelist;

private:

   BaseImporter * m_importer;
   Light   * newLight;
   Camera  * newCamera;
   Surface * newSurface;

   bool ReadMain(long Start, long Size);
   bool ReadEdit(long Start, long Size);
   bool ReadMaterial(long Start, long Size);
   bool ReadObject(long Start, long Size);
   bool ReadTriMesh(long Start, long Size);
   bool ReadLight( long Start, long Size);
   bool ReadCamera( long Start, long Size);
   bool ReadVertexArray();
   bool ReadFaceArray();
   bool ReadTexArray();
   bool ReadMeshMaterial();
   bool ReadSpotLight(long Start, long Size);
   bool ReadTextureMap(long Start, long Size,int type);
   bool ReadColor(Point3f & color);
   bool ReadPercentage(float &value);
   char * ReadAndAllocString3ds(unsigned int len);
		
};

#endif