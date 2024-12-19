#ifndef _CURVATURE_
#define _CURVATURE_

#include <math.h>
#include <vector>
#include "Common/point3.h"
#include "GeoData/MeshData.h"

//������������ĵ��������������

class LocalFrame//�ֲ����
{
public:
	LocalFrame()
	{
		k1 = k2 = k12 = 0;
		uDir.X() = uDir.Y() = uDir.Z() = 0;
		vDir.X() = vDir.Y() = vDir.Z() = 0;
		normal.X() = normal.Y() = normal.Z() = 0;
	}
	Point3f uDir;//�ֲ����
	Point3f vDir;
	Point3f normal;
	float k1;  //��������
	float k2;
	float k12;
};

class Curvature
{
public:
	Curvature();
	~Curvature();

	void ComputerCurvatrue(std::vector<Face *> & faces, std::vector<Vertex *> & vertices);
	void VertexCurvature(std::vector<Face *> & faces, std::vector<Vertex *> & vertices);
	void ComputeVertexArea(std::vector<Face *> & faces, std::vector<Vertex *> & vertices);
	void ProjectCurvature(LocalFrame & faceFrame, LocalFrame & vertexFrame);
	void DiagonalizeCurvature(LocalFrame & frame);

	void  choldc(double a[][3], Point3f & p);
	void  cholsl(double a[][3], Point3f & p, Point3f & b, Point3f & x);

public:
	LocalFrame * m_Frame;//������Ϣ

	Point3f * m_FaceCornerArea;
	float *  VertexArea;
};
#endif