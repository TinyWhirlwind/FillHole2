#ifndef FILLHOLE_H
#define FILLHOLE_H

#include <iostream>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;
class FillHole
{
public:
	FillHole();
	FillHole(MyMesh mesh);
	~FillHole();

public:
	void SetMesh(MyMesh mesh);
	MyMesh GetMesh();
	std::vector<std::vector<MyMesh::VertexHandle>> CheckBoundarys();
	int GetHoleNumber();
	void ApplyFillHole();
	void ApplyFillHole(int index);
	void MeshRefinement();
private:
	double MWT(std::vector<MyMesh::VertexHandle> boundary_vertex, int i, int j);
	double CalcAreaWeight(MyMesh::VertexHandle v0, MyMesh::VertexHandle v1, MyMesh::VertexHandle v2);
	void RecoverTriangles(std::vector<MyMesh::VertexHandle> boundary_vertex, int i, int j);
	double CalcDihedralAngleWeight(MyMesh::VertexHandle v0, MyMesh::VertexHandle v1, MyMesh::VertexHandle v2);
	MyMesh::HalfedgeHandle GetHalfEdge(MyMesh::VertexHandle from_v, MyMesh::VertexHandle to_v);
	OpenMesh::Vec3f getFaceNormal(MyMesh::FaceHandle face);
	bool RelaxEdge();
	bool ReplaceTriangle();
	void SmoothMesh();
	double GetInteriorEdgeAverageLength();
	void outputPatchMesh();
private:
	int origin_faces_num = 0;
	int origin_vertices_num = 0;
	int boundary_edge_num = 0;
	double average_edge_length = 0.0;
	MyMesh mesh_;
	MyMesh patch_mesh_;
	std::vector<std::vector<MyMesh::VertexHandle>> boundarys_;
	//std::vector<std::vector<LAngle>> min_weight_;
	std::vector<std::vector<double>> min_weight_;
	std::vector<MyMesh::FaceHandle> add_faces_;
	std::vector<int> add_vertices_;
	std::vector<std::vector<int>> best_k;
	std::vector<std::tuple<int, int, int>> triangles_;
};
#endif