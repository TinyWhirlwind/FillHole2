#include "FillHoleInterface.h"
#include "FillHole.h"
#include "GeoData/MeshData.h"
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>
#include <boost/smart_ptr.hpp>
typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;

FillHoleInterface::FillHoleInterface():_self(new FillHole)
{
}
FillHoleInterface::FillHoleInterface(boost::shared_ptr<sn3DCore::sn3DMeshData> mesh)
{
	MyMesh open_mesh;
	for (int i = 0; i < mesh->n_vertices(); i++)
	{
		Point3f p = mesh->V(i)->P();
		MyMesh::Point open_p(p.x, p.y, p.z);
		open_mesh.add_vertex(open_p);
	}
	for (int i = 0; i < mesh->n_faces(); i++)
	{
		int v0 = mesh->F(i)->m_v[0];
		MyMesh::VertexHandle v[3];
		for (int j = 0; j < 3; j++)
		{
			v[j] = open_mesh.vertex_handle(mesh->F(i)->m_v[j]);
		}
		open_mesh.add_face(v[0], v[1], v[2]);
	}
	open_mesh.update_normals();
	_self = (new FillHole(open_mesh));
}

FillHoleInterface::~FillHoleInterface()
{
	if (_self) delete _self;
}

void FillHoleInterface::SetMesh(boost::shared_ptr<sn3DCore::sn3DMeshData> mesh)
{
	MyMesh open_mesh;
	for (int i = 0; i < mesh->n_vertices(); i++)
	{
		Point3f p = mesh->V(i)->P();
		MyMesh::Point open_p(p.x, p.y, p.z);
		open_mesh.add_vertex(open_p);
	}
	std::cout << "Number of vertices: " << mesh->n_vertices() << std::endl;
	std::cout << "Number of faces: " << mesh->n_faces() << std::endl;
	for (int i = 0; i < mesh->n_faces(); i++)
	{
		MyMesh::VertexHandle v[3];
		bool is_vaild = true;
		for (int j = 0; j < 3; j++)
		{
			int v_id = mesh->F(i)->m_v[j];
			if (v_id == -1)
			{
				is_vaild = false;
				break;
			}
			v[j] = open_mesh.vertex_handle(mesh->F(i)->m_v[j]);
			std::cout << "Index of v[j]: " << v[j].idx() << std::endl;
		}
		if (is_vaild)
		{
			if (v[0] == v[1] || v[0] == v[2] || v[1] == v[2])
			{
				continue;
			}
			open_mesh.add_face(v[0], v[1], v[2]);
		}
	}
	open_mesh.update_normals();
	_self->SetMesh(open_mesh);
}
boost::shared_ptr<sn3DCore::sn3DMeshData> FillHoleInterface::GetMesh()
{
	boost::shared_ptr<sn3DCore::sn3DMeshData> mesh(new sn3DCore::sn3DMeshData());
	MyMesh open_mesh = _self->GetMesh();
	for (auto v_it = open_mesh.vertices_begin(); v_it!=open_mesh.vertices_end();++v_it)
	{
		MyMesh::Point open_p = open_mesh.point(*v_it);
		Point3f p(open_p[0], open_p[1], open_p[2]);
		mesh->AddVertex(p);
	}
	for (auto f_it = open_mesh.faces_begin(); f_it != open_mesh.faces_end(); ++f_it)
	{
		int v[3];
		int i = 0;
		for (auto v_it = open_mesh.fv_iter(*f_it); v_it.is_valid(); ++v_it)
		{
			v[i] = v_it->idx();
			i++;
		}
		mesh->AddFace(v[0], v[1], v[2]);
	}
	mesh->DirtyTopology();
	return mesh;
}
std::vector<std::vector<int>> FillHoleInterface::CheckBoundarys()
{
	std::vector<std::vector<MyMesh::VertexHandle>> open_boundarys = _self->CheckBoundarys();
	std::vector<std::vector<int>> boundarys;
	for (auto it : open_boundarys)
	{
		std::vector<int> cur_border;
		for (auto iit : it)
		{
			cur_border.push_back(iit.idx());
		}
		boundarys.push_back(cur_border);
	}
	return boundarys;
}
int FillHoleInterface::GetHoleNumber()
{
	return _self->GetHoleNumber();
}
void FillHoleInterface::ApplyFillHole()
{
	_self->ApplyFillHole();
}
void FillHoleInterface::ApplyFillHole(int index)
{
	_self->ApplyFillHole(index);
}
void FillHoleInterface::MeshRefinement()
{
	_self->MeshRefinement();
}