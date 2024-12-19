#include "FillHole.h"
using namespace std;
FillHole::FillHole()
{
	mesh_.clear();
	boundarys_.clear();
}

FillHole::FillHole(MyMesh mesh) :mesh_(mesh)
{
	boundarys_.clear();
	mesh_.request_halfedge_status();
	mesh_.request_face_status();
	mesh_.request_face_normals();
	mesh_.update_normals();
	origin_faces_num = mesh_.n_faces();
	origin_vertices_num = mesh_.n_vertices();
	average_edge_length = 0.0;
}

FillHole::~FillHole()
{
	mesh_.release_face_normals();
	mesh_.release_face_status();
};

void FillHole::SetMesh(MyMesh mesh)
{
	mesh_ = mesh;
	mesh_.request_halfedge_status();
	mesh_.request_face_status();
	mesh_.request_face_normals();
	mesh_.update_normals();
	origin_faces_num = mesh_.n_faces();
	origin_vertices_num = mesh_.n_vertices();
	average_edge_length = 0.0;
}

MyMesh FillHole::GetMesh()
{
	return mesh_;
}

std::vector<std::vector<MyMesh::VertexHandle>> FillHole::CheckBoundarys()
{
	boundarys_.clear();
	boundary_edge_num = 0.0;
	average_edge_length = 0.0;
	std::vector<std::vector<MyMesh::HalfedgeHandle>> all_boundary_half_edges;
	for (MyMesh::HalfedgeIter h_it = mesh_.halfedges_begin(); h_it != mesh_.halfedges_end(); ++h_it)
	{
		MyMesh::HalfedgeHandle h = *h_it;
		std::vector<MyMesh::HalfedgeHandle> boundary_half_edges;
		if (!mesh_.is_boundary(h) || mesh_.status(h).selected())
			continue;
		do {
			mesh_.status(h).set_selected(true);
			boundary_half_edges.push_back(h);
			double edge_length = mesh_.calc_edge_length(h);
			average_edge_length += edge_length;
			boundary_edge_num++;
			h = mesh_.next_halfedge_handle(h);
			if (!h.is_valid())
			{
				break;
			}
		} while (h != *h_it);
		all_boundary_half_edges.push_back(boundary_half_edges);
	}
	average_edge_length /= boundary_edge_num;
	for (auto cur_border : all_boundary_half_edges)
	{
		MyMesh ver_mesh;
		std::vector<MyMesh::VertexHandle> cur_boundary_points;
		for (auto h_it : cur_border)
		{
			MyMesh::VertexHandle vFrom = mesh_.from_vertex_handle(h_it);
			cur_boundary_points.push_back(vFrom);
			ver_mesh.add_vertex(mesh_.point(vFrom));
		}
		boundarys_.push_back(cur_boundary_points);
	}
	return boundarys_;
}

int FillHole::GetHoleNumber()
{
	return boundarys_.size();
}

void FillHole::ApplyFillHole()
{
	if (boundarys_.size() == 0)
	{
		CheckBoundarys();
	}
	if (boundarys_.size() == 0)
		return;
	for (int i = 0; i < boundarys_.size(); i++)
	{
		ApplyFillHole(i);
	}
}

void FillHole::ApplyFillHole(int index)
{
	if (boundarys_.size() == 0)
	{
		CheckBoundarys();
	}
	if (boundarys_.size() == 0)
		return;
	std::vector<MyMesh::VertexHandle> boundary_vertex;
	boundary_vertex.assign(boundarys_[index].begin(), boundarys_[index].end());
	int v_number = boundary_vertex.size();
	min_weight_.assign(v_number, std::vector<double>(v_number, std::numeric_limits<double>::max()));
	best_k.assign(v_number, std::vector<int>(v_number, -1));
	triangles_.clear();
	MWT(boundary_vertex, 0, v_number - 1);
	RecoverTriangles(boundary_vertex, 0, v_number - 1);
	for (auto it : triangles_)
	{
		int i = std::get<0>(it);
		int k = std::get<1>(it);
		int j = std::get<2>(it);
		std::cout << boundary_vertex[i].idx() << "," << boundary_vertex[k].idx() << "," << boundary_vertex[j].idx() << std::endl;
		//std::cout << "Triangle: " << i << ", " << k << ", " << j << std::endl;
		MyMesh::FaceHandle new_face = mesh_.add_face(boundary_vertex[i], boundary_vertex[k], boundary_vertex[j]);
		add_faces_.push_back(new_face);
	}

	std::string file_path = "D:/data/FillHole/fill_hole" + to_string(index) + ".obj";
	OpenMesh::IO::write_mesh(mesh_, file_path);
}

double FillHole::MWT(std::vector<MyMesh::VertexHandle> boundary_vertex, int i, int j)
{
	if (j < i + 2)
	{
		return 0.0;
	}
	if (min_weight_[i][j] != std::numeric_limits<double>::max()) {
		return min_weight_[i][j]; // 如果已经计算过，直接返回
	}
	double cost = std::numeric_limits<double>::max();
	int min_k = -1;
	for (int k = i + 1; k < j; k++)
	{
		double weight = CalcAreaWeight(boundary_vertex[i], boundary_vertex[k], boundary_vertex[j]);
		double cur_cost = weight + MWT(boundary_vertex, i, k) + MWT(boundary_vertex, k, j);
		if (cur_cost < cost)
		{
			cost = cur_cost;
			min_k = k;
		}
	}
	if (min_k != -1)
	{
		std::cout << "Triangle: " << i << ", " << min_k << ", " << j << std::endl;
		min_weight_[i][j] = cost;
		best_k[i][j] = min_k;
	}
	return min_weight_[i][j];
}

void FillHole::RecoverTriangles(std::vector<MyMesh::VertexHandle> boundary_vertex, int i, int j) {
	if (j < i + 2) {
		return;
	}
	int k = best_k[i][j];
	if (k != -1) {
		triangles_.emplace_back(i, k, j);
		std::cout << "Triangle: " << i << ", " << k << ", " << j << std::endl;
		RecoverTriangles(boundary_vertex, i, k);
		RecoverTriangles(boundary_vertex, k, j);
	}
}

double FillHole::CalcAreaWeight(MyMesh::VertexHandle v0, MyMesh::VertexHandle v1, MyMesh::VertexHandle v2)
{
	/*double length0 = (mesh_.point(v1) - mesh_.point(v0)).norm();
	double length1 = (mesh_.point(v2) - mesh_.point(v1)).norm();
	double length2 = (mesh_.point(v0) - mesh_.point(v2)).norm();
	return (length0 + length1 + length2);*/
	/*OpenMesh::Vec3f  dir0 = mesh_.point(v1) - mesh_.point(v0);
	OpenMesh::Vec3f  dir1 = mesh_.point(v2) - mesh_.point(v0);
	double triangle_area = OpenMesh::cross(dir0, dir1).norm();*/
	std::vector<MyMesh::VertexHandle> vertex_list;
	vertex_list.push_back(v0);
	vertex_list.push_back(v1);
	vertex_list.push_back(v2);
	auto dir0 = (mesh_.point(v1) - mesh_.point(v0)).normalize();
	auto dir1 = (mesh_.point(v2) - mesh_.point(v0)).normalize();
	MyMesh::Normal current_face_normal = (dir0 % dir1).normalized();

	double sun_dihedral_angle = 0.0;
	for (int i = 0; i < 3; i++)
	{
		MyMesh::HalfedgeHandle half_handle = GetHalfEdge(vertex_list[i], vertex_list[(i + 1) % 3]);
		if (half_handle.is_valid())
		{
			MyMesh::FaceHandle opposite_face = mesh_.opposite_face_handle(half_handle);
			if (!opposite_face.is_valid())
				continue;
			MyMesh::Normal opposite_face_normal = mesh_.normal(opposite_face);
			double dihedral_angle = acos(current_face_normal | opposite_face_normal);
			sun_dihedral_angle += dihedral_angle;
		}
	}
	return sun_dihedral_angle;
}

OpenMesh::Vec3f FillHole::getFaceNormal(MyMesh::FaceHandle face)
{
	OpenMesh::Vec3f p0, p1, p2;
	auto v_iter = mesh_.fv_begin(face);
	p0 = mesh_.point(*v_iter++);
	p1 = mesh_.point(*v_iter++);
	p2 = mesh_.point(*v_iter);
	OpenMesh::Vec3f d0, d1;
	d0 = (p1 - p0).normalize();
	d1 = (p2 - p1).normalized();
	return (d0 % d1).normalize();
}

double FillHole::CalcDihedralAngleWeight(MyMesh::VertexHandle v0, MyMesh::VertexHandle v1, MyMesh::VertexHandle v2)
{
	return 0;
}

MyMesh::HalfedgeHandle FillHole::GetHalfEdge(MyMesh::VertexHandle from_v, MyMesh::VertexHandle to_v)
{
	for (auto heh : mesh_.voh_range(from_v))
	{
		if (mesh_.to_vertex_handle(heh) == to_v)
		{
			return heh;
		}
	}
	return MyMesh::HalfedgeHandle();
}

void FillHole::MeshRefinement()
{
	bool is_swap = false;
	bool is_repalce = false;
	double interior_average_length = GetInteriorEdgeAverageLength();
	do {
		is_repalce = ReplaceTriangle();
		if (!is_repalce)
		{
			break;
		}
		//relax all interior edges of the patching mesh
		is_swap = RelaxEdge();

		// Loop if edges need further relaxation
		int loop_ = 0;
		while (is_swap)
		{
			SmoothMesh();
			is_swap = RelaxEdge();
			loop_++;
			if (loop_ > 5)
			{
				is_swap = false;
				break;
			}
		}
		interior_average_length = GetInteriorEdgeAverageLength();
		// Re-check replacement only if needed after relaxation
	} while (interior_average_length > average_edge_length);

	std::cout << "output vertex number is:" << mesh_.n_vertices() << std::endl;
	std::string file_path = "D:/data/FillHole/refineMesh.obj";
	OpenMesh::IO::write_mesh(mesh_, file_path);
}

bool FillHole::ReplaceTriangle()
{
	bool is_repalce = false;
	for (auto f_it : add_faces_)
	{
		double average_length = 0.0;
		double sigma_centroid = 0.0;//sigma(c)
		std::vector<double> sigma_vertex;
		std::vector<MyMesh::VertexHandle> vertex_list;
		MyMesh::Point face_centroid{ 0.0,0.0,0.0 };
		for (auto v_it = mesh_.fv_iter(f_it); v_it.is_valid(); v_it++)
		{
			double average_length = 0.0;
			int num = 0;
			for (auto e_it = mesh_.ve_iter(*v_it); e_it.is_valid(); ++e_it)
			{
				average_length += mesh_.calc_edge_length(*e_it);
				num++;
			}
			average_length /= num;
			sigma_centroid += average_length;
			face_centroid += mesh_.point(*v_it);
			sigma_vertex.push_back(average_length);
			vertex_list.push_back(*v_it);
		}
		sigma_centroid /= 3.0;
		face_centroid /= 3.0;

		float current_average_edge_length = 0.0;
		for (int i = 0; i < 3; i++)
		{
			float edge_length = (mesh_.point(vertex_list[i]) - mesh_.point(vertex_list[(i + 1) % 3])).norm();
			current_average_edge_length += edge_length;
		}
		current_average_edge_length /= 3;
		for (int i = 0; i < 3; i++)
		{
			double alpha_vc_m = sqrt(2) * (mesh_.point(vertex_list[i]) - face_centroid).norm();
			if (sigma_vertex[i] < alpha_vc_m && sigma_centroid < alpha_vc_m && current_average_edge_length > average_edge_length)
			{
				is_repalce = true;
				//replace triangle
				std::vector<int> add_face_vertexs;
				mesh_.delete_face(f_it, false);
				mesh_.garbage_collection();
				MyMesh::VertexHandle center_vertex = mesh_.add_vertex(face_centroid);
				add_vertices_.push_back(center_vertex.idx());
				MyMesh::FaceHandle add_face_handle[3];
				add_face_handle[0] = mesh_.add_face(center_vertex, vertex_list[0], vertex_list[1]);
				add_face_handle[1] = mesh_.add_face(center_vertex, vertex_list[1], vertex_list[2]);
				add_face_handle[2] = mesh_.add_face(center_vertex, vertex_list[2], vertex_list[0]);
				//relax the edge
				for (int i = 0; i < 3; i++)
				{
					MyMesh::HalfedgeHandle cw_edge = GetHalfEdge(vertex_list[i], vertex_list[(i + 1) % 3]);
					MyMesh::HalfedgeHandle ccw_edge = mesh_.opposite_halfedge_handle(cw_edge);
					if (!cw_edge.is_valid() || !ccw_edge.is_valid())
					{
						std::cout << "no find halfedge";
						continue;
					}
					MyMesh::FaceHandle cw_face = mesh_.face_handle(cw_edge);
					MyMesh::FaceHandle ccw_face = mesh_.face_handle(ccw_edge);
					MyMesh::VertexHandle cw_non_mutual_vertex = mesh_.to_vertex_handle(mesh_.next_halfedge_handle(cw_edge));
					MyMesh::VertexHandle ccw_non_mutual_vertex = mesh_.to_vertex_handle(mesh_.next_halfedge_handle(ccw_edge));
					double half_edge_length = mesh_.calc_edge_length(cw_edge) * 0.5f;
					MyMesh::Point edge_to_vertex = mesh_.point(mesh_.to_vertex_handle(cw_edge));
					MyMesh::Point edge_from_vertex = mesh_.point(mesh_.from_vertex_handle(cw_edge));
					MyMesh::Point edge_center = (edge_to_vertex + edge_from_vertex) / 2;
					double cw_distance = (mesh_.point(cw_non_mutual_vertex) - edge_center).norm();
					double ccw_distance = (mesh_.point(ccw_non_mutual_vertex) - edge_center).norm();
					if (cw_distance < half_edge_length || ccw_distance < half_edge_length)
					{
						auto edge_handle = mesh_.edge_handle(cw_edge);
						if (mesh_.is_flip_ok(edge_handle))
						{
							//mesh_.flip(edge_handle);
						}
					}
				}
				break;
			}
		}
	}
	mesh_.update_normals();
	add_faces_.clear();
	for (int i = origin_faces_num; i < mesh_.n_faces(); i++)
	{
		auto face_handle = mesh_.face_handle(i);
		add_faces_.push_back(face_handle);
	}
	return is_repalce;
}

void FillHole::SmoothMesh()
{
	for (int i = origin_vertices_num; i < mesh_.n_vertices(); i++)
	{
		auto vertex_handle = mesh_.vertex_handle(i);
		MyMesh::Point new_p = MyMesh::Point(0, 0, 0);
		int ring_num = 0;
		for (auto vv_it = mesh_.vv_iter(vertex_handle); vv_it.is_valid(); ++vv_it)
		{
			new_p += mesh_.point(*vv_it);
			ring_num++;
		}
		new_p /= ring_num;
		mesh_.set_point(vertex_handle, new_p);
	}
	mesh_.update_normals();
}

void FillHole::outputPatchMesh()
{
	MyMesh ver_mesh_;
	for (auto itor : add_faces_)
	{
		MyMesh::VertexHandle addV[3];
		int i = 0;
		for (auto v_it = mesh_.fv_iter(itor); v_it.is_valid(); v_it++)
		{
			addV[i] = ver_mesh_.add_vertex(mesh_.point(*v_it));
			i++;
		}
		ver_mesh_.add_face(addV[0], addV[1], addV[2]);
	}
	std::string file_path = "D:/data/FillHole/addMesh.obj";
	OpenMesh::IO::write_mesh(ver_mesh_, file_path);
}

bool FillHole::RelaxEdge()
{
	bool is_swap = false;
	if (add_faces_.size() == 0)
	{
		return is_swap;
	}
	//filter all interior edges
	std::map<MyMesh::EdgeHandle, int> patch_mesh_edge;
	for (auto f_it : add_faces_)
	{
		for (auto e_it = mesh_.fe_iter(f_it); e_it.is_valid(); ++e_it)
		{
			patch_mesh_edge[*e_it]++;
		}
	}
	//relax all interior edges of the patching mesh
	for (auto itor : patch_mesh_edge)
	{
		if (itor.second <= 1)
			continue;
		MyMesh::HalfedgeHandle cw_edge = mesh_.halfedge_handle(itor.first, 0);
		MyMesh::HalfedgeHandle ccw_edge = mesh_.halfedge_handle(itor.first, 1);
		if (!cw_edge.is_valid() || !ccw_edge.is_valid())
		{
			std::cout << "no find halfedge";
			continue;
		}
		MyMesh::FaceHandle cw_face = mesh_.face_handle(cw_edge);
		MyMesh::FaceHandle ccw_face = mesh_.face_handle(ccw_edge);
		MyMesh::VertexHandle cw_non_mutual_vertex = mesh_.to_vertex_handle(mesh_.next_halfedge_handle(cw_edge));
		MyMesh::VertexHandle ccw_non_mutual_vertex = mesh_.to_vertex_handle(mesh_.next_halfedge_handle(ccw_edge));
		double half_edge_length = mesh_.calc_edge_length(cw_edge) * 0.5f;
		MyMesh::Point edge_to_vertex = mesh_.point(mesh_.to_vertex_handle(cw_edge));
		MyMesh::Point edge_from_vertex = mesh_.point(mesh_.from_vertex_handle(cw_edge));
		MyMesh::Point edge_center = (edge_to_vertex + edge_from_vertex) / 2;
		double cw_distance = (mesh_.point(cw_non_mutual_vertex) - edge_center).norm();
		double ccw_distance = (mesh_.point(ccw_non_mutual_vertex) - edge_center).norm();
		if (cw_distance < half_edge_length || ccw_distance < half_edge_length)
		{
			auto edge_handle = mesh_.edge_handle(cw_edge);
			if (mesh_.is_flip_ok(edge_handle))
			{
				mesh_.flip(edge_handle);
				is_swap = true;
			}
		}
	}
	return is_swap;
}

double FillHole::GetInteriorEdgeAverageLength()
{
	double average_length = 0.0;
	std::vector<MyMesh::EdgeHandle> patch_mesh_edge;
	for (auto f_it : add_faces_)
	{
		for (auto e_it = mesh_.fe_iter(f_it); e_it.is_valid(); ++e_it)
		{
			patch_mesh_edge.push_back(*e_it);
		}
	}
	for (auto e_it : patch_mesh_edge)
	{
		double cur_length = mesh_.calc_edge_length(e_it);
		average_length += cur_length;
	}
	average_length /= patch_mesh_edge.size();
	return average_length;
}