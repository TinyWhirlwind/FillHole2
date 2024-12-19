#pragma once
#ifndef FILL_HOLE_INTERFACE_H
#define FILL_HOLE_INTERFACE_H

#ifdef MESHCHECK_EXPORTS
#define FILL_HOLE_API __declspec(dllexport)
#else
#define FILL_HOLE_API __declspec(dllimport)
#endif

#include <iostream>
#include <boost/smart_ptr.hpp>
#include <vector>
using namespace std;
namespace sn3DCore
{
	class sn3DMeshData;
}
class FillHole;
class FILL_HOLE_API FillHoleInterface
{
public:
	FillHoleInterface();
	FillHoleInterface(boost::shared_ptr<sn3DCore::sn3DMeshData> mesh);
	~FillHoleInterface();

public:
	void SetMesh(boost::shared_ptr<sn3DCore::sn3DMeshData> mesh);
	boost::shared_ptr<sn3DCore::sn3DMeshData> GetMesh();
	std::vector<std::vector<int>> CheckBoundarys();
	int GetHoleNumber();
	void ApplyFillHole();
	void ApplyFillHole(int index);
	void MeshRefinement();
private:
	FillHole* _self;
};
#endif