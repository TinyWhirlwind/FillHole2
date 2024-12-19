#ifndef _COORDINATESYSTEM_
#define _COORDINATESYSTEM_

#include "Common/Point3.h"
#include "SceneObject/SceneObject.h"

/* ��������ϵ���� */
namespace sn3DCore
{
	class CoordinateSystem : public SceneObject
	{
	public:
		void Render();

	private:
		void DrawCoordinateSystem();

		void Cone(int slices, float lenght, float width); //����Բ׶,x����

		void Cylinder(int slices, float lenght, float width); //����Բ��,x����

		void Arrow(float body_lengtht, float body_width, float head_lenght, float head_width, int body_slice = 50, int head_slice = 50);//���Ƽ�ͷ,x����

		void MoveArrow(float scale, int body_slice = 50, int head_slice = 50);

		void RotateArrow(float scale, int body_slice = 50, int head_slice = 50);

		void DrawX();
		void DrawY();
		void DrawZ();

	};
}
#endif