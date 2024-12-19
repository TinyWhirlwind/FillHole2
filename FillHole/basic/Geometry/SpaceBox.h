#ifndef _SPACEBOX_
#define _SPACEBOX_

#include <vector>
#include "Common/Point3.h"
#include "GeoData/MeshData.h"
#include "GeoData/PointsData.h"

/* �ռ��դ */
namespace sn3DCore
{
	class BoxCoord
	{
	public:
		int x;			//�ռ䷽������
		int y;
		int z;
		int entryIndex;	//�ڿռ䷽���еĴ洢����
	};

	/* CSpatialSubdivisionEntry��ʾ�ռ��ʷֵ�һ����Ԫ�� */
	class SpaceBoxEntry
	{
	public:
		SpaceBoxEntry() 
		{
			m_Size = 0; 
			m_BeginIndex = NULL; 
		}
		int m_Size;			//�÷�������������Ԫ������          
		int m_BeginIndex;	//���ݳ�����ʼ��ַ
	};


	/* �ռ仮�ָ�դ 
	*	\note
	*		����ά�Ȼ��ֵĸ�դ�����
	*/
	class CSpaceBox
	{
	public:
		/// \param
		///		res		[in]	��դ����res*res*res
		CSpaceBox(int res);

		/// ��ʼ���ռ��ʷֽṹ
		void Initialize(sn3DMeshData * m);

		void Initialize(sn3DPointsData * m);

		void Initialize(const std::vector<Point3f>& pointData);

		/// ��ȡ�ռ䳤��
		int GetSpaceLength();

		/// ��ȡ��Ԫ���С
		int GetEntrySize(int x, int y, int z);

		/// ��ȡ��С��
		Point3f GetMinPoint();

		/// ��ȡ����
		Point3f GetMaxPoint();

		/// ��ȡ��Ԫ�����(index����)
		int GetEntryItem(int x, int y, int z, int index);

		/// Clear���ڴ��������
		void Clear();

	private:
		void ComputeBBox(sn3DCore::sn3DMeshData* m, Point3f& minv, Point3f& maxv);
		void ComputeBBox(sn3DCore::sn3DPointsData* m, Point3f& minv, Point3f& maxv);
		void ComputeBBox(const std::vector<Point3f>& points, Point3f& minv, Point3f& maxv);

	public:
		std::vector<SpaceBoxEntry> m_SpaceBox;//�ռ䵥Ԫ��

		std::vector<int> m_DataPool;//����

		std::vector<int> m_EntryMap;//��Ԫ��ӳ��

		int m_DivisionNum;//�ռ��ʷֵ�����

		Point3f m_MaxCorner;//�ռ�ṹ������(��x,y,z����)

		Point3f m_MinCorner;//�ռ�ṹ����С��(��x,y,z����)

		Point3f m_Delta;//Ϊ���ܶ�Ԥ�ȼ����m_DivisionNum/(m_MaxCorner-m_MinCorner)
	};

}
#endif