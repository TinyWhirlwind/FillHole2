#ifndef _BOX2_
#define _BOX2_

#include <assert.h>
#include "base.h"
#include "point2.h"
namespace sn3DCore
{
/*��ά��Χ��ģ���� 	BoxScalarType ��������*/
template <class BoxScalarType>
class Box2
{
public:
	
	typedef BoxScalarType ScalarType;

    Point2<BoxScalarType> min;//��С�����
		
    Point2<BoxScalarType> max;//��������
	
	inline  Box2() { min.X()= 1; max.X()= -1; min.Y()= 1; max.Y()= -1; }
/*�������캯��*/	
	inline  Box2( const Box2 & b ) { min=b.min; max=b.max; }
	inline ~Box2() { }	
	inline bool operator == ( Box2 const & p ) const
	{
		return min==p.min && max==p.max;
	}
		
	void Set( const Point2<BoxScalarType> & p )
	{
		min = max = p;
	}
/*��ʼ������*/
	inline void Set( BoxScalarType minx, BoxScalarType miny, BoxScalarType maxx, BoxScalarType maxy )
	{
		min[0] = minx;
		min[1] = miny;
		max[0] = maxx;
		max[1] = maxy;
	}
/*����Χ����Ϊ��Ч*/
	void SetNull()
	{
		 min.X()= 1; max.X()= -1; min.Y()= 1; max.Y()= -1;
	}
/*��Χ�мӷ�*/
	void Add( Box2 const & b )
	{
		if(IsNull())
		{
			min=b.min;
			max=b.max;
		}
		else
		{
			if(min.X() > b.min.X()) min.X() = b.min.X();
			if(min.Y() > b.min.Y()) min.Y() = b.min.Y();

			if(max.X() < b.max.X()) max.X() = b.max.X();
			if(max.Y() < b.max.Y()) max.Y() = b.max.Y();
		}
	}
/*����һ����*/
	void Add( const Point2<BoxScalarType> & p )
	{
		if(IsNull()) Set(p);
		else 
		{
			if(min.X() > p.X()) min.X() = p.X();
			if(min.Y() > p.Y()) min.Y() = p.Y();

			if(max.X() < p.X()) max.X() = p.X();
			if(max.Y() < p.Y()) max.Y() = p.Y();
		}
	}
/*��Χ����*/
	void Intersect( const Box2 & b )
	{
		if(min.X() < b.min.X()) min.X() = b.min.X();
		if(min.Y() < b.min.Y()) min.Y() = b.min.Y();

		if(max.X() > b.max.X()) max.X() = b.max.X();
		if(max.Y() > b.max.Y()) max.Y() = b.max.Y();

		if(min.X()>max.X() || min.Y()>max.Y()) SetNull();
	}
/*ƽ�ư�Χ��*/
	void Translate( const Point2<BoxScalarType> & p )
	{
		min += p;
		max += p;
	}
/*�����Ƿ��ڰ�Χ����*/
	bool IsIn( Point2<BoxScalarType> const & p ) const
	{
		return (
			min.X() <= p.X() && p.X() <= max.X() &&
			min.Y() <= p.Y() && p.Y() <= max.Y()
		);
	}
/*�����ڰ�Χ������*/
	bool IsInEx( Point2<BoxScalarType> const & p ) const
	{
		return  (
			min.X() <= p.X() && p.X() < max.X() &&
			min.Y() <= p.Y() && p.Y() < max.Y() 
		);
	}
/*���������Χ���Ƿ���ײ*/
	bool Collide( Box2 const &b )
	{
		Box2 bb=*this;
		bb.Intersect(b);
		return bb.IsValid();
	}
/*����Χ���Ƿ���Ч*/
	inline bool IsNull() const { return min.X()>max.X() || min.Y()>max.Y(); }
/*����Χ���Ƿ���Ч*/
	inline bool IsValid() const { return min.X()<max.X() && min.Y()<max.Y(); }
/*����Χ���Ƿ�Ϊ��*/
	inline bool IsEmpty() const { return min==max; }
/*���ذ�Χ�жԽ��߳���*/
	BoxScalarType Diag() const
	{
		return Distance(min,max);
	}
/*���ذ�Χ������*/
	Point2<BoxScalarType> Center() const
	{
		return (min+max)/2;
	}
/*���ذ�Χ�����*/
	inline BoxScalarType Area() const
	{
		return (max[0]-min[0])*(max[1]-min[1]);
	}
/*���ذ�Χ��X���򳤶�*/
	inline BoxScalarType DimX() const { return max.X()-min.X(); }
/*���ذ�Χ��Y���򳤶�*/
	inline BoxScalarType DimY() const { return max.Y()-min.Y(); }

/*���ذ�Χ�жԽ�������*/
	inline Point2<BoxScalarType> Dim() const { return max-min; }

	inline void Normalize( Point2<BoxScalarType> & p )
	{
		p -= min;
		p[0] /= max[0]-min[0];
		p[1] /= max[1]-min[1];
	}
}; 
	
typedef Box2<short>  Box2s;
typedef Box2<int>	 Box2i;	
typedef Box2<float>  Box2f;
typedef Box2<double> Box2d;
}
#endif
