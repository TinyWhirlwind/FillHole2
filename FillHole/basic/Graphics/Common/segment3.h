#ifndef _SEGMENT3_
#define _SEGMENT3_

#include "point3.h"
#include "line3.h"
#include "box3.h"

namespace sn3DCore
{
template <class SegmentScalarType >
class Segment3
{
public: 

	/// The scalar type
	typedef SegmentScalarType ScalarType;

	/// The point type
	typedef Point3<SegmentScalarType> PointType;

	/// The point type
	typedef Segment3<SegmentScalarType> SegmentType;

private:

	/// _extrema
	PointType _p0,_p1;

public:

		/// Members to access either extrema
  inline const PointType &P0() const { return _p0; } 
  inline const PointType &P1() const { return _p1; } 
  inline PointType &P0() { return _p0; } 
  inline PointType &P1() { return _p1; } 
		/// The empty constructor
	Segment3() {};
		/// The (a,b) constructor
	Segment3(const PointType &a, const PointType &b) { _p0=a; _p1=b; };
		/// Operator to compare segments
	inline bool operator == ( SegmentType const & p ) const
	{	return _p0==p._p0 && _p1==p._p1; }
		/// Operator to dispare segments
	inline bool operator != ( SegmentType const & p ) const
	{	return _p0!=p._p0 || _p1!=p._p1; }
		/// initializes the segment with its extrema
	void Set( const PointType &a, const PointType &b)
	{	_p0=a; _p1=b;}
	  /// calculates the point of parameter t on the segment.
	  /// if t is in [0..1] returned point is inside the segment
	inline PointType P( const ScalarType t ) const
	{ return _p0 + (_p1 - _p0) * t; }
	  /// return the middle point
	inline PointType MidPoint( ) const
	{ return ( _p0 +  _p1) / ScalarType(2.0) ; }
	  /// return the bounding box
	inline Box3<ScalarType> BBox( ) const
	{ Box3<ScalarType> t; 
	  if (_p0[0]<_p1[0]) { t.min[0]=_p0[0];t.max[0]=_p1[0];} else { t.min[0]=_p1[0];t.max[0]=_p0[0];}
		if (_p0[1]<_p1[1]) { t.min[1]=_p0[1];t.max[1]=_p1[1];} else { t.min[1]=_p1[1];t.max[1]=_p0[1];}
	  if (_p0[2]<_p1[2]) { t.min[2]=_p0[2];t.max[2]=_p1[2];} else { t.min[2]=_p1[2];t.max[2]=_p0[2];}
	  return t; }
		/// returns segment length
	ScalarType Length()
	{ return (_p0 - _p1).Norm(); }
		/// return segment squared lenght
	ScalarType SquaredLength()
	{ return (_p0 - _p1).SquaredNorm(); }
	  /// flips: a-b becomes b-a
	void Flip()
	{ PointType t=_p0; _p0=_p1; _p1=t; }
	  /// importer for different line types
	template <class Q>
	inline void Import( const Segment3<Q> & b )
	{ _p0.Import( b.P0() );	_p1.Import( b.P1() );
	}
	  /// copy constructor (builds a new segment importing an existing one)
	template <class Q>
	static SegmentType Construct( const Segment3<Q> & b )
	{ return SegmentType(PointType::Construct(b.P0()), PointType::Construct(b.P1()));}

//@{
	 /** @name Linearity for 3d segments (operators +, -, *, /) **/
	inline SegmentType operator + ( SegmentType const & p) const
	{return SegmentType( _p0+p.P0(), _p1+p.P1() );}
	inline SegmentType operator - ( SegmentType const & p) const
	{return SegmentType( _p0-p.P0(), _p1-p.P1() );}
	inline SegmentType operator * ( const ScalarType s ) const
	{return SegmentType( _p0*s, _p1*s );}
	inline SegmentType operator / ( const ScalarType s ) const
	{ScalarType s0=((ScalarType)1.0)/s; return SegmentType( _p0*s0, _p1*s0 );}
//@}

}; // end class definition



typedef Segment3<short>  Segment3s;
typedef Segment3<int>	   Segment3i;
typedef Segment3<float>  Segment3f;
typedef Segment3<double> Segment3d;

/*
* Computes the minimum distance between a segment and a point
* @param[in] segment	The input segment
* @param[in] p				The input point
* @return							The distance between the segment and the point p
*/
template < class ScalarType >
ScalarType SquaredDistance(Segment3< ScalarType > &segment, Point3< ScalarType > &p)
{
	typedef typename  Point3< ScalarType > Point3t;

	Point3t	 dir = (segment.P1()-segment.P0()).Normalize();
	ScalarType h = dir * (p-segment.P0());
	if			(h<=ScalarType(0.0))	return  SquaredDistance<ScalarType>(p, segment.P0());
	else if (h>=segment.Length())	return  SquaredDistance<ScalarType>(p, segment.P1());
	else
	{
		dir = segment.P0() + dir*h;
		return  SquaredDistance<ScalarType>(p, dir);
	}
}; //end of Distance method

template <class ScalarType> 
Point3<ScalarType> ClosestPoint( Segment3<ScalarType> s, const Point3<ScalarType> & p) 
{
	 Line3<ScalarType> l;
	l.Set(s.P0(),s.P1()-s.P0());
	l.Normalize();
	Point3<ScalarType> clos= ClosestPoint<ScalarType,true>(l,p) ;//attention to call
	 Box3<ScalarType> b;
	b.Add(s.P0());
	b.Add(s.P1());
	if (b.IsIn(clos))
		return clos;
	else
	{
		ScalarType d0=(s.P0()-p).Norm();
		ScalarType d1=(s.P1()-p).Norm();
		if (d0<d1)
			return (s.P0());
		else
			return (s.P1());
	}
	
}
}
#endif
