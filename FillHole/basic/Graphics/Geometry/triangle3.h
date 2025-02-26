#ifndef _TRIANGLE3_
#define _TRIANGLE3_

#include "box3.h"
#include "plane3.h"
#include "segment3.h"


namespace sn3DCore
{
/*3D三角形通用模板类*/
template <class ScalarTriangleType> class Triangle3
{
public:
  typedef ScalarTriangleType ScalarType;
	typedef Point3< ScalarType > CoordType;
	/// The bounding box type
	typedef Box3<ScalarType> BoxType;

/*********************************************
    blah
    blah
**/
	Triangle3(const CoordType & c0,const CoordType & c1,const CoordType & c2){_v[0]=c0;_v[1]=c1;_v[2]=c2;}
protected:
	/// Vector of vertex pointer incident in the face
	Point3<ScalarType> _v[3];
public:

	/// Shortcut per accedere ai punti delle facce
	inline CoordType & P( const int j ) { return _v[j];}
	inline CoordType & P0( const int j ) { return _v[j];}
	inline CoordType & P1( const int j ) { return _v[(j+1)%3];}
	inline CoordType & P2( const int j ) { return _v[(j+2)%3];}
	inline const CoordType &  P( const int j ) const { return _v[j];}
	inline const CoordType &  P0( const int j ) const { return _v[j];}
	inline const CoordType &  P1( const int j ) const { return _v[(j+1)%3];}
	inline const CoordType &  P2( const int j ) const { return _v[(j+2)%3];}
	inline const CoordType & cP0( const int j ) const { return _v[j];}
	inline const CoordType & cP1( const int j ) const { return _v[(j+1)%3];}
	inline const CoordType & cP2( const int j ) const { return _v[(j+2)%3];}



	bool InterpolationParameters(const CoordType & bq, ScalarType &a, ScalarType &b, ScalarType &_c ) const{
		return InterpolationParameters(*this, bq, a, b,_c ); 
	}





/// Return the _q of the face, the return value is in [0,sqrt(3)/2] = [0 - 0.866.. ]
ScalarType QualityFace( ) const
{
	return Quality(P(0), P(1), P(2));
}

}; //end Class

/// Returns the normal to the plane passing through p0,p1,p2
template<class TriangleType>
typename TriangleType::ScalarType QualityFace(const TriangleType &t)
{
  return Quality(t.cP(0), t.cP(1), t.cP(2));
}

/** Calcola i coefficienti della combinazione convessa.
	@param bq Punto appartenente alla faccia
	@param a Valore di ritorno per il vertice V(0)
	@param b Valore di ritorno per il vertice V(1)
	@param _c Valore di ritorno per il vertice V(2)
	@return true se bq appartiene alla faccia, false altrimenti
*/
template<class TriangleType, class ScalarType>
bool InterpolationParameters(const TriangleType t,const  Point3<ScalarType> & bq, ScalarType &a, ScalarType &b, ScalarType &_c ) 
{	
const ScalarType EPSILON = ScalarType(0.000001);
#define x1 (t.P(0).X())
#define y1 (t.P(0).Y())
#define z1 (t.P(0).Z())
#define x2 (t.P(1).X())
#define y2 (t.P(1).Y())
#define z2 (t.P(1).Z())
#define x3 (t.P(2).X())
#define y3 (t.P(2).Y())
#define z3 (t.P(2).Z())
#define px (bq[0])
#define py (bq[1])
#define pz (bq[2])

     ScalarType t1  = px*y2;
     ScalarType t2  = px*y3;
     ScalarType t3  = py*x2;
     ScalarType t4  = py*x3;
     ScalarType t5  = x2*y3;
     ScalarType t6  = x3*y2;
     ScalarType t8  = x1*y2;
     ScalarType t9  = x1*y3;
     ScalarType t10 = y1*x2;
     ScalarType t11 = y1*x3;
     ScalarType t13 = t8-t9-t10+t11+t5-t6;
     if(fabs(t13)>=EPSILON)
	 {
         ScalarType t15 = px*y1;
         ScalarType t16 = py*x1;
         a =  (t1 -t2-t3 +t4+t5-t6 )/t13;
         b = -(t15-t2-t16+t4+t9-t11)/t13;
         _c =  (t15-t1-t16+t3+t8-t10)/t13;
		return true;
     }

     t1  = px*z2;
     t2  = px*z3;
     t3  = pz*x2;
     t4  = pz*x3;
     t5  = x2*z3;
     t6  = x3*z2;
     t8  = x1*z2;
     t9  = x1*z3;
     t10 = z1*x2;
     t11 = z1*x3;
     t13 = t8-t9-t10+t11+t5-t6;
     if(fabs(t13)>=EPSILON)
	 {
		ScalarType t15 = px*z1;
		ScalarType t16 = pz*x1;
		a =  (t1 -t2-t3 +t4+t5-t6 )/t13;
		b = -(t15-t2-t16+t4+t9-t11)/t13;
		_c =  (t15-t1-t16+t3+t8-t10)/t13;
		return true;
     }

     t1  = pz*y2; t2  = pz*y3;
     t3  = py*z2; t4  = py*z3;
     t5  = z2*y3; t6  = z3*y2;
     t8  = z1*y2; t9  = z1*y3;
     t10 = y1*z2; t11 = y1*z3;
     t13 = t8-t9-t10+t11+t5-t6;
     if(fabs(t13)>=EPSILON)
	 {
         ScalarType t15 = pz*y1;
         ScalarType t16 = py*z1;
         a =  (t1 -t2-t3 +t4+t5-t6 )/t13;
         b = -(t15-t2-t16+t4+t9-t11)/t13;
         _c =  (t15-t1-t16+t3+t8-t10)/t13;
		return true;
     }
	 
#undef x1
#undef y1
#undef z1
#undef x2
#undef y2
#undef z2
#undef x3
#undef y3
#undef z3
#undef px
#undef py
#undef pz

     return false;
}


/// Compute a shape quality measure of the triangle composed by points p0,p1,p2
/// It Returns 2*AreaTri/(MaxEdge^2), 
/// the range is range [0.0, 0.866] 
/// e.g. Equilateral triangle sqrt(3)/2, halfsquare: 1/2, ... up to a line that has zero quality.
template<class P3ScalarType>
P3ScalarType Quality( Point3<P3ScalarType> const &p0, Point3<P3ScalarType> const & p1,  Point3<P3ScalarType> const & p2)
{
	Point3<P3ScalarType> d10=p1-p0;
	Point3<P3ScalarType> d20=p2-p0;
	Point3<P3ScalarType> d12=p1-p2;
	Point3<P3ScalarType> x = d10^d20;

	P3ScalarType a = Norm( x );
	if(a==0) return 0; // Area zero triangles have surely quality==0;
	P3ScalarType b = SquaredNorm( d10 );
	P3ScalarType t = b;
	t = SquaredNorm( d20 ); if ( b<t ) b = t;
	t = SquaredNorm( d12 ); if ( b<t ) b = t;
	assert(b!=0.0);
	return a/b;
}


/// Compute a shape quality measure of the triangle composed by points p0,p1,p2
/// It Returns inradius/circumradius
/// the range is range [0, 1] 
/// e.g. Equilateral triangle 1, halfsquare: 0.81, ... up to a line that has zero quality.
template<class P3ScalarType>
P3ScalarType QualityRadii(Point3<P3ScalarType> const &p0,
													Point3<P3ScalarType> const &p1,
													Point3<P3ScalarType> const &p2) {

	P3ScalarType a=(p1-p0).Norm();
	P3ScalarType b=(p2-p0).Norm();
	P3ScalarType c=(p1-p2).Norm();

	P3ScalarType sum = (a + b + c)*0.5;
	P3ScalarType area2 =  sum*(a+b-sum)*(a+c-sum)*(b+c-sum);
	if(area2 <= 0) return 0;
	//circumradius: (a*b*c)/(4*sqrt(area2))
	//inradius: (a*b*c)/(4*circumradius*sum) => sqrt(area2)/sum;
	return (8*area2)/(a*b*c*sum);
}

/// Compute a shape quality measure of the triangle composed by points p0,p1,p2
/// It Returns mean ratio 2sqrt(a, b)/(a+b) where a+b are the eigenvalues of the M^tM of the
/// transformation matrix into a regular simplex
/// the range is range [0, 1] 
template<class P3ScalarType>
P3ScalarType QualityMeanRatio(Point3<P3ScalarType> const &p0,
													Point3<P3ScalarType> const &p1,
													Point3<P3ScalarType> const &p2) {

	P3ScalarType a=(p1-p0).Norm();
	P3ScalarType b=(p2-p0).Norm();
	P3ScalarType c=(p1-p2).Norm();
	P3ScalarType sum = (a + b + c)*0.5; //semiperimeter
	P3ScalarType area2 =  sum*(a+b-sum)*(a+c-sum)*(b+c-sum);
	if(area2 <= 0) return 0;
	return (4.0*sqrt(3.0)*sqrt(area2))/(a*a + b*b + c*c);
}

/// Returns the normal to the plane passing through p0,p1,p2
template<class TriangleType>
Point3<typename TriangleType::ScalarType> Normal(const TriangleType &t)
{
	return (( t.P(1) - t.P(0)) ^ (t.P(2) - t.P(0)));
}
template<class Point3Type>
Point3Type Normal( Point3Type const &p0, Point3Type const & p1,  Point3Type const & p2)
{
	return (( p1 - p0) ^ (p2 - p0));
}


/// Like the above, it returns the normal to the plane passing through p0,p1,p2, but normalized.
template<class TriangleType>
Point3<typename TriangleType::ScalarType> NormalizedNormal(const TriangleType &t)
{
	return (( t.P(1) - t.P(0)) ^ (t.P(2) - t.P(0))).Normalize();
}
template<class Point3Type>
Point3Type NormalizedNormal( Point3Type const &p0, Point3Type const & p1,  Point3Type const & p2)
{
	return (( p1 - p0) ^ (p2 - p0)).Normalize();
}



/// Return the Double of area of the triangle
// NOTE the old Area function has been removed to intentionally 
// cause compiling error that will help people to check their code...
// A some  people used Area assuming that it returns the double and some not. 
// So please check your codes!!!
// And please DO NOT Insert any Area named function here!

template<class TriangleType>
typename TriangleType::ScalarType DoubleArea(const TriangleType &t) 
{
	return Norm( (t.P(1) - t.P(0)) ^ (t.P(2) - t.P(0)) );
}

template<class TriangleType>
Point3<typename TriangleType::ScalarType> Barycenter(const TriangleType &t) 
{
	return ((t.P(0)+t.P(1)+t.P(2))/(typename TriangleType::ScalarType) 3.0);
}

template<class TriangleType>
typename TriangleType::ScalarType Perimeter(const TriangleType &t) 
{
	return Distance(t.P(0),t.P(1))+
		     Distance(t.P(1),t.P(2))+
				 Distance(t.P(2),t.P(0));
}

template<class TriangleType>
Point3<typename TriangleType::ScalarType> Circumcenter(const TriangleType &t)
{
   typename TriangleType::ScalarType a2 = (t.P(1) - t.P(2)).SquaredNorm();
   typename TriangleType::ScalarType b2 = (t.P(2) - t.P(0)).SquaredNorm();
   typename TriangleType::ScalarType c2 = (t.P(0) - t.P(1)).SquaredNorm();      
   Point3<typename TriangleType::ScalarType>c = t.P(0)*a2*(-a2 + b2 + c2) + 
                                                t.P(1)*b2*( a2 - b2 + c2) + 
                                                t.P(2)*c2*( a2 + b2 - c2);
   c /= 2*(a2*b2 + a2*c2 + b2*c2) - a2*a2 - b2*b2 - c2*c2;
   return c;
}

template<class TriangleType>
void TrianglePointDistance(const  TriangleType &t,
							const typename TriangleType::CoordType & q,
							typename TriangleType::ScalarType & dist, 
							typename TriangleType::CoordType & closest )
{
	typedef typename TriangleType::CoordType CoordType;
	typedef typename TriangleType::ScalarType ScalarType;

	CoordType clos[3];
	ScalarType distv[3];
	CoordType clos_proj;
	ScalarType distproj;

	///find distance on the plane
	 Plane3<ScalarType> plane;
	plane.Init(t.P(0),t.P(1),t.P(2));
	clos_proj=plane.Projection(q);

	///control if inside/outside
	CoordType n=(t.P(1)-t.P(0))^(t.P(2)-t.P(0));
	CoordType n0=(t.P(0)-clos_proj)^(t.P(1)-clos_proj);
	CoordType n1=(t.P(1)-clos_proj)^(t.P(2)-clos_proj);
	CoordType n2=(t.P(2)-clos_proj)^(t.P(0)-clos_proj);
	distproj=(clos_proj-q).Norm();
	if (((n*n0)>=0)&&((n*n1)>=0)&&((n*n2)>=0))
	{
		closest=clos_proj;
		dist=distproj;
		return;
	}
	

	//distance from the edges
	 Segment3<ScalarType> e0= Segment3<ScalarType>(t.P(0),t.P(1));
	 Segment3<ScalarType> e1= Segment3<ScalarType>(t.P(1),t.P(2));
	 Segment3<ScalarType> e2= Segment3<ScalarType>(t.P(2),t.P(0));
	clos[0]=ClosestPoint<ScalarType>( e0, q);
	clos[1]=ClosestPoint<ScalarType>( e1, q);
	clos[2]=ClosestPoint<ScalarType>( e2, q);
	
	distv[0]=(clos[0]-q).Norm();
	distv[1]=(clos[1]-q).Norm();
	distv[2]=(clos[2]-q).Norm();
	int min=0;

	///find minimum distance
	for (int i=1;i<3;i++)
	{
		if (distv[i]<distv[min])
			min=i;
	}

	closest=clos[min];
	dist=distv[min];
}
}
#endif

