#ifndef _INTERSECTION3_
#define _INTERSECTION3_

#include "base.h"
#include "point3.h"
#include "line3.h"
#include "ray3.h"
#include "plane3.h"
#include "segment3.h"
#include "sphere3.h"
#include "triangle3.h"
#include "triangle_triangle3.h"

/** 
    Function computing the intersection between couple of geometric primitives in
    3 dimension
*/
  /// interseciton between sphere and line
namespace sn3DCore
{
  template<class T>
    inline bool IntersectionLineSphere( const Sphere3<T> & sp, const Line3<T> & li, Point3<T> & p0,Point3<T> & p1 ){

    // Per prima cosa si sposta il sistema di riferimento 
    // fino a portare il centro della sfera nell'origine
    Point3<T> neworig=li.Origin()-sp.Center();
    // poi si risolve il sistema di secondo grado (con maple...)
    T t1 = li.Direction().X()*li.Direction().X();
    T t2 = li.Direction().Y()*li.Direction().Y();
    T t3 = li.Direction().Z()*li.Direction().Z();
    T t6 = neworig.Y()*li.Direction().Y();
    T t7 = neworig.X()*li.Direction().X();
    T t8 = neworig.Z()*li.Direction().Z();
    T t15 = sp.Radius()*sp.Radius();
    T t17 = neworig.Z()*neworig.Z();
    T t19 = neworig.Y()*neworig.Y();
    T t21 = neworig.X()*neworig.X();
    T t28 = T(2.0*t7*t6+2.0*t6*t8+2.0*t7*t8+t1*t15-t1*t17-t1*t19-t2*t21+t2*t15-t2*t17-t3*t21+t3*t15-t3*t19);
    if(t28<0) return false;
    T t29 = sqrt(t28);      
    T val0 = 1/(t1+t2+t3)*(-t6-t7-t8+t29); 
    T val1 = 1/(t1+t2+t3)*(-t6-t7-t8-t29);

    p0=li.P(val0);
    p1=li.P(val1);
    return true;
  }

	/*
	* Function computing the intersection between a sphere and a segment.
	* @param[in]	sphere				the sphere
	* @param[in]	segment				the segment
	* @param[out]	intersection  the intersection point, meaningful only if the segment intersects the sphere
	* \return			(0, 1 or 2)		the number of intersections between the segment and the sphere. 
	*														t1 is a valid intersection only if the returned value is at least 1; 
	*														similarly t2 is valid iff the returned value is 2.
	*/
	template < class SCALAR_TYPE >
	inline int IntersectionSegmentSphere(const Sphere3<SCALAR_TYPE>& sphere, const Segment3<SCALAR_TYPE>& segment, Point3<SCALAR_TYPE> & t0, Point3<SCALAR_TYPE> & t1)
	{
		typedef SCALAR_TYPE													ScalarType;
		typedef typename  Point3< ScalarType >	Point3t;

		Point3t s = segment.P0() - sphere.Center();
		Point3t r = segment.P1() - segment.P0();

		ScalarType rho2 = sphere.Radius()*sphere.Radius();

		ScalarType sr = s*r;
		ScalarType r_squared_norm = r.SquaredNorm(); 
		ScalarType s_squared_norm = s.SquaredNorm(); 
		ScalarType sigma = sr*sr - r_squared_norm*(s_squared_norm-rho2);

		if (sigma<ScalarType(0.0)) // the line containing the edge doesn't intersect the sphere
			return 0;

		ScalarType sqrt_sigma = ScalarType(sqrt( ScalarType(sigma) ));
		ScalarType lambda1 = (-sr - sqrt_sigma)/r_squared_norm;
		ScalarType lambda2 = (-sr + sqrt_sigma)/r_squared_norm;

		int solution_count = 0;
		if (ScalarType(0.0)<=lambda1 && lambda1<=ScalarType(1.0))
		{
			ScalarType t_enter =  math::Max< ScalarType >(lambda1, ScalarType(0.0));
			t0 = segment.P0() + r*t_enter;
			solution_count++;
		}

		if (ScalarType(0.0)<=lambda2 && lambda2<=ScalarType(1.0))
		{
			Point3t *pt = (solution_count>0) ? &t1 : &t0;
			ScalarType t_exit  =  math::Min< ScalarType >(lambda2, ScalarType(1.0));
			*pt = segment.P0() + r*t_exit;
			solution_count++;
		}
		return solution_count;
	}; // end of IntersectionSegmentSphere

	
	/*!
	* Compute the intersection between a sphere and a triangle.
	* \param[in]	sphere		the input sphere
	* \param[in]	triangle	the input triangle
	* \param[out]	witness		it is the point on the triangle nearest to the center of the sphere (even when there isn't intersection)
	* \param[out] res				if not null, in the first item is stored the minimum distance between the triangle and the sphere,
	*                       while in the second item is stored the penetration depth
	* \return			true			iff there is an intersection between the sphere and the triangle
	*/
	template < class SCALAR_TYPE >
	bool IntersectionSphereTriangle(const  Sphere3	< SCALAR_TYPE >		& sphere  ,
																	const  Triangle3< SCALAR_TYPE >		& triangle, 
																	 Point3					< SCALAR_TYPE >		& witness ,
																	std::pair< SCALAR_TYPE, SCALAR_TYPE > * res=NULL)
	{
		typedef SCALAR_TYPE														ScalarType;
		typedef typename  Point3< ScalarType >		Point3t;
		typedef typename  Triangle3< ScalarType > Triangle3t;

		bool penetration_detected = false;

		ScalarType radius = sphere.Radius();
		Point3t	center = sphere.Center();
		Point3t p0 = triangle.P(0)-center;
		Point3t p1 = triangle.P(1)-center;
		Point3t p2 = triangle.P(2)-center;

		Point3t p10 = p1-p0;
		Point3t p21 = p2-p1;
		Point3t p20 = p2-p0;

		ScalarType delta0_p01 =  p10*p1;
		ScalarType delta1_p01 = -p10*p0;
		ScalarType delta0_p02 =  p20*p2;
		ScalarType delta2_p02 = -p20*p0;
		ScalarType delta1_p12 =  p21*p2;
		ScalarType delta2_p12 = -p21*p1;

		// the closest point can be one of the vertices of the triangle
		if			(delta1_p01<=ScalarType(0.0) && delta2_p02<=ScalarType(0.0)) { witness = p0; }
		else if (delta0_p01<=ScalarType(0.0) && delta2_p12<=ScalarType(0.0)) { witness = p1; }
		else if (delta0_p02<=ScalarType(0.0) && delta1_p12<=ScalarType(0.0)) { witness = p2; }
		else
		{
			ScalarType temp = p10*p2;
			ScalarType delta0_p012 = delta0_p01*delta1_p12 + delta2_p12*temp;
			ScalarType delta1_p012 = delta1_p01*delta0_p02 - delta2_p02*temp;
			ScalarType delta2_p012 = delta2_p02*delta0_p01 - delta1_p01*(p20*p1);

			// otherwise, can be a point lying on same edge of the triangle
			if (delta0_p012<=ScalarType(0.0)) 
			{
				ScalarType denominator = delta1_p12+delta2_p12;
				ScalarType mu1 = delta1_p12/denominator;
				ScalarType mu2 = delta2_p12/denominator;
				witness = (p1*mu1 + p2*mu2);
			}
			else if (delta1_p012<=ScalarType(0.0))
			{
				ScalarType denominator = delta0_p02+delta2_p02;
				ScalarType mu0 = delta0_p02/denominator;
				ScalarType mu2 = delta2_p02/denominator;
				witness = (p0*mu0 + p2*mu2);
			}
			else if (delta2_p012<=ScalarType(0.0))
			{
				ScalarType denominator = delta0_p01+delta1_p01;
				ScalarType mu0 = delta0_p01/denominator;
				ScalarType mu1 = delta1_p01/denominator;
				witness = (p0*mu0 + p1*mu1);
			}
			else
			{
				// or else can be an point internal to the triangle
				ScalarType denominator =  delta0_p012 + delta1_p012 + delta2_p012;
				ScalarType lambda0 = delta0_p012/denominator;
				ScalarType lambda1 = delta1_p012/denominator;
				ScalarType lambda2 = delta2_p012/denominator;
				witness = p0*lambda0 + p1*lambda1 + p2*lambda2;
			}
		}

		if (res!=NULL)
		{
			ScalarType witness_norm = witness.Norm();
			res->first  =  math::Max< ScalarType >( witness_norm-radius, ScalarType(0.0) );
			res->second =  math::Max< ScalarType >( radius-witness_norm, ScalarType(0.0) );
		}
		penetration_detected = (witness.SquaredNorm() <= (radius*radius));
		witness += center;
		return penetration_detected;
	}; //end of IntersectionSphereTriangle


  /// intersection between line and plane
  template<class T>
    inline bool IntersectionLinePlane( const Plane3<T> & pl, const Line3<T> & li, Point3<T> & po){
    const T epsilon = T(1e-8);

    T k = pl.Direction() * li.Direction();						// Compute 'k' factor
    if( (k > -epsilon) && (k < epsilon))
      return false;
    T r = (pl.Offset() - pl.Direction()*li.Origin())/k;	// Compute ray distance
    po = li.Origin() + li.Direction()*r;
    return true;
  }
	
   /// intersection between segment and plane
  template<class T>
    inline bool IntersectionSegmentPlane( const Plane3<T> & pl, const Segment3<T> & s, Point3<T> & po){
	 Line3<T> l;
	l.Set(s.P0(),s.P1()-s.P0());
	l.Normalize();
	if (IntersectionLinePlane(pl,l,po))
		return((po-s.P0()).Norm()<=(s.P0()-s.P1()).Norm());
    return false;
  }

  /// intersection between segment and plane
  template<typename SEGMENTTYPE>
    inline bool Intersection( const Plane3<typename SEGMENTTYPE::ScalarType> & pl, 
			      const SEGMENTTYPE & sg, 
			      Point3<typename SEGMENTTYPE::ScalarType> & po){
    typedef typename SEGMENTTYPE::ScalarType T;
    const T epsilon = T(1e-8);

    T k = pl.Direction() * (sg.P1()-sg.P0());
    if( (k > -epsilon) && (k < epsilon))
      return false;
    T r = (pl.Offset() - pl.Direction()*sg.P0())/k;	// Compute ray distance
    if( (r<0) || (r > 1.0))
      return false;
    po = sg.P0()*(1-r)+sg.P1() * r;
    return true;
  }

  /// intersection between plane and triangle 
  // not optimal: uses plane-segment intersection (and the fact the two or none edges can be intersected)
  template<typename TRIANGLETYPE> 
    inline bool IntersectionPlaneTriangle( const Plane3<typename TRIANGLETYPE::ScalarType> & pl, 
			      const  TRIANGLETYPE & tr, 
			      Segment3<typename TRIANGLETYPE::ScalarType> & sg){
    typedef typename TRIANGLETYPE::ScalarType T;
    if(Intersection(pl,Segment3<T>(tr.P(0),tr.P(1)),sg.P0())){
      if(Intersection(pl,Segment3<T>(tr.P(0),tr.P(2)),sg.P1()))
	return true;
      else
	{
	  Intersection(pl,Segment3<T>(tr.P(1),tr.P(2)),sg.P1());
	  return true;
	}
    }else
      {
	if(Intersection(pl,Segment3<T>(tr.P(1),tr.P(2)),sg.P0()))
	  {
	    Intersection(pl,Segment3<T>(tr.P(0),tr.P(2)),sg.P1());
	    return true;
	  }
      }
    return false;
  }

  /// intersection between two triangles
  template<typename TRIANGLETYPE> 
    inline bool Intersection(const TRIANGLETYPE & t0,const TRIANGLETYPE & t1){
    return NoDivTriTriIsect(t0.P0(0),t0.P0(1),t0.P0(2),
			    t1.P0(0),t1.P0(1),t1.P0(2));
  }
  template<class T>
    inline bool Intersection(Point3<T> V0,Point3<T> V1,Point3<T> V2,
			     Point3<T> U0,Point3<T> U1,Point3<T> U2){
    return NoDivTriTriIsect(V0,V1,V2,U0,U1,U2);
  }

  template<class T>
    inline bool Intersection(Point3<T> V0,Point3<T> V1,Point3<T> V2,
			     Point3<T> U0,Point3<T> U1,Point3<T> U2,int *coplanar,
			     Point3<T> &isectpt1,Point3<T> &isectpt2){

    return tri_tri_intersect_with_isectline(V0,V1,V2,U0,U1,U2,
					    coplanar,isectpt1,isectpt2);
  }

  template<typename TRIANGLETYPE,typename SEGMENTTYPE >
    inline bool Intersection(const TRIANGLETYPE & t0,const TRIANGLETYPE & t1,bool &coplanar,
			     SEGMENTTYPE  & sg){
    Point3<typename SEGMENTTYPE::PointType> ip0,ip1; 
    return  tri_tri_intersect_with_isectline(t0.P0(0),t0.P0(1),t0.P0(2),
					     t1.P0(0),t1.P0(1),t1.P0(2),
					     coplanar,sg.P0(),sg.P1()
					     );              
  }

	
  // ray-triangle, gives barycentric coords of intersection and distance along ray
template<class T>
bool Intersection( const Line3<T> & ray, const Point3<T> & vert0, 
				  const Point3<T> & vert1, const Point3<T> & vert2,
				  T & a ,T & b, T & dist)
{
	// small (hum) borders around triangle
	const T EPSILON2= T(1e-8); 
	
	const T EPSILON = T(1e-8);
	
	Point3<T> edge1 = vert1 - vert0;
	Point3<T> edge2 = vert2 - vert0;

	// determinant 
	Point3<T> pvec  = ray.Direction() ^ edge2;

	T det = edge1*pvec;

	// if determinant is near zero, ray lies in plane of triangle
  if (fabs(det) < EPSILON) return false;

  // calculate distance from vert0 to ray origin 
  Point3<T> tvec = ray.Origin()- vert0;

  // calculate A parameter and test bounds 
  a = tvec * pvec;
  if (a < -EPSILON2*det || a > det+det*EPSILON2) return false;

  // prepare to test V parameter 
  Point3<T> qvec = tvec ^ edge1;

  // calculate B parameter and test bounds 
  b = ray.Direction() * qvec ;
  if (b < -EPSILON2*det || b + a > det+det*EPSILON2) return false;

  // calculate t, scale parameters, ray intersects triangle 
  dist = edge2 * qvec;
	if (dist<0) return false;
  T inv_det = T(1.0) / det;
  dist *= inv_det;
  a *= inv_det;
  b *= inv_det;

  return true;
}

  // ray-triangle, gives barycentric coords of intersection and distance along ray.
	// Ray3 type used.
template<class T>
bool Intersection( const Ray3<T> & ray, const Point3<T> & vert0, 
				  const Point3<T> & vert1, const Point3<T> & vert2,
				  T & a ,T & b, T & dist)
{
	// small (hum) borders around triangle
	const T EPSILON2= T(1e-8); 

	const T EPSILON = T(1e-8);
	
	Point3<T> edge1 = vert1 - vert0;
	Point3<T> edge2 = vert2 - vert0;

	// determinant 
	Point3<T> pvec  = ray.Direction() ^ edge2;

	T det = edge1*pvec;

	// if determinant is near zero, ray lies in plane of triangle
  if (fabs(det) < EPSILON) return false;

  // calculate distance from vert0 to ray origin 
  Point3<T> tvec = ray.Origin()- vert0;

  // calculate A parameter and test bounds 
  a = tvec * pvec;
  if (a < -EPSILON2*det || a > det+det*EPSILON2) return false;

  // prepare to test V parameter 
  Point3<T> qvec = tvec ^ edge1;

  // calculate B parameter and test bounds 
  b = ray.Direction() * qvec ;
  if (b < -EPSILON2*det || b + a > det+det*EPSILON2) return false;

  // calculate t, scale parameters, ray intersects triangle 
  dist = edge2 * qvec;
	if (dist<0) return false;
  T inv_det = T(1.0) / det;
  dist *= inv_det;
  a *= inv_det;
  b *= inv_det;

  return true;
}

// ray-triangle, gives intersection 3d point and distance along ray
template<class T>
bool Intersection( const Line3<T> & ray, const Point3<T> & vert0, 
				  const Point3<T> & vert1, const Point3<T> & vert2,
			   Point3<T> & inte)
{

	// small (hum) borders around triangle
	const T EPSILON2= T(1e-8); 

	const T EPSILON = T(1e-8);
	
	Point3<T> edge1 = vert1 - vert0;
	Point3<T> edge2 = vert2 - vert0;

	// determinant 
	Point3<T> pvec  = ray.Direction() ^ edge2;

	T det = edge1*pvec;

	// if determinant is near zero, ray lies in plane of triangle
  if (fabs(det) < EPSILON) return false;

  // calculate distance from vert0 to ray origin 
  Point3<T> tvec = ray.Origin() - vert0;

  // calculate A parameter and test bounds 
  T a = tvec * pvec;
  if (a < -EPSILON2*det || a > det+det*EPSILON2) return false;

  // prepare to test V parameter 
  Point3<T> qvec = tvec ^ edge1;

  // calculate B parameter and test bounds 
  T b = ray.Direction() * qvec ;
  if (b < -EPSILON2*det || b + a > det+det*EPSILON2) return false;

  // calculate t, scale parameters, ray intersects triangle 
  double dist = edge2 * qvec;
	//if (dist<0) return false;
  T inv_det = 1.0 / det;
  dist *= inv_det;
  a *= inv_det;
  b *= inv_det;

	inte = vert0 + edge1*a + edge2*b;
  return true;
}

// line-box
template<class T>
bool Intersection_Line_Box( const Box3<T> & box, const Line3<T> & r, Point3<T> & coord )
{
	const int NUMDIM = 3;
	const int RIGHT  = 0;
	const int LEFT	 = 1;
	const int MIDDLE = 2;

	int inside = 1;
	char quadrant[NUMDIM];
    int i;
    int whichPlane;
    Point3<T> maxT,candidatePlane;
    
	// Find candidate planes; this loop can be avoided if
   	// rays cast all from the eye(assume perpsective view)
    for (i=0; i<NUMDIM; i++)
    {
        if(r.Origin()[i] < box.min[i])
		{
			quadrant[i] = LEFT;
			candidatePlane[i] = box.min[i];
			inside = 0;
		}
		else if (r.Origin()[i] > box.max[i])
		{
			quadrant[i] = RIGHT;
			candidatePlane[i] = box.max[i];
			inside = 0;
		}
		else
		{
			quadrant[i] = MIDDLE;
		}
    }

		// Ray origin inside bounding box
	if(inside){
	    coord = r.Origin();
	    return true;
	}

	// Calculate T distances to candidate planes 
    for (i = 0; i < NUMDIM; i++)
    {
		if (quadrant[i] != MIDDLE && r.Direction()[i] !=0.)
			maxT[i] = (candidatePlane[i]-r.Origin()[i]) / r.Direction()[i];
		else
			maxT[i] = -1.;
    }

	// Get largest of the maxT's for final choice of intersection
    whichPlane = 0;
    for (i = 1; i < NUMDIM; i++)
	    if (maxT[whichPlane] < maxT[i])
			whichPlane = i;

	// Check final candidate actually inside box 
    if (maxT[whichPlane] < 0.) return false;
    for (i = 0; i < NUMDIM; i++)
		if (whichPlane != i)
		{
			coord[i] = r.Origin()[i] + maxT[whichPlane] *r.Direction()[i];
			if (coord[i] < box.min[i] || coord[i] > box.max[i])
				return false;
		}
		else
		{
			coord[i] = candidatePlane[i];
		}
    return true;			// ray hits box
}	

// ray-box
template<class T>
bool Intersection_Ray_Box( const Box3<T> & box, const Ray3<T> & r, Point3<T> & coord )
{
	Line3<T> l;
	l.SetOrigin(r.Origin());
	l.SetDirection(r.Direction());
	return(Intersection_Line_Box<T>(box,l,coord));
}	

// segment-box return fist intersection found  from p0 to p1
template<class ScalarType>
bool Intersection_Segment_Box( const Box3<ScalarType> & box, 
							  const Segment3<ScalarType> & s, 
							  Point3<ScalarType> & coord )
{
	//as first perform box-box intersection
	Box3<ScalarType> test;
	test.Add(s.P0());
	test.Add(s.P1());
	if (!test.Collide(box))
		return false;
	else
	{
		Line3<ScalarType> l;
		Point3<ScalarType> dir=s.P1()-s.P0();
		dir.Normalize();
		l.SetOrigin(s.P0());
		l.SetDirection(dir);
		if(Intersection_Line_Box<ScalarType>(box,l,coord))
			return (test.IsIn(coord));
		return false;
	}
}

// segment-box intersection , return number of intersections and intersection points
template<class ScalarType>
int Intersection_Segment_Box( const Box3<ScalarType> & box, 
							 const Segment3<ScalarType> & s,
							 Point3<ScalarType> & coord0,
							 Point3<ScalarType> & coord1 )
{
	int num=0;
	Segment3<ScalarType> test= s;
	if (Intersection_Segment_Box(box,test,coord0 ))
	{
		num++;
		Point3<ScalarType> swap=test.P0();
		test.P0()=test.P1();
		test.P1()=swap;
		if (Intersection_Segment_Box(box,test,coord1 ))
			num++;
	}
	return num;
}	


// segment-triangle intersection
template<class ScalarType>
bool Intersection_Segment_Triangle( const  Segment3<ScalarType> & seg, 
								   const Point3<ScalarType> & vert0, 
									const Point3<ScalarType> & vert1, const
									Point3<ScalarType> & vert2,
									ScalarType & a ,ScalarType & b, ScalarType & dist)
{
	//control intersection of bounding boxes
	 Box3<ScalarType> bb0,bb1;
	bb0.Add(seg.P0());
	bb0.Add(seg.P1());
	bb1.Add(vert0);
	bb1.Add(vert1);
	bb1.Add(vert2);
	Point3<ScalarType> inter;
	if (!bb0.Collide(bb1))
		return false;
	if (! Intersection_Segment_Box(bb1,seg,inter))
		return false;

	//first set both directions of ray
	 Ray3<ScalarType> ray;
	 Point3<ScalarType> dir;
	dir=(seg.P1()-seg.P0());
	dir.Normalize();
	ray.Set(seg.P0(),dir);

	//then control for each direction the intersection with triangle
	if ((Intersection<ScalarType>(ray,vert0,vert1,vert2,a,b,dist))
		||(Intersection<ScalarType>(ray,vert1,vert0,vert2,b,a,dist)))
		return (dist<(seg.P1()-seg.P0()).Norm());
	else
		return(false);
}

template <class ScalarType>
bool Intersection_Plane_Box(const  Plane3<ScalarType> &pl,
							 Box3<ScalarType> &bbox)
{
	
	typedef typename  Segment3<ScalarType> SegmentType;
	typedef typename  Point3<ScalarType> CoordType;
	SegmentType diag[4];
	
	CoordType intersection;
	//find the 4 diagonals
	diag[0]=SegmentType(bbox.P(0),bbox.P(7));
	diag[1]=SegmentType(bbox.P(1),bbox.P(6));
	diag[2]=SegmentType(bbox.P(2),bbox.P(5));
	diag[3]=SegmentType(bbox.P(3),bbox.P(4));
	ScalarType a,b,dist;
	for (int i=0;i<3;i++)
		//call intersection of segment and plane
		if ( Intersection<SegmentType>(pl,diag[i],intersection)) 
			return true;
	return false;
}

///if exists return the center and ardius of circle
///that is the intersectionk between the sphere and 
//the plane
template <class ScalarType>
bool Intersection_Plane_Sphere(const Plane3<ScalarType> &pl,
							   const Sphere3<ScalarType> &sphere,
							   Point3<ScalarType> &center,
							   ScalarType &radius)
{
	/* ///set the origin on the center of the sphere
	 Plane3<ScalarType> pl1;
	 Point3<ScalarType> p_plane=pl.Direction()*pl.Offset();
	 Point3<ScalarType> p_plane=p_plane-sphere.Center();

	///set again the plane
	pl1.Set(p_plane,pl.Direction());

	///test d must be positive
	d=pl1.Offset();
	 Point3<ScalarType> n=pl1.Direction();
	///invert d if is <0
	if (d<0)
	{
		n=-n;
		d=-d;
	}
	///no intersection occour
	if (d>r)
		return false;
	else
	{
		///calculate center and translate in back
		center=n*d;
		center+=sphere.Center();
		radius=math::Sqrt((r*r)-(d*d));
	}
	 */
	 assert(0);
	 return true;
}


template<class ScalarType>
bool Intersection_Triangle_Box(const  Box3<ScalarType>   &bbox,
							   const  Point3<ScalarType> &p0,
							   const  Point3<ScalarType> &p1,
							   const  Point3<ScalarType> &p2)
{
	typedef typename  Point3<ScalarType> CoordType;
	CoordType intersection;

	/// control bounding box collision
	 Box3<ScalarType> test;
	test.SetNull();
	test.Add(p0);
	test.Add(p1);
	test.Add(p2);
	if (!test.Collide(bbox))
		return false;
	/// control if each point is inside the bouding box
	if ((bbox.IsIn(p0))||(bbox.IsIn(p1))||(bbox.IsIn(p2)))
		return true;

	/////control plane of the triangle with bbox
	// Plane3<ScalarType> plTri= Plane3<ScalarType>();
	//plTri.Init(p0,p1,p2);
	//if (!Intersection_Plane_Box<ScalarType>(plTri,bbox))
	//	return false;

	///then control intersection of segments with box
	if (Intersection_Segment_Box<ScalarType>(bbox, Segment3<ScalarType>(p0,p1),intersection)||
		Intersection_Segment_Box<ScalarType>(bbox, Segment3<ScalarType>(p1,p2),intersection)||
		Intersection_Segment_Box<ScalarType>(bbox, Segment3<ScalarType>(p2,p0),intersection))
		return true;
	///control intersection of diagonal of the cube with triangle

	Segment3<ScalarType> diag[4];

	diag[0]=Segment3<ScalarType>(bbox.P(0),bbox.P(7));
	diag[1]=Segment3<ScalarType>(bbox.P(1),bbox.P(6));
	diag[2]=Segment3<ScalarType>(bbox.P(2),bbox.P(5));
	diag[3]=Segment3<ScalarType>(bbox.P(3),bbox.P(4));
	ScalarType a,b,dist;
	for (int i=0;i<3;i++)
		if (Intersection_Segment_Triangle<ScalarType>(diag[i],p0,p1,p2,a,b,dist))
			return true;

	return false;
}

template <class SphereType>
bool Intersection_Sphere_Sphere( const SphereType & s0,const SphereType & s1){
	return (s0.Center()-s1.Center()).SquaredNorm() < (s0.Radius()+s1.Radius())*(s0.Radius()+s1.Radius());
}

template<class T>
bool IntersectionPlanePlane (const Plane3<T> & plane0, const Plane3<T> & plane1,
                             Line3<T> & line)
{
    // If Cross(N0,N1) is zero, then either planes are parallel and separated
    // or the same plane.  In both cases, 'false' is returned.  Otherwise,
    // the intersection line is
    //
    //   L(t) = t*Cross(N0,N1) + c0*N0 + c1*N1
    //
    // for some coefficients c0 and c1 and for t any real number (the line
    // parameter).  Taking dot products with the normals,
    //
    //   d0 = Dot(N0,L) = c0*Dot(N0,N0) + c1*Dot(N0,N1)
    //   d1 = Dot(N1,L) = c0*Dot(N0,N1) + c1*Dot(N1,N1)
    //
    // which are two equations in two unknowns.  The solution is
    //
    //   c0 = (Dot(N1,N1)*d0 - Dot(N0,N1)*d1)/det
    //   c1 = (Dot(N0,N0)*d1 - Dot(N0,N1)*d0)/det
    //
    // where det = Dot(N0,N0)*Dot(N1,N1)-Dot(N0,N1)^2.

    T n00 = plane0.Direction()*plane0.Direction();
    T n01 = plane0.Direction()*plane1.Direction();
    T n11 = plane1.Direction()*plane1.Direction();
    T det = n00*n11-n01*n01;

    const T tolerance = (T)(1e-06f);
		if ( math::Abs(det) < tolerance )
        return false;

    T invDet = 1.0f/det;
    T c0 = (n11*plane0.Offset() - n01*plane1.Offset())*invDet;
    T c1 = (n00*plane1.Offset() - n01*plane0.Offset())*invDet;

    line.SetDirection(plane0.Direction()^plane1.Direction());
    line.SetOrigin(plane0.Direction()*c0+ plane1.Direction()*c1);

    return true;
}


// Ray-Triangle Functor
template <bool BACKFACETEST = true>
class RayTriangleIntersectionFunctor {
public:
	template <class TRIANGLETYPE, class SCALARTYPE>
	inline bool operator () (const TRIANGLETYPE & f, const Ray3<SCALARTYPE> & ray, SCALARTYPE & t) {
		typedef SCALARTYPE ScalarType;
		ScalarType a;
		ScalarType b;

		bool bret = Intersection(ray, Point3<SCALARTYPE>::Construct(f.P(0)), Point3<SCALARTYPE>::Construct(f.P(1)), Point3<SCALARTYPE>::Construct(f.P(2)), a, b, t);
		if (BACKFACETEST) {
			if (!bret) {
				bret = Intersection(ray, Point3<SCALARTYPE>::Construct(f.P(0)), Point3<SCALARTYPE>::Construct(f.P(2)), Point3<SCALARTYPE>::Construct(f.P(1)), a, b, t);
			}
		}
		return (bret);
	}
};
}
#endif
