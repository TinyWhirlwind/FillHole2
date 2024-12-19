#ifndef _POINT_
#define _POINT_

#include <assert.h>
#include "../sn3DMath/base.h"
#include "space.h"

namespace ndim{
	
template <int N, class S> 
class Point;
/*��ģ�����ʾ R^N �ռ��еĵ�,ģ���������Ϊ��������*/

template <int N, class S> 
class PointBase 
{
public:
	typedef S          ScalarType;
	typedef VoidType   ParamType;
	typedef Point<N,S> PointType;
	enum {Dimension=N};
protected:
	S _v[N];//Ψһ�����ݣ����û�����

public:
  inline PointBase () { };
	inline S Ext( const int i ) const
	{
		if(i>=0 && i<=N) return _v[i];
		else             return 0;
	}
/*import����*/
	template <int N2, class S2>
	inline void Import( const Point<N2,S2> & b )
	{
		_v[0] = ScalarType(b[0]);
		_v[1] = ScalarType(b[1]);
		if (N>2) { if (N2>2) _v[2] = ScalarType(b[2]); else _v[2] = 0;};
		if (N>3) { if (N2>3) _v[3] = ScalarType(b[3]); else _v[3] = 0;};
	}

/*���������������벻ͬά�ȵĵ�*/
	template <int N2, class S2>
  static inline PointType Construct( const Point<N2,S2> & b )
  {
		PointType p; p.Import(b);
    return p;
  }

/*��ε�import����*/
	template <class S2>
	inline void ImportHomo( const Point<N-1,S2> & b )
	{
		_v[0] = ScalarType(b[0]);
		_v[1] = ScalarType(b[1]);
		if (N>2) { _v[2] = ScalarType(_v[2]); };
		_v[N-1] = 1.0;
	}

/*��ε㹹������*/
	template <int N2, class S2>
  static inline PointType Construct( const Point<N-1,S2> & b )
  {
		PointType p; p.ImportHomo(b);
    return p;
  }

	inline S & operator [] ( const int i )
	{
		assert(i>=0 && i<N);
		return _v[i];
	}
	inline const S & operator [] ( const int i ) const
	{
		assert(i>=0 && i<N);
		return _v[i];
	}
    inline const S &X() const { return _v[0]; } 
	inline const S &Y() const { return _v[1]; }
	inline const S &Z() const { static_assert(N>2); return _v[2]; }
	inline const S &W() const { return _v[N-1]; }//�������󲿷�
	inline S &X() { return _v[0]; }
	inline S &Y() { return _v[1]; }
	inline S &Z() { static_assert(N>2); return _v[2]; }
	inline S &W() { return _v[N-1]; }
	inline const S * V() const
	{
		return _v;
	}
	inline S & V( const int i )
	{
		assert(i>=0 && i<N);
		return _v[i];
	}
	inline const S & V( const int i ) const
	{
		assert(i>=0 && i<N);
		return _v[i];
	}
/*���Բ���*/
	inline void Zero();
	inline PointType operator + ( PointType const & p) const;
	inline PointType operator - ( PointType const & p) const;
	inline PointType operator * ( const S s ) const;
	inline PointType operator / ( const S s ) const;
	inline PointType & operator += ( PointType const & p);
	inline PointType & operator -= ( PointType const & p);
	inline PointType & operator *= ( const S s );
	inline PointType & operator /= ( const S s );
	inline PointType operator - () const;

	inline S operator * ( PointType const & p ) const;

	inline S StableDot ( const PointType & p ) const;

	inline S Norm() const;
		
	template <class PT> static S Norm(const PT &p );
		
	inline S SquaredNorm() const;
		
	template <class PT> static S SquaredNorm(const PT &p );
		
	inline PointType & Normalize();
	
	template <class PT> static PointType & Normalize(const PT &p);
		
	inline PointType & HomoNormalize();//��ι�һ��

	inline S NormInfinity() const;//������겿��

	inline S NormOne() const;
/*������������������㹹�ɵ�ƽ��������*/
	inline S operator % ( PointType const & p ) const;
/*��������ĺ�*/
	inline S Sum() const;
/*����������*/
	inline S Max() const;
/*������С����*/
	inline S Min() const;
/*����������������*/
	inline int MaxI() const;
/*������С����������*/
	inline int MinI() const;
/*����*/
	inline PointType & Scale( const PointType & p );
/*ת��Ϊ������*/
	void ToPolar( S & ro, S & tetha, S & fi ) const
	{
		ro = Norm();
		tetha = (S)atan2( _v[1], _v[0] );
		fi    = (S)acos( _v[2]/ro );
	}


	inline bool operator == ( PointType const & p ) const;
	inline bool operator != ( PointType const & p ) const;
	inline bool operator <  ( PointType const & p ) const;
	inline bool operator >  ( PointType const & p ) const;
	inline bool operator <= ( PointType const & p ) const;
	inline bool operator >= ( PointType const & p ) const;
 /*�������ռ��������ʹ��*/
	inline PointType LocalToGlobal(ParamType p) const{
		return *this; }

	inline ParamType GlobalToLocal(PointType p) const{
		ParamType p(); return p; }

}; 

template <class S> 
class Point2 : public PointBase<2,S> {
public:	
	typedef S ScalarType;
	typedef Point2 PointType;
	using PointBase<2,S>::_v;
	using PointBase<2,S>::V;
	using PointBase<2,S>::W;

	inline Point2 (){}
	inline Point2 ( const S a, const S b){
		_v[0]=a; _v[1]=b;  };

/*��ת90��*/
	inline Point2 operator ~ () const {
		return Point2 ( -_v[2], _v[1] );
	}

/*������X��ļн�*/
	inline ScalarType &Angle(){
		return math::Atan2(_v[1],_v[0]);}

/*ת��Ϊ������*/
	inline Point2 & ToPolar(){
		ScalarType t = Angle();
		_v[0] = Norm();
		_v[1] = t;
		return *this;}

/*ת��Ϊ�Ͽ�������*/
	inline Point2 & ToCartesian() {
		ScalarType l = _v[0];
		_v[0] = (ScalarType)(l*math::Cos(_v[1]));
		_v[1] = (ScalarType)(l*math::Sin(_v[1]));
		return *this;}

/*��ת�㣬��ʱ��*/
	inline Point2 & Rotate( const ScalarType rad ){
		ScalarType t = _v[0];
		ScalarType s = math::Sin(rad);
		ScalarType c = math::Cos(rad);
		_v[0] = _v[0]*c - _v[1]*s;
		_v[1] =   t *s + _v[1]*c;
		return *this;}


	inline void Zero(){
		_v[0]=0; _v[1]=0;  };

	inline Point2 ( const S nv[2] ){
		_v[0]=nv[0]; _v[1]=nv[1];   };

	inline Point2 operator + ( Point2 const & p) const { 
		return Point2( _v[0]+p._v[0], _v[1]+p._v[1]); }

	inline Point2 operator - ( Point2 const & p) const { 
		return Point2( _v[0]-p._v[0], _v[1]-p._v[1]); }

	inline Point2 operator * ( const S s ) const { 
		return Point2( _v[0]*s, _v[1]*s  ); }

	inline Point2 operator / ( const S s ) const {
		S t=1.0/s;
		return Point2( _v[0]*t, _v[1]*t   ); }

	inline Point2 operator - () const {
		return Point2 ( -_v[0], -_v[1]  ); }

	inline Point2 & operator += ( Point2 const & p ) {
		_v[0] += p._v[0]; _v[1] += p._v[1];   return *this; }

	inline Point2 & operator -= ( Point2 const & p ) {
		_v[0] -= p._v[0]; _v[1] -= p._v[1];   return *this; }

	inline Point2 & operator *= ( const S s ) {
		_v[0] *= s; _v[1] *= s;  return *this; }

	inline Point2 & operator /= ( const S s ) {
		S t=1.0/s; _v[0] *= t; _v[1] *= t;   return *this; }

	inline S Norm() const {
		return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1]   );}

	template <class PT> static S Norm(const PT &p ) {
		return math::Sqrt( p.V(0)*p.V(0) + p.V(1)*p.V(1)  );}

	inline S SquaredNorm() const {
		return ( _v[0]*_v[0] + _v[1]*_v[1] );}

	template <class PT> static S SquaredNorm(const PT &p ) {
		return ( p.V(0)*p.V(0) + p.V(1)*p.V(1) );}

	inline S operator * ( Point2 const & p ) const {
    return ( _v[0]*p._v[0] + _v[1]*p._v[1]) ; }

	inline bool operator == ( Point2 const & p ) const {
		return _v[0]==p._v[0] && _v[1]==p._v[1] ;}

	inline bool operator != ( Point2 const & p ) const {
		return _v[0]!=p._v[0] || _v[1]!=p._v[1] ;}

	inline bool operator <  ( Point2 const & p ) const{
		return (_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<p._v[0]); }

	inline bool operator >  ( Point2 const & p ) const	{
		return (_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>p._v[0]); }

	inline bool operator <= ( Point2 const & p ) {
		return (_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<=p._v[0]); }

	inline bool operator >= ( Point2 const & p ) const {
		return (_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>=p._v[0]); }

	inline Point2 & Normalize() {
		PointType n = Norm(); if(n!=0.0) { n=1.0/n;	_v[0]*=n;	_v[1]*=n;} return *this;};

	template <class PT>  Point2 & Normalize(const PT &p){		
		PointType n = Norm(); if(n!=0.0) { n=1.0/n;	V(0)*=n;	V(1)*=n; }
		return *this;};

	inline Point2 & HomoNormalize(){
		if (_v[2]!=0.0) {	_v[0] /= W(); W()=1.0; } return *this;};

	inline S NormInfinity() const {
		return math::Max( math::Abs(_v[0]), math::Abs(_v[1]) ); }  

	inline S NormOne() const {
		return math::Abs(_v[0])+ math::Abs(_v[1]);}  

	inline S operator % ( Point2 const & p ) const {
		return _v[0] * p._v[1] - _v[1] * p._v[0]; }

	inline S Sum() const {
		return _v[0]+_v[1];}

	inline S Max() const {
		return math::Max( _v[0], _v[1] ); }  

	inline S Min() const {
		return math::Min( _v[0], _v[1] ); }  

	inline int MaxI() const {
		return (_v[0] < _v[1]) ? 1:0; };

	inline int MinI() const {
	  return (_v[0] > _v[1]) ? 1:0; };

	inline PointType & Scale( const PointType & p ) {
		_v[0] *= p._v[0]; _v[1] *= p._v[1];  return *this; }

	inline S StableDot ( const PointType & p ) const {
		return _v[0]*p._v[0] +_v[1]*p._v[1]; }
};

template <typename S> 
class Point3 : public PointBase<3,S> {
public:	
	typedef S ScalarType;
	typedef Point3<S>  PointType;
	using PointBase<3,S>::_v;
	using PointBase<3,S>::V;
	using PointBase<3,S>::W;

	inline Point3 ():PointBase<3,S>(){}
	inline Point3 ( const S a, const S b,  const S c){
		_v[0]=a; _v[1]=b; _v[2]=c; };

/*������*/
	inline PointType operator ^ ( PointType const & p ) const {
		return Point3 (
			_v[1]*p._v[2] - _v[2]*p._v[1],
			_v[2]*p._v[0] - _v[0]*p._v[2],
			_v[0]*p._v[1] - _v[1]*p._v[0] );
	}
	
	inline void Zero(){
		_v[0]=0; _v[1]=0; _v[2]=0; };

	inline Point3 ( const S nv[3] ){
		_v[0]=nv[0]; _v[1]=nv[1]; _v[2]=nv[2];  };

	inline Point3 operator + ( Point3 const & p)  const{ 
		return Point3( _v[0]+p._v[0], _v[1]+p._v[1], _v[2]+p._v[2]); }

	inline Point3 operator - ( Point3 const & p) const { 
		return Point3( _v[0]-p._v[0], _v[1]-p._v[1], _v[2]-p._v[2]); }

	inline Point3 operator * ( const S s ) const { 
		return Point3( _v[0]*s, _v[1]*s , _v[2]*s  ); }

	inline Point3 operator / ( const S s ) const {
		S t=1.0/s;
		return Point3( _v[0]*t, _v[1]*t , _v[2]*t  ); }

	inline Point3 operator - () const {
		return Point3 ( -_v[0], -_v[1] , -_v[2]  ); }

	inline Point3 & operator += ( Point3 const & p ) {
		_v[0] += p._v[0]; _v[1] += p._v[1]; _v[2] += p._v[2];  return *this; }

	inline Point3 & operator -= ( Point3 const & p ) {
		_v[0] -= p._v[0]; _v[1] -= p._v[1]; _v[2] -= p._v[2];  return *this; }

	inline Point3 & operator *= ( const S s ) {
		_v[0] *= s; _v[1] *= s; _v[2] *= s; return *this; }

	inline Point3 & operator /= ( const S s ) {
		S t=1.0/s; _v[0] *= t; _v[1] *= t;  _v[2] *= t;   return *this; }

	inline S Norm() const {
		return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2]  );}

	template <class PT> static S Norm(const PT &p ) {
		return math::Sqrt( p.V(0)*p.V(0) + p.V(1)*p.V(1) + p.V(2)*p.V(2)  );}

	inline S SquaredNorm() const {
		return ( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2]  );}

	template <class PT> static S SquaredNorm(const PT &p ) {
		return ( p.V(0)*p.V(0) + p.V(1)*p.V(1) + p.V(2)*p.V(2)  );}

	inline S operator * ( PointType const & p ) const {
    return ( _v[0]*p._v[0] + _v[1]*p._v[1] + _v[2]*p._v[2]) ; }

	inline bool operator == ( PointType const & p ) const {
		return _v[0]==p._v[0] && _v[1]==p._v[1] && _v[2]==p._v[2] ;}

	inline bool operator != ( PointType const & p ) const {
		return _v[0]!=p._v[0] || _v[1]!=p._v[1] || _v[2]!=p._v[2] ;}

	inline bool operator <  ( PointType const & p ) const{
		return	(_v[2]!=p._v[2])?(_v[2]< p._v[2]):
				    (_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<p._v[0]); }

	inline bool operator >  ( PointType const & p ) const	{
		return	(_v[2]!=p._v[2])?(_v[2]> p._v[2]):
				    (_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>p._v[0]); }

	inline bool operator <= ( PointType const & p ) {
		return	(_v[2]!=p._v[2])?(_v[2]< p._v[2]):
				    (_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<=p._v[0]); }

	inline bool operator >= ( PointType const & p ) const {
		return	(_v[2]!=p._v[2])?(_v[2]> p._v[2]):
				    (_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>=p._v[0]); }

	inline PointType & Normalize() {
		S n = Norm(); 
		if(n!=0.0) {
			n=S(1.0)/n;
		_v[0]*=n;	_v[1]*=n;	_v[2]*=n; }
		return *this;};

	template <class PT>  PointType & Normalize(const PT &p){		
		S n = Norm(); if(n!=0.0) { n=1.0/n;	V(0)*=n;	V(1)*=n;	V(2)*=n;  }
		return *this;};

	inline PointType & HomoNormalize(){
		if (_v[2]!=0.0) {	_v[0] /= W();	_v[1] /= W(); W()=1.0; }
		return *this;};

	inline S NormInfinity() const {
		return math::Max( math::Max( math::Abs(_v[0]), math::Abs(_v[1]) ),
			                math::Abs(_v[3])  ); }  

	inline S NormOne() const {
		return math::Abs(_v[0])+ math::Abs(_v[1])+math::Max(math::Abs(_v[2]));}  

	inline S operator % ( PointType const & p ) const {
		S t = (*this)*p; /* Area, general formula */
		return math::Sqrt( SquaredNorm() * p.SquaredNorm() - (t*t) );};

	inline S Sum() const {
		return _v[0]+_v[1]+_v[2];}

	inline S Max() const {
		return math::Max( math::Max( _v[0], _v[1] ),  _v[2] ); }  

	inline S Min() const {
		return math::Min( math::Min( _v[0], _v[1] ),  _v[2] ); }  

	inline int MaxI() const {
		int i= (_v[0] < _v[1]) ? 1:0; if (_v[i] < _v[2]) i=2; return i;};

	inline int MinI() const {
		int i= (_v[0] > _v[1]) ? 1:0; if (_v[i] > _v[2]) i=2; return i;};

	inline PointType & Scale( const PointType & p ) {
		_v[0] *= p._v[0]; _v[1] *= p._v[1]; _v[2] *= p._v[2]; return *this; }

	inline S StableDot ( const PointType & p ) const {
		S k0=_v[0]*p._v[0],	k1=_v[1]*p._v[1], k2=_v[2]*p._v[2];
		int exp0,exp1,exp2;
		frexp( double(k0), &exp0 );
		frexp( double(k1), &exp1 );
		frexp( double(k2), &exp2 );
		if( exp0<exp1 ) 
			if(exp0<exp2) return (k1+k2)+k0; else return (k0+k1)+k2;
		else 
			if(exp1<exp2) return (k0+k2)+k1; else return (k0+k1)+k2;
	}


};



template <typename S> 
class Point4 : public PointBase<4,S> {
public:
	typedef S ScalarType;
	typedef Point4<S> PointType;
	using PointBase<3,S>::_v;
	using PointBase<3,S>::V;
	using PointBase<3,S>::W;

	inline Point4 (){}
	 
	inline Point4 ( const S a, const S b,  const S c, const S d){
		_v[0]=a; _v[1]=b; _v[2]=c; _v[3]=d; };


	inline void Zero(){
		_v[0]=0; _v[1]=0; _v[2]=0; _v[3]=0; };

	inline Point4 ( const S nv[4] ){
		_v[0]=nv[0]; _v[1]=nv[1]; _v[2]=nv[2]; _v[3]=nv[3]; };

	inline Point4 operator + ( Point4 const & p) const { 
		return Point4( _v[0]+p._v[0], _v[1]+p._v[1], _v[2]+p._v[2], _v[3]+p._v[3] ); }

	inline Point4 operator - ( Point4 const & p) const { 
		return Point4( _v[0]-p._v[0], _v[1]-p._v[1], _v[2]-p._v[2], _v[3]-p._v[3] ); }

	inline Point4 operator * ( const S s ) const { 
		return Point4( _v[0]*s, _v[1]*s , _v[2]*s , _v[3]*s ); }

	inline PointType operator ^ ( PointType const & p ) const {
		assert(0);
		return *this;
	}

	inline Point4 operator / ( const S s ) const {
		S t=1.0/s;
		return Point4( _v[0]*t, _v[1]*t , _v[2]*t , _v[3]*t ); }

	inline Point4 operator - () const {
		return Point4 ( -_v[0], -_v[1] , -_v[2] , -_v[3] ); }

	inline Point4 & operator += ( Point4 const & p ) {
		_v[0] += p._v[0]; _v[1] += p._v[1]; _v[2] += p._v[2]; _v[3] += p._v[3]; return *this; }

	inline Point4 & operator -= ( Point4 const & p ) {
		_v[0] -= p._v[0]; _v[1] -= p._v[1]; _v[2] -= p._v[2]; _v[3] -= p._v[3]; return *this; }

	inline Point4 & operator *= ( const S s ) {
		_v[0] *= s; _v[1] *= s; _v[2] *= s; _v[3] *= s; return *this; }

	inline Point4 & operator /= ( const S s ) {
		S t=1.0/s; _v[0] *= t; _v[1] *= t;  _v[2] *= t;  _v[3] *= t; return *this; }

	inline S Norm() const {
		return math::Sqrt( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3] );}

	template <class PT> static S Norm(const PT &p ) {
		return math::Sqrt( p.V(0)*p.V(0) + p.V(1)*p.V(1) + p.V(2)*p.V(2) + p.V(3)*p.V(3) );}

	inline S SquaredNorm() const {
		return ( _v[0]*_v[0] + _v[1]*_v[1] + _v[2]*_v[2] + _v[3]*_v[3] );}

	template <class PT> static S SquaredNorm(const PT &p ) {
		return ( p.V(0)*p.V(0) + p.V(1)*p.V(1) + p.V(2)*p.V(2) + p.V(3)*p.V(3) );}

	inline S operator * ( PointType const & p ) const {
    return ( _v[0]*p._v[0] + _v[1]*p._v[1] + _v[2]*p._v[2] + _v[3]*p._v[3] ); }

	inline bool operator == ( PointType const & p ) const {
		return _v[0]==p._v[0] && _v[1]==p._v[1] && _v[2]==p._v[2] && _v[3]==p._v[3];}

	inline bool operator != ( PointType const & p ) const {
		return _v[0]!=p._v[0] || _v[1]!=p._v[1] || _v[2]!=p._v[2] || _v[3]!=p._v[3];}

	inline bool operator <  ( PointType const & p ) const{
		return	(_v[3]!=p._v[3])?(_v[3]< p._v[3]) : (_v[2]!=p._v[2])?(_v[2]< p._v[2]):
				    (_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<p._v[0]); }

	inline bool operator >  ( PointType const & p ) const	{
		return	(_v[3]!=p._v[3])?(_v[3]> p._v[3]) : (_v[2]!=p._v[2])?(_v[2]> p._v[2]):
				    (_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>p._v[0]); }

	inline bool operator <= ( PointType const & p ) {
		return	(_v[3]!=p._v[3])?(_v[3]< p._v[3]) : (_v[2]!=p._v[2])?(_v[2]< p._v[2]):
				    (_v[1]!=p._v[1])?(_v[1]< p._v[1]) : (_v[0]<=p._v[0]); }

	inline bool operator >= ( PointType const & p ) const {
		return	(_v[3]!=p._v[3])?(_v[3]> p._v[3]) : (_v[2]!=p._v[2])?(_v[2]> p._v[2]):
				    (_v[1]!=p._v[1])?(_v[1]> p._v[1]) : (_v[0]>=p._v[0]); }

	inline PointType & Normalize() {
		PointType n = Norm(); if(n!=0.0) { n=1.0/n;	_v[0]*=n;	_v[1]*=n;	_v[2]*=n; _v[3]*=n; }
		return *this;};

	template <class PT>  PointType & Normalize(const PT &p){		
		PointType n = Norm(); if(n!=0.0) { n=1.0/n;	V(0)*=n;	V(1)*=n;	V(2)*=n; V(3)*=n; }
		return *this;};

	inline PointType & HomoNormalize(){
		if (_v[3]!=0.0) {	_v[0] /= W();	_v[1] /= W();	_v[2] /= W(); W()=1.0; }
		return *this;};

	inline S NormInfinity() const {
		return math::Max( math::Max( math::Abs(_v[0]), math::Abs(_v[1]) ),
			                math::Max( math::Abs(_v[2]), math::Abs(_v[3]) ) ); }  

	inline S NormOne() const {
		return math::Abs(_v[0])+ math::Abs(_v[1])+math::Max(math::Abs(_v[2]),math::Abs(_v[3]));}  

	inline S operator % ( PointType const & p ) const {
		S t = (*this)*p; /* Area, general formula */
		return math::Sqrt( SquaredNorm() * p.SquaredNorm() - (t*t) );};

	inline S Sum() const {
		return _v[0]+_v[1]+_v[2]+_v[3];}

	inline S Max() const {
		return math::Max( math::Max( _v[0], _v[1] ), math::Max( _v[2], _v[3] )); }  

	inline S Min() const {
		return math::Min( math::Min( _v[0], _v[1] ), math::Min( _v[2], _v[3] )); }  

	inline int MaxI() const {
		int i= (_v[0] < _v[1]) ? 1:0; if (_v[i] < _v[2]) i=2; if (_v[i] < _v[3]) i=3;
		return i;};

	inline int MinI() const {
		int i= (_v[0] > _v[1]) ? 1:0; if (_v[i] > _v[2]) i=2; if (_v[i] > _v[3]) i=3;
		return i;};

	inline PointType & Scale( const PointType & p ) {
		_v[0] *= p._v[0]; _v[1] *= p._v[1]; _v[2] *= p._v[2]; _v[3] *= p._v[3]; return *this; }

	inline S StableDot ( const PointType & p ) const {
		S k0=_v[0]*p._v[0],	k1=_v[1]*p._v[1], k2=_v[2]*p._v[2], k3=_v[3]*p._v[3];
		int exp0,exp1,exp2,exp3;
		frexp( double(k0), &exp0 );frexp( double(k1), &exp1 );
		frexp( double(k2), &exp2 );frexp( double(k3), &exp3 );
		if (exp0>exp1) { math::Swap(k0,k1); math::Swap(exp0,exp1); }
		if (exp2>exp3) { math::Swap(k2,k3); math::Swap(exp2,exp3); }
		if (exp0>exp2) { math::Swap(k0,k2); math::Swap(exp0,exp2); }
		if (exp1>exp3) { math::Swap(k1,k3); math::Swap(exp1,exp3); }
		if (exp2>exp3) { math::Swap(k2,k3); math::Swap(exp2,exp3); }
		return ( (k0 + k1) + k2 ) +k3; }
	
};






template <class S>
inline S Angle( Point3<S>  const & p1, Point3<S>  const & p2 )
{
	S w = p1.Norm()*p2.Norm();
	if(w==0) return -1;
	S t = (p1*p2)/w;
	if(t>1) t = 1;
	else if(t<-1) t = -1;
    return (S) acos(t);
}

template <class S>
inline S AngleN( Point3<S>  const & p1, Point3<S>  const & p2 )
{
	S w = p1*p2;
	if(w>1) 
		w = 1;
	else if(w<-1) 
		w=-1;
  return (S) acos(w);
}


template <int N,class S>
inline S Norm( Point<N,S> const & p )
{
	return p.Norm();
}

template <int N,class S>
inline S SquaredNorm( Point<N,S> const & p )
{
  return p.SquaredNorm();
}

template <int N,class S>
inline Point<N,S> & Normalize( Point<N,S> & p )
{
  p.Normalize();
  return p;
}

template <int N, class S>
inline S Distance( Point<N,S> const & p1,Point<N,S> const & p2 )
{
    return (p1-p2).Norm();
}

template <int N, class S>
inline S SquaredDistance( Point<N,S> const & p1,Point<N,S> const & p2 )
{
    return (p1-p2).SquaredNorm();
}






typedef Point2<short>  Point2s;
typedef Point2<int>	  Point2i;
typedef Point2<float>  Point2f;
typedef Point2<double> Point2d;
typedef Point2<short>  Vector2s;
typedef Point2<int>	  Vector2i;
typedef Point2<float>  Vector2f;
typedef Point2<double> Vector2d;
							
typedef Point3<short>  Point3s;
typedef Point3<int>	  Point3i;
typedef Point3<float>  Point3f;
typedef Point3<double> Point3d;
typedef Point3<short>  Vector3s;
typedef Point3<int>	  Vector3i;
typedef Point3<float>  Vector3f;
typedef Point3<double> Vector3d;
														
typedef Point4<short>  Point4s;
typedef Point4<int>	  Point4i;
typedef Point4<float>  Point4f;
typedef Point4<double> Point4d;
typedef Point4<short>  Vector4s;
typedef Point4<int>	  Vector4i;
typedef Point4<float>  Vector4f;
typedef Point4<double> Vector4d;

} 
#endif

