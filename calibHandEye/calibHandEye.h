/*
 *  平成14-19年（独）産業技術総合研究所 著作権所有
 *  
 *  創作者：植芝俊夫
 *
 *  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
 *  （独）産業技術総合研究所が著作権を所有する秘密情報です．創作者によ
 *  る許可なしに本プログラムを使用，複製，改変，第三者へ開示する等の著
 *  作権を侵害する行為を禁止します．
 *  
 *  このプログラムによって生じるいかなる損害に対しても，著作権所有者お
 *  よび創作者は責任を負いません。
 *
 *  Copyright 2002-2007.
 *  National Institute of Advanced Industrial Science and Technology (AIST)
 *
 *  Creator: Toshio UESHIBA
 *
 *  [AIST Confidential and all rights reserved.]
 *  This program is confidential. Any using, copying, changing or
 *  giving any information concerning with this program to others
 *  without permission by the creator are strictly prohibited.
 *
 *  [No Warranty.]
 *  The copyright holders or the creator are not responsible for any
 *  damages in the use of this program.
 *  
 *  $Id: calibHandEye.h,v 1.6 2012-09-01 07:23:38 ueshiba Exp $
 */
#include <list>
#include <utility>
#include <iterator>
#include "TU/Geometry++.h"
#include "TU/Minimize.h"
#include <boost/foreach.hpp>

namespace TU
{
/************************************************************************
*  type definitions							*
************************************************************************/
typedef Array<Point2f>				Point2Array;
struct Corres : public std::pair<Point3d, Point2Array>
{
    Corres(u_int d=0)	{second.resize(d);}
};
typedef std::list<Corres>			Correses;

/************************************************************************
*  class TriangulationError						*
************************************************************************/
template <class CamIter, class P2Iter>
class TriangulationError
{
  public:
    typedef typename std::iterator_traits<CamIter>::value_type	camera_type;
    typedef typename camera_type::element_type			element_type;
    typedef typename camera_type::vector_type			vector_type;
    typedef typename camera_type::matrix_type			matrix_type;
    typedef typename camera_type::point3_type			point3_type;
    typedef matrix_type						jacobian_type;
    
    TriangulationError(CamIter camBegin, CamIter camEnd, P2Iter u)	;

    vector_type		operator ()(const point3_type& X)	const	;
    jacobian_type	jacobian(const point3_type& X)		const	;
    static void		update(point3_type& X, const vector_type& dX)	;
    element_type	err(const point3_type& X)		const	;
    std::ostream&	put(const point3_type& X,
			    std::ostream& out)			const	;
    
  private:
    const CamIter	_camBegin, _camEnd;
    const P2Iter	_u;
    const u_int		_nviews;
};

template <class CamIter, class P2Iter> inline
TriangulationError<CamIter, P2Iter>::TriangulationError(CamIter camBegin,
							CamIter camEnd,
							P2Iter  u)
    :_camBegin(camBegin), _camEnd(camEnd), _u(u),
     _nviews(std::distance(_camBegin, _camEnd))
{
}

template <class CamIter, class P2Iter>
typename TriangulationError<CamIter, P2Iter>::vector_type
TriangulationError<CamIter, P2Iter>::operator ()(const point3_type& X) const
{
    vector_type	val(2*_nviews);
    int		n = 0;
    P2Iter	u = _u;
    for (CamIter cam = _camBegin; cam != _camEnd; ++cam)
    {
	val(n, 2) = (*cam)(X) - *u++;
	n += 2;
    }

    return val;
}

template <class CamIter, class P2Iter>
typename TriangulationError<CamIter, P2Iter>::jacobian_type
TriangulationError<CamIter, P2Iter>::jacobian(const point3_type& X) const
{
    jacobian_type	J(2*_nviews, 3);
    int			n = 0;
    for (CamIter cam = _camBegin; cam != _camEnd; ++cam)
    {
	J(n, 0, 2, 3) = cam->Jx(X);
	n += 2;
    }

    return J;
}

template <class CamIter, class P2Iter> inline void
TriangulationError<CamIter, P2Iter>::update(point3_type& X,
					    const vector_type& dX)
{
    X -= dX;
}

template <class CamIter, class P2Iter>
inline typename TriangulationError<CamIter, P2Iter>::element_type
TriangulationError<CamIter, P2Iter>::err(const point3_type& X) const
{
    return std::sqrt((*this)(X).square() / _nviews);
}

template <class CamIter, class P2Iter> std::ostream&
TriangulationError<CamIter, P2Iter>::put(const point3_type& X,
					 std::ostream& out) const
{
    const vector_type&	val = (*this)(X);
    for (u_int i = 0; i < _nviews; ++i)
	out << "\t(" << val[2*i] << ' ' << val[2*i+1] << ')';
    return out << std::endl;
}

/************************************************************************
*  class ReprojectionError						*
************************************************************************/
template <class CamIter, class CorresIter>
class ReprojectionError
{
  public:
    typedef typename std::iterator_traits<CamIter>::value_type	camera_type;
    typedef typename camera_type::element_type			element_type;
    typedef typename camera_type::vector_type			vector_type;
    typedef typename camera_type::matrix_type			matrix_type;
    typedef typename camera_type::matrix44_type			matrix44_type;
    typedef matrix_type						jacobian_type;
    typedef Rigidity<matrix44_type>				rigidity_type;

    ReprojectionError(CamIter camBegin, CamIter camEnd,
		    CorresIter corresBegin, CorresIter corresEnd)	;
    
    vector_type		operator ()(const rigidity_type& D)	const	;
    jacobian_type	jacobian(const rigidity_type& D)	const	;
    static void		update(rigidity_type& D, const vector_type& dD)	;
    element_type	err(const rigidity_type& D)		const	;
    std::ostream&	put(const rigidity_type& D,
			    std::ostream& out)			const	;
    
  private:
    const CamIter	_camBegin, _camEnd;
    const CorresIter	_corresBegin, _corresEnd;
    const u_int		_nviews, _npoints;
};

template <class CamIter, class CorresIter>
ReprojectionError<CamIter, CorresIter>::ReprojectionError(
    CamIter camBegin, CamIter camEnd,
    CorresIter corresBegin, CorresIter corresEnd)
    :_camBegin(camBegin), _camEnd(camEnd),
     _corresBegin(corresBegin), _corresEnd(corresEnd),
     _nviews(std::distance(_camBegin, _camEnd)),
     _npoints(std::distance(_corresBegin, _corresEnd))
{
}

template <class CamIter, class CorresIter>
typename ReprojectionError<CamIter, CorresIter>::vector_type
ReprojectionError<CamIter, CorresIter>::operator ()(
    const rigidity_type& D) const
{
    typedef typename std::iterator_traits<CorresIter>
			::value_type			corres_type;
    
    vector_type	val(2*_nviews*_npoints);
    int		n = 0;
    for (CorresIter corres = _corresBegin; corres != _corresEnd; ++corres)
    {
	const vector_type&	X = D(corres->first);
	typename corres_type::second_type::const_iterator
				u = corres->second.begin();
	for (CamIter cam = _camBegin; cam != _camEnd; ++cam)
	{
	    val(n, 2) = (*cam)(X) - *u++;
	    n += 2;
	}
    }

    return val;
}

template <class CamIter, class CorresIter>
typename ReprojectionError<CamIter, CorresIter>::jacobian_type
ReprojectionError<CamIter, CorresIter>::jacobian(
    const rigidity_type& D) const
{
    jacobian_type	J(2*_nviews*_npoints, D.nparams());
    int			n = 0;
    for (CorresIter corres = _corresBegin; corres != _corresEnd; ++corres)
    {
	const vector_type&	X = D(corres->first);
	for (CamIter cam = _camBegin; cam != _camEnd; ++cam)
	{
	    J(n, 0, 2, J.ncol()) = cam->Jx(X) * D.jacobian(corres->first);
	    n += 2;
	}
    }

    return J;
}

template <class CamIter, class CorresIter> inline void
ReprojectionError<CamIter, CorresIter>::update(rigidity_type& D,
					       const vector_type& dD)
{
    D.update(dD);
}

template <class CamIter, class CorresIter>
inline typename ReprojectionError<CamIter, CorresIter>::element_type
ReprojectionError<CamIter, CorresIter>::err(const rigidity_type& D) const
{
    return sqrt((*this)(D).square() / (_nviews * _npoints));
}

template <class CamIter, class CorresIter> std::ostream&
ReprojectionError<CamIter, CorresIter>::put(const rigidity_type& D,
					    std::ostream& out) const
{
    const vector_type&	val = (*this)(D);
    int			n = 0;
    for (u_int j = 0; j < _npoints; ++j)
    {
	for (u_int i = 0; i < _nviews; ++i)
	{
	    out << "\t(" << val[n] << ' ' << val[n+1] << ')';
	    n += 2;
	}
	out << std::endl;
    }
    return out;
}

/************************************************************************
*  global functions							*
************************************************************************/
template <class S, class T> std::istream&
operator >>(std::istream& in, std::pair<S, T>& pair)
{
    return in >> pair.first >> pair.second;
}

template <class S, class T> std::ostream&
operator <<(std::ostream& out, const std::pair<S, T>& pair)
{
    pair.first.put(out);
    BOOST_FOREACH (typename T::const_reference val, pair.second)
	val.put(out);
    return out << std::endl;
}

//! 複数のカメラ行列と各カメラに観測された2D点から3D点を復元する．
/*!
  \param camBegin	最初のカメラ行列を指す反復子
  \param camEnd		最後の次のカメラ行列を指す反復子
  \param u		最初のカメラに観測された2D点を指す反復子
  \param refine		再投影誤差最小化による非線形最適化の有無
*/
template <class CamIter, class P2Iter> double
triangulate(typename std::iterator_traits<CamIter>
			::value_type::point3_type& X,
	    CamIter camBegin, CamIter camEnd, P2Iter u, bool refine)
{
    using namespace	std;

    typedef typename std::iterator_traits<CamIter>::value_type	camera_type;
    typedef typename camera_type::element_type			element_type;
    typedef typename camera_type::vector_type			vector_type;
    typedef typename camera_type::point3_type			point3_type;
    
    Matrix44d	A;
    P2Iter	uu = u;
    for (CamIter cam = camBegin; cam != camEnd; ++cam)	// for each view...
    {
	Vector4d	a;
	Matrix34d	P = cam->P();
	a = P[0] - (*uu)[0] * P[2];
	A += a % a;
	a = P[1] - (*uu)[1] * P[2];
	A += a % a;
	++uu;
    }
    vector_type		evalue;
    X = A.eigen(evalue)[3].inhomogeneous();
    TriangulationError<CamIter, P2Iter>
	triangulationError(camBegin, camEnd, u);

    if (refine)
    {
	element_type	err = triangulationError.err(X);
	
	minimizeSquare(triangulationError, NullConstraint<element_type>(), X,
		       100, 1.5e-16);
#ifdef TRIANGULATION_DEBUG
	cerr << X;
	cerr << "  triangulation err: " << err
	     << " -> " << triangulationError.err(X)
	     << endl;
	triangulationError.put(X, cerr);
#endif
    }
    else
    {
#ifdef TRIANGULATION_DEBUG
	cerr << X;
	cerr << "  triangulation err: " << triangulationError.err(X) << endl;
	triangulationError.put(X, cerr);
#endif    
    }

    return triangulationError.err(X);
}

//! 3D点とその投影像からワールド座標系からカメラ座標系への剛体変換を求める．
/*!
  複数のカメラと点データを与える．点データは，ワールド座標における
  3D点と各カメラにおける2D投影点の配列からなるペアである．
  \param camBegin	最初のカメラを指す反復子
  \param camEnd		最後の次のカメラを指す反復子
  \param corresBegin	最初の点データを指す反復子
  \param corresEnd	最後の次の点データを指す反復子
  \param refine		再投影誤差最小化による非線形最適化の有無．投影点から
			カメラ座標系における3D点を復元する際に既に再投影語差
			最小化を行っているので，剛体変換時に再度これを行うの
			は弊害が大きい．よって，このパラメータはfalseを指定す
			ることが望ましい．
*/
template <class CamIter, class CorresIter>
Rigidity<typename std::iterator_traits<CamIter>::value_type::matrix44_type>
calibHandEye(CamIter camBegin, CamIter camEnd,
	     CorresIter corresBegin, CorresIter corresEnd, bool refine)
{
    using namespace	std;
    
    typedef typename std::iterator_traits<CamIter>::value_type	camera_type;
    typedef typename camera_type::element_type			element_type;
    typedef typename camera_type::point3_type			point3_type;
    typedef std::pair<point3_type, point3_type>			corres_type;
    typedef Rigidity<typename camera_type::matrix44_type>	rigidity_type;
    
  // カメラ座標系における各校正点の位置をtriangulationによって求める．
    list<corres_type>	correses;
    for (CorresIter corres = corresBegin; corres != corresEnd; ++corres)
    {
	point3_type	X;
	if (triangulate(X,
			camBegin, camEnd, corres->second.begin(), true) < 1.0)
	    correses.push_back(make_pair(corres->first, X));
    }

  // ワールド座標系からカメラ座標系への剛体変換の初期値を求める．
    rigidity_type	D(correses.begin(), correses.end());
    ReprojectionError<CamIter, CorresIter>
	reprojectionError(camBegin, camEnd, corresBegin, corresEnd);

    cerr << "Initial 3D err: " << D.rmsError(correses.begin(), correses.end())
	 << endl;
#ifdef DEBUG
    rigidity_type	Dinv = D.inv();
    BOOST_FOREACH (const corres_type& corres, correses)
    {
	point3_type	p = Dinv(corres.second);  // カメラ座標系→ワールド座標系
	cerr << '\t';
	p.put(cerr) << "\t(";
	(corres.first - p).put(cerr) << ')' << endl;
    }
#endif
    cerr << "Initial 2D err: " << reprojectionError.err(D) << endl;
#ifdef DEBUG
    reprojectionError.put(D, cerr) << endl;
#endif
    
  // 再投影誤差最小化により剛体変換を最適化する．
    if (refine)
    {
	minimizeSquare(reprojectionError, NullConstraint<element_type>(), D);

	cerr << "Refined 3D err: "
	     << D.rmsError(correses.begin(), correses.end())
	     << endl;
#ifdef DEBUG
	rigidity_type	Dinv = D.inv();
	BOOST_FOREACH (const corres_type& corres, correses)
	{
	    point3_type	p = Dinv(corres.second);
	    cerr << '\t';
	    p.put(cerr) << "\t(";
	    (corres.first - p).put(cerr) << ')' << endl;
	}
#endif
	cerr << "Refined 2D err: " << reprojectionError.err(D) << endl;
#ifdef DEBUG
	reprojectionError.put(D, cerr) << endl;
#endif
    }

    return D;
}

}
