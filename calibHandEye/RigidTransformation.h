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
 *  $Id: RigidTransformation.h,v 1.4 2012-09-01 07:23:38 ueshiba Exp $
 */
#include <algorithm>
#include "TU/Geometry++.h"

namespace TU
{
/************************************************************************
*  class RigidTransformation						*
************************************************************************/
class RigidTransformation
{
  public:
    typedef double			element_type;
    typedef Vector<element_type>	vector_type;
    typedef Matrix<element_type>	matrix_type;
    
  public:
    RigidTransformation(u_int dim=2)					;
    RigidTransformation(const matrix_type& D)	:_D(D)			{}
    template <class CorresIter>
    RigidTransformation(CorresIter corresBegin, CorresIter corresEnd)	;

    template <class CorresIter>
    void		initialize(CorresIter corresBegin,
				   CorresIter corresEnd)	;
    u_int		size()				  const	{return
								   _D.size()-1;}
    u_int		ndataMin()			  const	{return size();}
    const matrix_type&	D()				  const	{return _D;}
    template <class S, class B>
    vector_type		operator ()(const Vector<S, B>& x)const	;
    template <class S, class B>
    vector_type		mapP(const Vector<S, B>& x)	  const	;
    template <class S, class B>
    vector_type		invmap(const Vector<S, B>& x)	  const	;
    template <class S, class B>
    vector_type		invmapP(const Vector<S, B>& x)	  const	;
    template <class S, class B>
    matrix_type		jacobian(const Vector<S, B>& x)	  const	;
    u_int		dof()				  const	;
    void		update(const vector_type& dD)		;
    template <class CorresIter>
    element_type	err(CorresIter corresBegin,
			    CorresIter corresEnd)	  const	;
    
  private:
    matrix_type		_D;
};

inline
RigidTransformation::RigidTransformation(u_int dim)
    :_D(dim + 1, dim + 1)
{
    for (int i = 0; i < _D.size(); ++i)
	_D[i][i] = 1.0;
}
    
template <class CorresIter> inline
RigidTransformation::RigidTransformation(CorresIter corresBegin,
					 CorresIter corresEnd)
{
    initialize(corresBegin, corresEnd);
}

template <class CorresIter> void
RigidTransformation::initialize(CorresIter corresBegin, CorresIter corresEnd)
{
    if (corresBegin == corresEnd)
	throw std::invalid_argument("RigidTransformation::initialize(): zero-length input data!!");

    const u_int	xdim = corresBegin->first.size();
    if (corresBegin->second.size() != xdim)
	throw std::invalid_argument("RigidTransformation::initialize(): correspondence between two points with different dimensions!!");

    _D.resize(xdim + 1, xdim + 1);
    if (std::distance(corresBegin, corresEnd) < ndataMin())
	throw std::invalid_argument("RigidTransformation::initialize(): not enough input data!!");
    
  // ２つの点群の重心を求める．
    vector_type	xc(size()), yc(size());
    u_int	n = 0;
    for (CorresIter corres = corresBegin; corres != corresEnd; ++corres)
    {
	xc += corres->first;
	yc += corres->second;
	++n;
    }
    xc /= n;
    yc /= n;

  // モーメント行列Aを求める．
    matrix_type	A(size(), size());
    for (CorresIter corres = corresBegin; corres != corresEnd; ++corres)
	A += (corres->first - xc) % (corres->second - yc);

  // 点群間の剛体変換を求める．
    SVDecomposition<element_type>	svd(A);
    _D(0, 0, size(), size()) = svd.Ut().trns() * svd.Vt();
    for (int i = 0; i < size(); ++i)
	_D[i][size()] = yc[i] - _D[i](0, size()) * xc;
    _D[size()][size()] = 1.0;
}
    
template <class S, class B> inline RigidTransformation::vector_type
RigidTransformation::operator ()(const Vector<S, B>& x) const
{
    const vector_type&	yP = mapP(x);
    return yP(0, size()) / yP[size()];
}

template <class S, class B> RigidTransformation::vector_type
RigidTransformation::mapP(const Vector<S, B>& x) const
{
    if (x.size() == size())
    {
	vector_type	xP(size()+1);
	xP(0, size()) = x;
	xP[size()]    = 1.0;
	return _D * xP;
    }
    else
	return _D * x;
}

template <class S, class B> RigidTransformation::vector_type
RigidTransformation::invmap(const Vector<S, B>& x) const
{
    const vector_type&	yP = invmapP(x);
    return yP(0, size()) / yP[size()];
}

template <class S, class B> RigidTransformation::vector_type
RigidTransformation::invmapP(const Vector<S, B>& x) const
{
    vector_type	xP(size()+1);
    if (x.size() == size())
    {
	xP(0, size()) = x;
	xP[size()]    = 1.0;
    }
    else
	xP = x;
    return xP.solve(_D.trns());
}

template <class S, class B> RigidTransformation::matrix_type
RigidTransformation::jacobian(const Vector<S, B>& x) const
{
    vector_type	xx(size());
    if (x.size() == size())
	xx = x;
    else if (x.size() == size() + 1)
	xx = x(0, size()) / x[size()];
    else
	throw std::invalid_argument("RigidTransformation::jacobian(): invalid dimension of the input vector!!");
    
    matrix_type	J(size(), dof());
    switch (size())
    {
      case 2:
	J[0][0] = J[1][1] = 1.0;
	J[0][2] =  _D[1](0, 2) * xx;
	J[1][2] = -_D[0](0, 2) * xx;
	break;
      case 3:
	J[0][0] = J[1][1] = J[2][2] = 1.0;
	J(0, 3, 3, 3) = (_D(0, 0, 3, 3)*xx).skew();
	break;
      default:
	throw std::invalid_argument("RigidTransformation::jacobian(): not implemented for this dimension!!");
	break;
    }

    return J;
}

inline u_int
RigidTransformation::dof() const
{
    return size() * (size() + 1) / 2;
}
    
template <class CorresIter> RigidTransformation::element_type
RigidTransformation::err(CorresIter corresBegin, CorresIter corresEnd) const
{
    element_type	e = 0.0;
    u_int		n = 0;
    for (CorresIter corres = corresBegin; corres != corresEnd; ++corres)
    {
	e += ((*this)(corres->first) - corres->second).square();
	++n;
    }
    
    return sqrt(e / n);
}

}
