/*
 *  $Id: refineCalibration.cc,v 1.4 2007-03-08 05:21:00 ueshiba Exp $
 */
#include <fstream>
#include "TU/Minimize++.h"
#include "TU/BlockMatrix++.h"
#include "MarkerDetector.h"

namespace TU
{
/************************************************************************
*  class ReferencePlane							*
************************************************************************/
class ReferencePlane
{
  public:
    typedef double	ET;
    
    ReferencePlane()	:_d(3), _Rt(3, 3)				{}
    
    void	initialize(const Matrix<ET>& Qt)			;
    Matrix<ET>	Qt()						const	;
    Vector<ET>	operator ()(const PointF& x)			const	;
    Matrix<ET>	jacobian(const PointF& x)			const	;
    void	update(const Vector<ET>& dq)				;
    
  private:
    Vector<ET>	_d;		// location of the reference plane.
    Matrix<ET>	_Rt;		// orientation of the reference plane.
};

void
ReferencePlane::initialize(const Matrix<ET>& Qt)
{
  // Location of the plane.
    _d = Qt[2];
  // Optimal rotation matrix representing the orientation of the plane.
  /*    SVDecomposition<ET>	svd(Qt(0, 0, 2, 3));
    _Rt(0, 0, 2, 3) = svd.Vt().trns() * svd.Ut()(0, 0, 2, 3);
    _Rt[2] = _Rt[0] ^ _Rt[1];*/
    (_Rt[0] = Qt[0]).normalize();
    (_Rt[2] = Qt[0] ^ Qt[1]).normalize();
    _Rt[1] = _Rt[2] ^ _Rt[0];
}

Matrix<double>
ReferencePlane::Qt() const
{
    Matrix<ET>	Qt(3, 3);
    Qt[0] = _Rt[0];
    Qt[1] = _Rt[1];
    Qt[2] = _d;

    return Qt;
}

Vector<double>
ReferencePlane::operator ()(const PointF& x) const
{
    return _d + x[0] * _Rt[0] + x[1] * _Rt[1];
}

Matrix<double>
ReferencePlane::jacobian(const PointF& x) const
{
    Matrix<ET>	J(3, 6);
    J(0, 0, 3, 3) = Matrix<ET>::I(3);
    J(0, 3, 3, 3) = x[0] * _Rt[0].skew() + x[1] * _Rt[1].skew();

    return J;
}

void
ReferencePlane::update(const Vector<ET>& dq)
{
    _d  -= dq(0, 3);
    _Rt *= Matrix<double>::Rt(dq(3, 3));
}

/************************************************************************
*  class CostFunction							*
************************************************************************/
template <class CAMERA>
class CostFunction
{
  public:
    typedef double		ET;
    typedef Array<CAMERA>	ATA;
    typedef ReferencePlane	ATB;
    typedef BlockMatrix<ET>	JT;
    
    CostFunction(const PairListArrayList& data, u_int dofIntrinsic)	;

    u_int		adim()			const	{return _adim;}
    const Array<u_int>&	adims()			const	{return _adims;}
    u_int		bdim()			const	{return 6;}
    
    Vector<ET>	operator ()(const ATA& p, const ATB& q, int j)	const	;
    JT		jacobianA(const ATA& p, const ATB& q, int j)	const	;
    Matrix<ET>	jacobianB(const ATA& p, const ATB& q, int j)	const	;
    void	updateA(ATA& p, const Vector<ET>& dp)		const	;
    void	updateB(ATB& q, const Vector<ET>& dq)		const	;
    
  private:
    u_int	ncameras()			const	{return _adims.dim();}
    const PairListArray&
		pairListArray(int j)		const	;
    
    const PairListArrayList&	_data;
    u_int			_adim;
    Array<u_int>		_adims;
    Array<u_int>		_npoints;
};

template <class CAMERA> 
CostFunction<CAMERA>::CostFunction(const PairListArrayList& data,
				   u_int dofIntrinsic)
    :_data(data), _adim(0),
     _adims(_data.begin() != _data.end() ? _data.begin()->dim() : 0),
     _npoints(_data.size())
{
  // Compute DOF values for each camera.
    _adims[0] = dofIntrinsic;
    _adim += _adims[0];
    for (int i = 1; i < _adims.dim(); ++i)
    {
	_adims[i] = 6 + dofIntrinsic;
	_adim += _adims[i];
    }

  // Compute the total number of points observed by all the cameras
  // for each plane.
    int	j = 0;
    for (PairListArrayList::const_iterator
	     iter = _data.begin(); iter != _data.end(); ++iter)
    {
	_npoints[j] = 0;
	for (int i = 0; i < iter->dim(); ++i)
	    _npoints[j] += (*iter)[i].size();
	++j;
    }
}

template <class CAMERA> Vector<double>
CostFunction<CAMERA>::operator ()(const ATA& p, const ATB& q, int j) const
{
    const PairListArray&	data = pairListArray(j);
    Vector<ET>			val(2 * _npoints[j]);
    for (int k = 0, i = 0; i < ncameras(); ++i)
	for (PairList::const_iterator
		 iter = data[i].begin(); iter != data[i].end(); ++iter)
	{
	    const Point2<ET>&	u = p[i](q(iter->first));
	    val[k++] = u[0] - iter->second[0];
	    val[k++] = u[1] - iter->second[1];
	}

    return val;
}

template <class CAMERA> BlockMatrix<double>
CostFunction<CAMERA>::jacobianA(const ATA& p, const ATB& q, int j) const
{
    const PairListArray&	data = pairListArray(j);
    BlockMatrix<ET>		J(ncameras());
    for (int i = 0; i < ncameras(); ++i)
    {
	J[i].resize(2 * data[i].size(), _adims[i]);
	int	k = 0;
	for (PairList::const_iterator
		 iter = data[i].begin(); iter != data[i].end(); ++iter)
	{
	    const Vector<ET>&	X = q(iter->first);
	    J[i](k, 0, 2, _adims[i]) = (i == 0 ? p[0].jacobianK(X)
					       : p[i].jacobianP(X));
	    k += 2;
	}
    }
    
    return J;
}

template <class CAMERA> Matrix<double>
CostFunction<CAMERA>::jacobianB(const ATA& p, const ATB& q, int j) const
{
    const PairListArray&	data = pairListArray(j);
    Matrix<ET>			K(2 * _npoints[j], 6);
    for (int k = 0, i = 0; i < ncameras(); ++i)
	for (PairList::const_iterator
		 iter = data[i].begin(); iter != data[i].end(); ++iter)
	{
	    const PointF&	x = iter->first;
	    const Vector<ET>&	X = q(x);
	    K(k, 0, 2, 6) = p[i].jacobianX(X) * q.jacobian(x);
	    k += 2;
	}
    
    return K;
}

template <class CAMERA> void
CostFunction<CAMERA>::updateA(ATA& p, const Vector<ET>& dp) const
{
    int	d = 0;
    p[0].updateIntrinsic(dp(d, _adims[0]));
    d += _adims[0];
    for (int i = 1; i < _adims.dim(); ++i)
    {
	p[i].update(dp(d, _adims[i]));
	d += _adims[i];
    }
}

template <class CAMERA> void
CostFunction<CAMERA>::updateB(ATB& q, const Vector<ET>& dq) const
{
    q.update(dq);
}

template <class CAMERA> const PairListArray&
CostFunction<CAMERA>::pairListArray(int j) const
{
    PairListArrayList::const_iterator	iter = _data.begin();
    while (--j >= 0)
	++iter;
    return *iter;
}

/************************************************************************
*  global functions							*
************************************************************************/
template <class CAMERA> Matrix<double>
refineCalibration(const PairListArrayList& data,
		  Array<CAMERA>& cameras,
		  Matrix<double>& Qt)
{
    using namespace	std;
#ifdef DEBUG
    cerr << "*** Begin: TU::refineCalibration() ***" << endl;
#endif
    CostFunction<CAMERA>	err(data, cameras[0].dofIntrinsic());
    NullConstraint<double>	g;
    Array<ReferencePlane>	planes(data.size());
    for (int j = 0; j < planes.dim(); ++j)
	planes[j].initialize(Qt(3*j, 0, 3, 3));
    Matrix<double>	S = minimizeSquareSparse(err, g, cameras, planes,
						 100, 1.0e-10);
    for (int j = 0; j < planes.dim(); ++j)
	Qt(3*j, 0, 3, 3) = planes[j].Qt();
#ifdef DEBUG
    cerr << "*** End:   TU::refineCalibration() ***\n" << endl;
#endif
    return S;
}

}

namespace TU
{
template Matrix<double>
refineCalibration<CanonicalCamera>(const PairListArrayList&,
				   Array<CanonicalCamera>&,
				   Matrix<double>&);
template Matrix<double>
refineCalibration<Camera>(const PairListArrayList& data,
			  Array<Camera>& cameras,
			  Matrix<double>& Qt);
template Matrix<double>
refineCalibration<CameraWithDistortion>(const PairListArrayList&,
					Array<CameraWithDistortion>&,
					Matrix<double>&);
}
