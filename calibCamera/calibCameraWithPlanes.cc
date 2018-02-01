/*
 *  $Id: calibCameraWithPlanes.cc,v 1.1.1.1 2007-05-23 05:55:43 ueshiba Exp $
 */
#include <vector>
#include "TU/utility.h"
#include "TU/Minimize++.h"
#include "MarkerDetector.h"

namespace TU
{
/************************************************************************
*  static functions							*
************************************************************************/
static Vector<double>
extpro(const Vector<double>& a, const Vector<double>& b)
{
    const u_int		d = a.dim();
    Vector<double>	v(d * (d+1) / 2);
    int			n = 0;
    for (int i = 0; i < d; ++i)
    {
	v[n++] = a[i]*b[i];
	for (int j = i+1; j < d; ++j)
	    v[n++] = a[i]*b[j] + a[j]*b[i];
    }
    return v;
}

/************************************************************************
*  class CostFunction							*
************************************************************************/
class CostFunction	// cost function for refining calibration
{			//   using multiple planar patterns.
  public:
    typedef double			ET;
    typedef CameraBase::Intrinsic	ATA;
    typedef CanonicalCamera		ATB;
    typedef Matrix<ET>			JT;

  public:
    CostFunction(const PairListList& data, u_int dofIntrinsic)		;

    Vector<ET>	operator ()(const ATA& intrinsic,
			    const ATB& camera, int i)		const	;
    JT		jacobianA(const ATA& intrinsic,
			  const ATB& camera, int i)		const	;
    Matrix<ET>	jacobianB(const ATA& intrinsic,
			  const ATB& camera, int i)		const	;
    void	updateA(ATA& intrinsic,
			const Vector<ET>& dintrinsic)		const	;
    void	updateB(ATB& camera, const Vector<ET>& dcamera)	const	;

    u_int	adim()					const	{return _adim;}
    u_int	adims()					const	{return adim();}
    
  private:
    const PairList&	pairList(int i)				const	;
    
    const PairListList&	_data;
    const u_int		_adim;
    Array<u_int>	_npoints;
};

CostFunction::CostFunction(const PairListList& data, u_int dofIntrinsic)
    :_data(data), _adim(dofIntrinsic), _npoints(_data.size())
{
    int	i = 0;
    for (PairListList::const_iterator
	     iter = _data.begin(); iter != _data.end(); ++iter)
	_npoints[i++] = iter->size();
}
	
Vector<double>
CostFunction::operator ()(const ATA& intrinsic, const ATB& camera, int i) const
{
    const PairList&	data = pairList(i);
    Vector<ET>		val(2 * _npoints[i]);
    int			k = 0;
    for (PairList::const_iterator
	     iter = data.begin(); iter != data.end(); ++iter)
    {
	Vector<ET>		X(3);
	X(0, 2) = iter->first;		// 3D reference points (X[2] = 0.0)
      	const Point2<ET>&	u = intrinsic(camera.xc(X));
	val[k++] = u[0] - iter->second[0];
	val[k++] = u[1] - iter->second[1];
    }

    return val;
}

Matrix<double>
CostFunction::jacobianA(const ATA& intrinsic, const ATB& camera, int i) const
{
    const PairList&	data = pairList(i);
    Matrix<ET>		J(2 * _npoints[i], adim());
    int			k = 0;
    for (PairList::const_iterator
	     iter = data.begin(); iter != data.end(); ++iter)
    {
	Vector<ET>	X(3);
	X(0, 2) = iter->first;		// 3D reference points (X[2] = 0.0)
	J(k, 0, 2, J.ncol()) = intrinsic.jacobianK(camera.xc(X));

	k += 2;
    }
    
    return J;
}

Matrix<double>
CostFunction::jacobianB(const ATA& intrinsic, const ATB& camera, int i) const
{
    const PairList&	data = pairList(i);
    Matrix<ET>		K(2 * _npoints[i], 6);
    int			k = 0;
    for (PairList::const_iterator
	     iter = data.begin(); iter != data.end(); ++iter)
    {
	Vector<ET>	X(3);
	X(0, 2) = iter->first;		// 3D reference points (X[2] = 0.0)
	K(k, 0, 2, K.ncol())
	    = intrinsic.jacobianXC(camera.xc(X)) * camera.jacobianPc(X);

	k += 2;
    }
    
    return K;
}

void
CostFunction::updateA(ATA& intrinsic, const Vector<ET>& dintrinsic) const
{
    intrinsic.update(dintrinsic);
}

void
CostFunction::updateB(ATB& camera, const Vector<ET>& dcamera) const
{
    camera.update(dcamera);
}
    
const PairList&
CostFunction::pairList(int i) const
{
    PairListList::const_iterator	iter = _data.begin();
    while (--i >= 0)
	++iter;
    return *iter;
}
    
/************************************************************************
*  global functions							*
************************************************************************/
template <class INTRINSIC> INTRINSIC
calibCameraWithPlanes(const PairListList& data, bool doRefinement)
{
    using namespace	std;

    const u_int	nplanes = data.size();
    if (nplanes < 2)
	throw runtime_error("Three or more planes needed!!");
#ifdef DEBUG
    cerr << "*** Begin: TU::calibCameraWithPlanes() ***" << endl;
#endif
  // Compute homography matrices for each plane.
    Array<Matrix<double> >	Ht(nplanes);
    int				i = 0;
    for (PairListList::const_iterator
	     iter = data.begin(); iter != data.end(); ++iter)
    {
	ProjectiveMapping	H(iter->begin(), iter->end(), true);
	Ht[i++] = H.T().trns();
    }
#ifdef DEBUG
    cerr << "--- Homography matrices computed..." << endl;
#endif
    
  // Compute IAC(Image of Absolute Conic).
    Matrix<double>	A(2*Ht.dim(), 6);
    for (int i = 0; i < Ht.dim(); ++i)
    {
	A[2*i  ] = extpro(Ht[i][0], Ht[i][0]) - extpro(Ht[i][1], Ht[i][1]);
	A[2*i+1] = extpro(Ht[i][0], Ht[i][1]);
    }
    Vector<double>		evalue;
    const Matrix<double>&	evector = (A.trns() * A).eigen(evalue);
    Vector<double>		a;
    if (Ht.dim() < 3)			// two reference planes.
	a = evector[5][1] * evector[4] - evector[4][1] * evector[5];
    else
	a = evector[5];
    Matrix<double>	omega(3, 3);
    omega[0][0]		      = a[0];
    omega[0][1] = omega[1][0] = a[1];
    omega[0][2] = omega[2][0] = a[2];
    omega[1][1]		      = a[3];
    omega[1][2] = omega[2][1] = a[4];
    omega[2][2]		      = a[5];
    if (omega.det() < 0.0)
	omega *= -1.0;
#ifdef DEBUG
    cerr << "--- IAC(Image of Absolute Conic) computed..." << endl;
#endif

  // Extract intrinsic parameters from IAG.
    const Matrix<double>	&Kinv = omega.cholesky(), &Ktinv = Kinv.trns();
    Matrix<double>		K(Kinv.inv());
    K /= K[2][2];
    INTRINSIC			intrinsic(K);
#ifdef DEBUG
    cerr << "--- Intrinsic parameters extracted..." << endl;
#endif
    
  // Do refinement if required.
    if (doRefinement)
    {
      	Array<CanonicalCamera>	cameras(Ht.dim());
      	for (int i = 0; i < cameras.dim(); ++i)
	{
	    const Matrix<double>&	tmp = Ht[i] * Ktinv;
	    SVDecomposition<double>	svd(tmp(0, 0, 2, 3));
	    Matrix<double>		R(3, 3);
	    R(0, 0, 2, 3) = svd.Vt().trns() * svd.Ut()(0, 0, 2, 3);
	    R[2] = R[0] ^ R[1];
	
	    const Vector<double>&	t = -2.0/(svd[0] + svd[1])*(R * tmp[2]);

	    cameras[i].setRotation(R.trns()).setTranslation(t);
	}
    
      	CostFunction		err(data, intrinsic.dof());
	NullConstraint<double>	g;

	minimizeSquareSparse(err, g, intrinsic, cameras, 200);
#ifdef DEBUG
	cerr << "--- Intrinsic parameters refined..." << endl;
#endif
    }
#ifdef DEBUG
    cerr << "*** End:   TU::calibCameraWithPlanes() ***\n" << endl;
#endif
    return intrinsic;
}

template Camera::Intrinsic
calibCameraWithPlanes<Camera::Intrinsic>(const PairListList&	 data,
					 bool			 doRefinement);
template CameraWithDistortion::Intrinsic
calibCameraWithPlanes<CameraWithDistortion::Intrinsic>(
					 const PairListList&	 data,
					 bool			 doRefinement);
}
