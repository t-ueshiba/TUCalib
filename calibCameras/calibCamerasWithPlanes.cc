/*
 *  $Id: calibCamerasWithPlanes.cc,v 1.2 2007-03-08 05:11:09 ueshiba Exp $
 */
#include "TU/utility.h"
#include "MarkerDetector.h"

namespace TU
{
template <class CAMERA> Matrix<double>
refineCalibration(const PairListArrayList&	data,
		  Array<CAMERA>&		cameras,
		  Matrix<double>&		Qt);

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

static double
polynomial3(double a0, double a1, double a2, double a3, double x)
{
    return a0 + x * (a1 + x * (a2 + x * a3));
}

static double
optimalEigenValue(const Matrix<double>& G)
{
    const Vector<double> &a = G[1] ^ G[2], &b = G[2] ^ G[0], &c = G[0] ^ G[1];
    return -(a[1]*G[1][0] + a[2]*G[2][0] + b[0]*G[0][1] + b[2]*G[2][1] +
	     c[0]*G[0][2] + c[1]*G[1][2])
	   /(G[0][1]*G[0][1] + G[0][2]*G[0][2] +
	     G[1][0]*G[1][0] + G[1][2]*G[1][2] +
	     G[2][0]*G[2][0] + G[2][1]*G[2][1]);
}

static Array<Normalize>
computeNormalization(const PairListArrayList& data)
{
    Array<Normalize>	norm(data.begin() != data.end() ?
			     1 + data.begin()->dim() : 0);

    for (PairListArrayList::const_iterator
	     iter = data.begin(); iter != data.end(); ++iter)
	for (int i = 0; i < iter->dim(); ++i)
	{
	    norm[1+i].update(make_const_second_iterator((*iter)[i].begin()),
			     make_const_second_iterator((*iter)[i].end()));
	    norm[0].update(make_const_first_iterator((*iter)[i].begin()),
			   make_const_first_iterator((*iter)[i].end()));
	}

    return norm;
}

static Matrix<double>
computeHomographies(const PairListArrayList& data)
{
    using namespace	std;

    const u_int		nplanes  = data.size(),
			ncameras = (nplanes != 0 ? data.begin()->dim() : 0);
#ifdef DEBUG
    cerr << "*** Begin: TU::computeHomographies() ***\n "
	 << ncameras << " cameras observing " << nplanes << " planes."
	 << data << endl;
#endif
    Matrix<double>	W(3*ncameras, 3*nplanes);
    int			j = 0;
    for (PairListArrayList::const_iterator
	     iter = data.begin(); iter != data.end(); ++iter)
    {
	for (int i = 0; i < ncameras; ++i)
	{
	    ProjectiveMapping	H((*iter)[i].begin(), (*iter)[i].end(), true);
	    
	    W(3*i, 3*j, 3, 3) = H.T();
#ifdef DEBUG
	    cerr << "--- H" << i << j << " ---\n"
		 << W(3*i, 3*j, 3, 3) << endl;
#endif
	}
	++j;
    }
#ifdef DEBUG
    cerr << "*** End:   TU::computeHomographies() ***\n" << endl;
#endif
    return W;
}

static void
rescaleHomographies(Matrix<double>& W)
{
    using namespace		std;
    
#ifdef DEBUG
    cerr << "*** Begin: TU::rescaleHomographies() ***" << endl;
#endif
    const u_int			ncameras = W.nrow()/3, nplanes = W.ncol()/3;
    const Matrix<double>&	H00inv = W(0, 0, 3, 3).inv();
    for (int i = 1; i < ncameras; ++i)		// for each camera...
    {
      // Inter-image homography between camera 0 and i through plane 0.
	const Matrix<double>&	A = W(3*i, 0, 3, 3) * H00inv;

	for (int j = 1; j < nplanes; ++j)	// for each plane...
	{
	    Matrix<double>		Hij = W(3*i, 3*j, 3, 3);
	    const Matrix<double>&	G   = W(0, 3*j, 3, 3) * Hij.inv() * A;
	    const double		mu  = optimalEigenValue(G);
	    W(3*i, 3*j, 3, 3) *= mu;
#ifdef DEBUG
	    const double		trG = G.trace(),
					trAdjG = G.adj().trace();
	    cerr << " H" << i << j
		 << ": a0 = " << polynomial3(-G.det(), trAdjG, -trG, 1, mu)
		 << ", a1 = " << polynomial3(trAdjG, -2*trG, 3, 0, mu)
		 << ", a2 = " << polynomial3(-trG, 3, 0, 0, mu)
		 << ", mu = " << mu
		 << endl;
#endif
	}
    }
#ifdef DEBUG
    cerr << "*** End:   TU::rescaleHomographies() ***\n" << endl;
#endif
}

static void
factorHomographies(const Matrix<double>& W,
		   Matrix<double>& P, Matrix<double>& Qt)
{
    using namespace		std;
    
    SVDecomposition<double>	svd(W);
#ifdef DEBUG
    cerr << "*** Begin: TU::factorHomographies() ***\n"
	 << " singular values: " << svd.diagonal()(0, 5);
#endif
    P.resize(W.nrow(), 4);
    Qt.resize(W.ncol(), 4);
    for (int n = 0; n < 4; ++n)
    {
	for (int i = 0; i < P.nrow(); ++i)
	    P[i][n] = svd.Vt()[n][i];
	for (int j = 0; j < Qt.nrow(); ++j)
	    Qt[j][n] = svd[n] * svd.Ut()[n][j];
    }
#ifdef DEBUG
    cerr << "*** End:   TU::factorHomographies() ***\n" << endl;
#endif
}

static void
projectiveToMetric(Matrix<double>& P, Matrix<double>& Qt)
{
    using namespace	std;
    
#ifdef DEBUG
    cerr << "*** Begin: TU::projectiveToMetric() ***" << endl;
#endif
    const u_int	nplanes = Qt.nrow() / 3;
    
  // Fix WC(World Coordinates) to 0-th camera.
    SVDecomposition<double>	svd(P(0, 0, 3, 4));
    Matrix<double>		Sinv(4, 4);
    Sinv(0, 0, 3, 4) = P(0, 0, 3, 4);
    Sinv[3] = svd.Ut()[3];
#ifdef DEBUG
    cerr << "  det(Sinv) = " << Sinv.det() << endl;
#endif    
    P  *= Sinv.inv();
    Qt *= Sinv.trns();

  // Compute IAC(Image of Absolute Conic) of 0-th camera.
    Matrix<double>	A(2*nplanes, 6);
    for (int j = 0; j < nplanes; ++j)
    {
	const Matrix<double>&	Qt2x3 = Qt(3*j, 0, 2, 3);
	A[2*j  ] = extpro(Qt2x3[0], Qt2x3[0]) - extpro(Qt2x3[1], Qt2x3[1]);
	A[2*j+1] = extpro(Qt2x3[0], Qt2x3[1]);
    }
    Vector<double>		evalue;
    const Matrix<double>&	evector = (A.trns() * A).eigen(evalue);
#ifdef DEBUG
    cerr << "  eigen values = " << evalue;
#endif
    Vector<double>		a;
    if (nplanes < 3)			// two reference planes.
	a = evector[5][1] * evector[4] - evector[4][1] * evector[5];
    else
	a = evector[5];
    Matrix<double>	omega(3, 3);	// IAC
    omega[0][0]		      = a[0];
    omega[0][1] = omega[1][0] = a[1];
    omega[0][2] = omega[2][0] = a[2];
    omega[1][1]		      = a[3];
    omega[1][2] = omega[2][1] = a[4];
    omega[2][2]		      = a[5];
    if (omega.det() < 0.0)
	omega *= -1.0;
    Matrix<double>	K1inv = omega.cholesky();
    K1inv /= K1inv[2][2];
    omega = K1inv.trns() * K1inv;
    
  // Compute PI(Plane at Inifinity).
    Vector<double>	b(3*nplanes), beta(nplanes);
    for (int j = 0; j < nplanes; ++j)
    {
	const Matrix<double>&	Qt2x3 = Qt(3*j, 0, 2, 3);
	beta[j] = sqrt((Qt2x3[0]*omega*Qt2x3[0] +
			Qt2x3[1]*omega*Qt2x3[1])/2.0);
#ifdef DEBUG
	cerr << "  " << j << "-th plane: beta = " << beta[j] << endl;
#endif
      	b[3*j  ] = 0.0;
	b[3*j+1] = 0.0;
	b[3*j+2] = beta[j];
    }
    Vector<double>	h = b * Qt;	// PI(Plane at Infinity)
    h.solve(Qt.trns() * Qt);
    
  // Compute transformation from projective to Euclidean coordinates.
    Matrix<double>	T(4, 4);
    T(0, 0, 3, 3) = K1inv;
    T[3] = h;
    P  *= T.inv();
    Qt *= T.trns();
    for (int j = 0; j < nplanes; ++j)
	Qt(3*j, 0, 3, 4) /= beta[j];
#ifdef DEBUG
    for (int j = 0; j < nplanes; ++j)
    {
	cerr << " === " << j << "-th plane ===\n"
	     << "  Q30 = " << Qt[3*j  ][3] << ", Q31 = " << Qt[3*j+1][3]
	     << ", Q32 = " << Qt[3*j+2][3] << endl;
	const Matrix<double>&	Qt2x3 = Qt(3*j, 0, 2, 3);
	cerr << " --- I(2x2) ---\n" <<  Qt2x3 * Qt2x3.trns();
    }
    cerr << "*** End:   TU::projectiveToMetric() ***\n" << endl;
#endif    
}

/************************************************************************
*  global functions							*
************************************************************************/
template <class CAMERA> Array<CAMERA>
calibCamerasWithPlanes(const PairListArrayList& data, bool doRefinement)
{
    using namespace	std;

    if (data.size() < 3)
	throw runtime_error("Three or more planes needed!!");
#ifdef DEBUG
    cerr << "*** Begin: TU::calibCamerasWithPlanes() ***" << endl;
#endif
  // Compute homography matrices for each plane-camera pair.
    Matrix<double>	W = computeHomographies(data);

  // Normalize the homography matrices.
    Array<Normalize>	norm = computeNormalization(data);
    for (int i = 0; i < W.nrow()/3; ++i)
	for (int j = 0; j < W.ncol()/3; ++j)
	    W(3*i, 3*j, 3, 3)
		= norm[1+i].T() * W(3*i, 3*j, 3, 3) * norm[0].Tinv();

  // Rescale the homography matrices and factor them into cameras and planes,
    rescaleHomographies(W);
    Matrix<double>	P, Qt;
    factorHomographies(W, P, Qt);

  // Transform camera matrices and plane parameters to Euclidean frame.
    projectiveToMetric(P, Qt);
    
  // Unnormalize computed camera and plane parameters.
    const double	scale = norm[0].Tinv()[0][0];
    for (int i = 0; i < P.nrow()/3; ++i)
	P(3*i, 0, 3, 4) = norm[1+i].Tinv() * P(3*i, 0, 3, 4);
    P(0, 3, P.nrow(), 1) *= scale;
    for (int j = 0; j < Qt.nrow()/3; ++j)
	Qt(3*j, 0, 3, 3) = norm[0].Tt() * Qt(3*j, 0, 3, 3);
    Qt(0, 0, Qt.nrow(), 3) *= scale;

  // Set camera matrices.
    Array<CAMERA>	cameras(P.nrow() / 3);
    for (int i = 0; i < cameras.dim(); ++i)
	cameras[i].setProjection(P(3*i, 0, 3, 4));

  // Do refinement if required.
    if (doRefinement)
    {
	Matrix<double>	S = refineCalibration(data, cameras, Qt);
      /*
	u_int		npoints = 0;
	for (PairListArrayList::const_iterator
		 iter = data.begin(); iter != data.end(); ++iter)
	    for (int i = 0; i < iter->dim(); ++i)
		npoints += (*iter)[i].size();
	S /= double(npoints);
	for (int ii = 0, i = 0; i < cameras.dim(); ++i)
	{
	    const double	DEG = 180.0 / M_PI;
	    cerr << " --- " << i << "-th camera: RMS error ---" << endl;
	    if (i != 0)
	    {
		cerr <<   "  Position:        "
		     << sqrt(S[ii][ii]) << ' '
		     << sqrt(S[ii+1][ii+1]) << ' '
		     << sqrt(S[ii+2][ii+2])
		     << "\n  Rotation:        "
		     << DEG * sqrt(S[ii+3][ii+3]) << ' '
		     << DEG * sqrt(S[ii+4][ii+4]) << ' '
		     << DEG * sqrt(S[ii+5][ii+5])
		     << endl;
		ii += 6;
	    }
	    cerr <<   "  Focal lengths:   " << sqrt(S[ii][ii])
		 << "\n  Principal point: " << sqrt(S[ii+1][ii+1])
		 << ' '			    << sqrt(S[ii+2][ii+2])
		 << "\n  Aspect ratio:    " << sqrt(S[ii+3][ii+3])
		 << "\n  Skew:            " << sqrt(S[ii+4][ii+4])
	      //<< "\n  d1:              " << sqrt(S[ii+5][ii+5])
	      // << "\n  d2:              " << sqrt(S[ii+6][ii+6])
		 << endl;
	    ii += cameras[i].dofIntrinsic();
	    }*/
    }
#ifdef DEBUG
    cerr << "\n*** End:   TU::calibCamerasWithPlanes() ***\n" << endl;
#endif
    return cameras;
}

template Array<Camera>
calibCamerasWithPlanes<Camera>(const PairListArrayList& data,
			       bool doRefinement);
template Array<CameraWithDistortion>
calibCamerasWithPlanes<CameraWithDistortion>(const PairListArrayList& data,
					     bool doRefinement);
}
