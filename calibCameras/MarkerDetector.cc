/*
 *  $Id: MarkerDetector.cc,v 1.3 2007-05-23 23:47:35 ueshiba Exp $
 */
#include <algorithm>
#include "TU/Ransac++.h"
#include "TU/utility.h"
#include "MarkerDetector.h"
#ifdef DEBUG
#  include "debug.h"
#endif

namespace TU
{
/************************************************************************
*   static functions							*
************************************************************************/
static inline bool
isMinimum(const float* prv, const float* cur, const float* nxt)
{
    return (*cur < *(prv - 1) && *cur < *prv && *cur < *(prv + 1) &&
	    *cur < *(cur - 1)		     && *cur < *(cur + 1) &&
	    *cur < *(nxt - 1) && *cur < *nxt && *cur < *(nxt + 1));
}
    
static inline bool
isMaximum(const float* prv, const float* cur, const float* nxt)
{
    return (*cur > *(prv - 1) && *cur > *prv && *cur > *(prv + 1) &&
	    *cur > *(cur - 1)		     && *cur > *(cur + 1) &&
	    *cur > *(nxt - 1) && *cur > *nxt && *cur > *(nxt + 1));
}

/************************************************************************
*   class QuadricSurface						*
************************************************************************/
class QuadricSurface
{
  public:
    template <class Iterator>
    QuadricSurface(Iterator first, Iterator last)			;

    Point2<double>	center()				const	;

  private:
    Matrix<double>	_A;
};

template <class Iterator>
QuadricSurface::QuadricSurface(Iterator first, Iterator last)
    :_A(3, 3)
{
    const Normalize	normalize(make_const_first_iterator(first),
				  make_const_first_iterator(last));
    Matrix<double>	M(6, 6);
    Vector<double>	a(6);
    for (Iterator iter = first; iter != last; ++iter)
    {
	const Vector<double>&	x = normalize(iter->first);
	Vector<double>		m(6);
	m[0] =	   x[0]*x[0];
	m[1] = 2.0*x[0]*x[1];
	m[2] = 2.0*x[0];
	m[3] =	   x[1]*x[1];
	m[4] = 2.0*x[1];
	m[5] = 1.0;

	M += m % m;
	a += iter->second * m;
    }
    a.solve(M);
    _A[0][0]		= a[0];
    _A[0][1] = _A[1][0] = a[1];
    _A[0][2] = _A[2][0] = a[2];
    _A[1][1]		= a[3];
    _A[1][2] = _A[2][1] = a[4];
    _A[2][2]		= a[5];
    _A = normalize.Tt() * _A * normalize.T();

  //print(std::cout, _A);
  //print(std::cout, first, last);
}

inline Point2<double>
QuadricSurface::center() const
{
    Point2<double>	c(-_A[2][0], -_A[2][1]);
    return c.solve(_A(0, 0, 2, 2));
}

/************************************************************************
*   class MarkerDetector						*
************************************************************************/
MarkerDetector::MarkerDetector(u_int cropSize,
			       float strengthTh, float distTh, float alpha,
			       float th_low, float th_high, float conformTh)
    :_cropSize(cropSize), _strengthTh(std::min(strengthTh, 1.0f)),
     _sqdistTh(distTh*distTh), _alpha(alpha),
     _th_low(th_low), _th_high(th_high), _sqconformTh(conformTh*conformTh)
{
}

const MarkerDetector&
MarkerDetector::operator ()(const Image<u_char>& image,
			    PairList& pairs, bool edgeIntersection) const
{
    using namespace	std;
    
  // 原点を検出するため，入力画像に十字テンプレートを適用．
    Image<float>	strength;
    IntegralImage<int>	integralImage(image);
    integralImage.crossVal(strength, _cropSize);

  // 極大点を検出．
    MarkerList	originCandidates;
    for (int v = _cropSize; v < strength.height() - _cropSize; ++v)
    {
	const float*	prv = &strength[v-1][_cropSize];
	const float*	cur = &strength[v  ][_cropSize];
	const float*	nxt = &strength[v+1][_cropSize];
	
	for (int u = _cropSize; u < strength.width() - _cropSize; ++u)
	{
	    if (isMaximum(prv, cur, nxt))
		originCandidates.push_back(Marker(u, v, *cur));
	    ++prv;
	    ++cur;
	    ++nxt;
	}
    }

  // マーカ点を検出のため，入力画像に×字テンプレートを適用．
    DiagonalIntegralImage<int>	diagonalIntegralImage(image);
    diagonalIntegralImage.crossVal(strength, _cropSize);
    
  // 極大点を検出．
    MarkerList	candidatesH, candidatesV;
    for (int v = _cropSize; v < strength.height() - _cropSize; ++v)
    {
	const float*	prv = &strength[v-1][_cropSize];
	const float*	cur = &strength[v  ][_cropSize];
	const float*	nxt = &strength[v+1][_cropSize];
	
	for (int u = _cropSize; u < strength.width() - _cropSize; ++u)
	{
	    if (isMaximum(prv, cur, nxt))
		candidatesH.push_back(Marker(u, v, *cur));
	    else if (isMinimum(prv, cur, nxt))
		candidatesV.push_back(Marker(u, v, *cur));
	    ++prv;
	    ++cur;
	    ++nxt;
	}
    }
    
  // 最大強度の一定割合に満たない弱い極大点を除去．
    MarkerList::const_iterator	m = max_element(originCandidates.begin(),
						originCandidates.end());
    originCandidates.erase(remove_if(originCandidates.begin(),
				     originCandidates.end(),
				     bind2nd(less<Marker>(),
					     Marker(0, 0,
						    (0.5 + 0.5*_strengthTh)
						    *m->strength))),
			   originCandidates.end());
    m = max_element(candidatesH.begin(), candidatesH.end());
    candidatesH.erase(remove_if(candidatesH.begin(), candidatesH.end(),
				bind2nd(less<Marker>(),
					Marker(0, 0,
					       _strengthTh * m->strength))),
		      candidatesH.end());
    m = min_element(candidatesV.begin(), candidatesV.end());
    candidatesV.erase(remove_if(candidatesV.begin(), candidatesV.end(),
				bind2nd(greater<Marker>(),
					Marker(0, 0,
					       _strengthTh * m->strength))),
		      candidatesV.end());
#ifdef DEBUG
    Image<RGB>	tmp = Y2RGB(image);
    for (MarkerList::const_iterator iter = originCandidates.begin();
	 iter != originCandidates.end(); ++iter)
	drawMarker(tmp, *iter, RGB(0, 255, 255));
    for (MarkerList::const_iterator iter = candidatesH.begin();
	 iter != candidatesH.end(); ++iter)
	drawMarker(tmp, *iter, RGB(0, 255, 0));
    for (MarkerList::const_iterator iter = candidatesV.begin();
	 iter != candidatesV.end(); ++iter)
	drawMarker(tmp, *iter, RGB(255, 0, 0));
    tmp.save(std::cout, ImageBase::RGB_24);
#endif

  // 検出したマーカ点（ベースマーカ点を除く）の位置を高精度化する．
    if (edgeIntersection)
    {
	Image<float>	edgeH, edgeV;
	DericheConvolver(_alpha).diffH(image, edgeH).diffV(image, edgeV);
	Image<float>	str;
	Image<u_char>	dir, edgeImage;
	EdgeDetector(_th_low, _th_high).strength(edgeH, edgeV, str)
				       .direction4(edgeH, edgeV, dir)
				       .suppressNonmaxima(str, dir, edgeImage)
				       .hysteresisThresholding(edgeImage);
	refineMarkerLocation(edgeImage, candidatesH);
	refineMarkerLocation(edgeImage, candidatesV);
    }
    else
    {
	Image<float>	smoothedImage;
	DericheConvolver(_alpha).smooth(image, smoothedImage);
	refineMarkerLocation2(smoothedImage, originCandidates);
	refineMarkerLocation2(smoothedImage, candidatesH);
	refineMarkerLocation2(smoothedImage, candidatesV);
    }
    
  // 強度の強い順に原点の候補をとり，それを囲む４つのベースマーカ点を探す．
    ProjectiveMappingWithDistortion
	H(Point2<int>(image.width()/2, image.height()/2));
    originCandidates.sort(greater<Marker>());	// 強度の強い順にソート
    for (m = originCandidates.begin(); m != originCandidates.end(); ++m)
    {
	pairs.clear();
	
      // 原点に最も近いベースマーカ点対をそれぞれ横軸／縦軸上で求める．
	MarkerList::iterator	pH, qH, pV, qV;
	if (findBaseMarkers(*m, candidatesH, true,  pH, qH) &&
	    findBaseMarkers(*m, candidatesV, false, pV, qV))
	{
	  // 原点と４つのベースマーカ点から射影変換行列を計算する．
	    pairs.push_back(make_pair(PointF( 0,  0), *m));
	    pairs.push_back(make_pair(PointF(-1,  0), *pH));
	    candidatesH.erase(pH);
	    pairs.push_back(make_pair(PointF( 1,  0), *qH));
	    candidatesH.erase(qH);
	    pairs.push_back(make_pair(PointF( 0, -1), *pV));
	    candidatesV.erase(pV);
	    pairs.push_back(make_pair(PointF( 0,  1), *qV));
	    candidatesV.erase(qV);
	    H.initialize(pairs.begin(), pairs.end(), false);
	    PairList::const_iterator	iter = pairs.begin();
	    if ((H.sqdist(*iter++) < _sqdistTh) &&
		(H.sqdist(*iter++) < _sqdistTh) &&
		(H.sqdist(*iter++) < _sqdistTh) &&
		(H.sqdist(*iter)   < _sqdistTh))
		break;
	}
    }
    if (m == originCandidates.end())
	throw runtime_error("MarkerDetector::operator (): the origin and surronding base markers not found!!");

  // ベースマーカ点のまわりの４点のいずれかを出発点として他のマーカ点を探す．
    findMarkers( 2,  1, candidatesH, candidatesV, H, pairs);
    findMarkers(-1,  2, candidatesH, candidatesV, H, pairs);
    findMarkers(-2, -1, candidatesH, candidatesV, H, pairs);
    findMarkers( 1, -2, candidatesH, candidatesV, H, pairs);

    return *this;
}

bool
MarkerDetector::findBaseMarkers(const Marker& origin, MarkerList& candidates,
				bool horizontal,
				MarkerList::iterator& p,
				MarkerList::iterator& q) const
{
    candidates.sort(NearerTo(origin));
    for (p = candidates.begin(); p != candidates.end(); ++p)
	if (p->sqdist(origin) > _cropSize)  // 充分離れているかチェックし，原
	{			// 点がマーカ点としても検出された場合に備える．
	    const PointF	r(2.0*origin - *p);  //p と対象位置にある点
	
	    q = std::min_element(candidates.begin(), candidates.end(),
				 NearerTo(r));
	    if (q->sqdist(r) < _sqdistTh)  // 予測位置に充分近ければ…
	    {
		if (horizontal)
		{
		    if ((*p)[0] > (*q)[0])
			std::swap(p, q);
		}
		else
		{
		    if ((*p)[1] > (*q)[1])
			std::swap(p, q);
		}
		
		return true;
	    }
	}

    return false;
}

void
MarkerDetector::findMarkers(int i, int j,
			    MarkerList& candidatesH, MarkerList& candidatesV,
			    ProjectiveMappingWithDistortion& H,
			    PairList& pairs) const
{
    MarkerList&			candidates = (i % 2 ? candidatesH
						    : candidatesV);
    const PointF&		r = H(PointF(i, j));
    MarkerList::iterator	p = std::min_element(candidates.begin(),
						     candidates.end(),
						     NearerTo(r));
    if ((p != candidates.end()) && (p->sqdist(r) < _sqdistTh))
    {
	pairs.push_back(std::make_pair(PointF(i, j), *p));
	candidates.erase(p);

	H.initialize(pairs.begin(), pairs.end(), true);
	
	findMarkers(i+1, j+1, candidatesH, candidatesV, H, pairs);
	findMarkers(i-1, j+1, candidatesH, candidatesV, H, pairs);
	findMarkers(i-1, j-1, candidatesH, candidatesV, H, pairs);
	findMarkers(i+1, j-1, candidatesH, candidatesV, H, pairs);
    }
}

void
MarkerDetector::refineMarkerLocation(const Image<u_char>& edgeImage,
				     MarkerList& markers) const
{
    using namespace	std;

    for (MarkerList::iterator p = markers.begin(); p != markers.end(); )
    {
      // pを中心とする正方形ウィンドウ内におけるエッジ点を検出．
	Image<u_char>	window(edgeImage,
			       int((*p)[0]) - _cropSize,
			       int((*p)[1]) - _cropSize,
			       2*_cropSize + 1, 2*_cropSize + 1);

	vector<PointF>	edgePoints;
	for (int v = 0; v < window.height(); ++v)
	    for (int u = 0; u < window.width(); ++u)
		if (window[v][u])
		    edgePoints.push_back(PointF(u, v));
    
      // エッジ点を4つの象限に分類．
	vector<PointF>::iterator
	    lowerLeft = pull_if(edgePoints.begin(), edgePoints.end(),
				AboveOf(_cropSize));
	vector<PointF>::iterator
	    upperRight = pull_if(edgePoints.begin(), lowerLeft,
				 LeftOf(_cropSize));
	vector<PointF>::iterator
	    lowerRight = pull_if(lowerLeft, edgePoints.end(),
				 LeftOf(_cropSize));
	vector<PointF>::difference_type
	    lowerRightSize = edgePoints.end() - lowerRight;
	rotate(edgePoints.begin(), lowerRight, edgePoints.end());
	lowerRight = edgePoints.begin();
	vector<PointF>::iterator
	    upperLeft = lowerRight + lowerRightSize;
	upperRight += lowerRightSize;
	lowerLeft  += lowerRightSize;

	if ((lowerRight != upperLeft) && (upperLeft != upperRight) &&
	    (upperRight != lowerLeft) && (lowerLeft != edgePoints.end()))
	{
	  // pの周りの4象限を点対称な2組に分け，それぞれからRANSACで直線を検出．
	    PointSet		pointSetP(lowerRight, upperLeft, upperRight,
					  0.5);
	    HyperPlane		lineP(2);
	    const PointSet::Container&
				inliersP = ransac(pointSetP, lineP,
						  Conform(_sqconformTh));
	    PointSet		pointSetN(upperRight, lowerLeft,
					  edgePoints.end(), 0.5);
	    HyperPlane		lineN(2);
	    const PointSet::Container&
				inliersN = ransac(pointSetN, lineN,
						  Conform(_sqconformTh));
#ifdef DEBUG
	    Image<RGB>	tmp = Y2RGB(window);
	    for (vector<PointF>::const_iterator iter  = edgePoints.begin();
						iter != edgePoints.end(); )
		tmp(*iter++) = RGB(255, 255, 0);
	    for (vector<PointF>::const_iterator iter  = inliersP.begin();
						iter != inliersP.end(); )
		tmp(*iter++) = RGB(255, 0, 0);
	    for (vector<PointF>::const_iterator iter  = inliersN.begin();
						iter != inliersN.end(); )
		tmp(*iter++) = RGB(0, 255, 0);
#endif
	  // 2直線の交点を計算．
	    Vector<double>	delta = lineP.p() ^ lineN.p();
	    (delta[0] /= delta[2]) -= _cropSize;
	    (delta[1] /= delta[2]) -= _cropSize;
	    if (fabs(delta[0]) > 2.0 || fabs(delta[1]) > 2.0)
	    {  // 修正量が2 pixelを越えればマーカ点でない。 
		cerr << "correction at [" << (*p)[0] << ", " << (*p)[1]
		     << "] with more than 1 pixel!! ("
		     << delta[0] << ", " << delta[1] << ')' << endl;
		markers.erase(p++);
	    }
	    else
	    {  // 修正量が2 pixel以内であれば修正する。
		(*p)[0] += delta[0];
		(*p)[1] += delta[1];
		++p;
	    }
	}
	else
	    ++p;
    }
    
}
    
void
MarkerDetector::refineMarkerLocation2(const Image<float>& smoothedImage,
				      MarkerList& markers) const
{
    using namespace	std;
    
    for (MarkerList::iterator p = markers.begin(); p != markers.end(); )
    {
	Image<float>	window(smoothedImage,
			       int((*p)[0]) - _cropSize,
			       int((*p)[1]) - _cropSize,
			       2*_cropSize + 1, 2*_cropSize + 1);

      // 輝度分布に２次曲面を当てはめ．
	vector<pair<PointF, float> >	data;
	for (int v = 0; v < window.height(); ++v)
	    for (int u = 0; u < window.width(); ++u)
		data.push_back(make_pair(PointF(u, v), window[v][u]));
	QuadricSurface	quad(data.begin(), data.end());

      // ２次曲面の中心を修正されたマーカ位置とし．修正量を求める．
	Point2<double>	delta = quad.center();
	delta[0] -= _cropSize;
	delta[1] -= _cropSize;
	if (fabs(delta[0]) > 2.0 || fabs(delta[1]) > 2.0)
	{  // 修正量が2 pixelを越えればマーカ点でない．
	    cerr << "correction at [" << (*p)[0] << ", " << (*p)[1]
		 << "] with more than 1 pixel!! ("
		 << delta[0] << ", " << delta[1] << ')' << endl;
	    markers.erase(p++);
	}
	else
	{  // 修正量が2 pixel以内であれば修正する．
	    (*p)[0] += delta[0];
	    (*p)[1] += delta[1];
	    ++p;
	}
    }
}
    
/************************************************************************
*   class MarkerDetector::PointSet					*
************************************************************************/
MarkerDetector::PointSet::PointSet(const_iterator first, const_iterator middle,
				   const_iterator last,  double rate)
    :_first(first), _middle(middle), _last(last), _inlierRate(rate)
{
    srand48(long(time(0)));
}

MarkerDetector::PointSet::Container
MarkerDetector::PointSet::sample(u_int npoints) const
{
    Container	samplePoints;
    int		n = lrand48() % (_middle - _first);
    samplePoints.push_back(*(_first + n));
    n = lrand48() % (_last - _middle);
    samplePoints.push_back(*(_middle + n));
    
    return samplePoints;
}

}
