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
    
  // $B86E@$r8!=P$9$k$?$a!$F~NO2hA|$K==;z%F%s%W%l!<%H$rE,MQ!%(B
    Image<float>	strength;
    IntegralImage<int>	integralImage(image);
    integralImage.crossVal(strength, _cropSize);

  // $B6KBgE@$r8!=P!%(B
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

  // $B%^!<%+E@$r8!=P$N$?$a!$F~NO2hA|$K!_;z%F%s%W%l!<%H$rE,MQ!%(B
    DiagonalIntegralImage<int>	diagonalIntegralImage(image);
    diagonalIntegralImage.crossVal(strength, _cropSize);
    
  // $B6KBgE@$r8!=P!%(B
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
    
  // $B:GBg6/EY$N0lDj3d9g$KK~$?$J$$<e$$6KBgE@$r=|5n!%(B
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

  // $B8!=P$7$?%^!<%+E@!J%Y!<%9%^!<%+E@$r=|$/!K$N0LCV$r9b@:EY2=$9$k!%(B
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
    
  // $B6/EY$N6/$$=g$K86E@$N8uJd$r$H$j!$$=$l$r0O$`#4$D$N%Y!<%9%^!<%+E@$rC5$9!%(B
    ProjectiveMappingWithDistortion
	H(Point2<int>(image.width()/2, image.height()/2));
    originCandidates.sort(greater<Marker>());	// $B6/EY$N6/$$=g$K%=!<%H(B
    for (m = originCandidates.begin(); m != originCandidates.end(); ++m)
    {
	pairs.clear();
	
      // $B86E@$K:G$b6a$$%Y!<%9%^!<%+E@BP$r$=$l$>$l2#<4!?=D<4>e$G5a$a$k!%(B
	MarkerList::iterator	pH, qH, pV, qV;
	if (findBaseMarkers(*m, candidatesH, true,  pH, qH) &&
	    findBaseMarkers(*m, candidatesV, false, pV, qV))
	{
	  // $B86E@$H#4$D$N%Y!<%9%^!<%+E@$+$i<M1FJQ499TNs$r7W;;$9$k!%(B
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

  // $B%Y!<%9%^!<%+E@$N$^$o$j$N#4E@$N$$$:$l$+$r=PH/E@$H$7$FB>$N%^!<%+E@$rC5$9!%(B
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
	if (p->sqdist(origin) > _cropSize)  // $B=<J,N%$l$F$$$k$+%A%'%C%/$7!$86(B
	{			// $BE@$,%^!<%+E@$H$7$F$b8!=P$5$l$?>l9g$KHw$($k!%(B
	    const PointF	r(2.0*origin - *p);  //p $B$HBP>]0LCV$K$"$kE@(B
	
	    q = std::min_element(candidates.begin(), candidates.end(),
				 NearerTo(r));
	    if (q->sqdist(r) < _sqdistTh)  // $BM=B,0LCV$K=<J,6a$1$l$P!D(B
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
      // p$B$rCf?4$H$9$k@5J}7A%&%#%s%I%&Fb$K$*$1$k%(%C%8E@$r8!=P!%(B
	Image<u_char>	window(edgeImage,
			       int((*p)[0]) - _cropSize,
			       int((*p)[1]) - _cropSize,
			       2*_cropSize + 1, 2*_cropSize + 1);

	vector<PointF>	edgePoints;
	for (int v = 0; v < window.height(); ++v)
	    for (int u = 0; u < window.width(); ++u)
		if (window[v][u])
		    edgePoints.push_back(PointF(u, v));
    
      // $B%(%C%8E@$r(B4$B$D$N>]8B$KJ,N`!%(B
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
	  // p$B$N<~$j$N(B4$B>]8B$rE@BP>N$J(B2$BAH$KJ,$1!$$=$l$>$l$+$i(BRANSAC$B$GD>@~$r8!=P!%(B
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
	  // 2$BD>@~$N8rE@$r7W;;!%(B
	    Vector<double>	delta = lineP.p() ^ lineN.p();
	    (delta[0] /= delta[2]) -= _cropSize;
	    (delta[1] /= delta[2]) -= _cropSize;
	    if (fabs(delta[0]) > 2.0 || fabs(delta[1]) > 2.0)
	    {  // $B=$@5NL$,(B2 pixel$B$r1[$($l$P%^!<%+E@$G$J$$!#(B 
		cerr << "correction at [" << (*p)[0] << ", " << (*p)[1]
		     << "] with more than 1 pixel!! ("
		     << delta[0] << ", " << delta[1] << ')' << endl;
		markers.erase(p++);
	    }
	    else
	    {  // $B=$@5NL$,(B2 pixel$B0JFb$G$"$l$P=$@5$9$k!#(B
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

      // $B51EYJ,I[$K#2<!6JLL$rEv$F$O$a!%(B
	vector<pair<PointF, float> >	data;
	for (int v = 0; v < window.height(); ++v)
	    for (int u = 0; u < window.width(); ++u)
		data.push_back(make_pair(PointF(u, v), window[v][u]));
	QuadricSurface	quad(data.begin(), data.end());

      // $B#2<!6JLL$NCf?4$r=$@5$5$l$?%^!<%+0LCV$H$7!%=$@5NL$r5a$a$k!%(B
	Point2<double>	delta = quad.center();
	delta[0] -= _cropSize;
	delta[1] -= _cropSize;
	if (fabs(delta[0]) > 2.0 || fabs(delta[1]) > 2.0)
	{  // $B=$@5NL$,(B2 pixel$B$r1[$($l$P%^!<%+E@$G$J$$!%(B
	    cerr << "correction at [" << (*p)[0] << ", " << (*p)[1]
		 << "] with more than 1 pixel!! ("
		 << delta[0] << ", " << delta[1] << ')' << endl;
	    markers.erase(p++);
	}
	else
	{  // $B=$@5NL$,(B2 pixel$B0JFb$G$"$l$P=$@5$9$k!%(B
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
