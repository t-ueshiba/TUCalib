/*
 *  $Id
 */
#include <vector>
#include <utility>
#include <list>
#include <algorithm>
#include <iterator>
#include "TU/Image++.h"
#include "ProjectiveMappingWithDistortion.h"

namespace TU
{
typedef Point2<float>			PointF;
typedef std::pair<PointF, PointF>	Pair;
typedef std::list<Pair>			PairList;
typedef Array<PairList>			PairListArray;
typedef std::list<PairListArray>	PairListArrayList;
typedef std::list<PairList>		PairListList;
    
/************************************************************************
*  class MarkerDetector							*
************************************************************************/
class MarkerDetector
{
  private:
    struct Marker : public PointF
    {
	Marker(float u, float v, float str) :PointF(u, v), strength(str){}

	float	strength;
    };

    typedef std::list<Marker>				MarkerList;
    
    struct NearerTo : public std::binary_function<PointF, PointF, bool>
    {
	NearerTo(const PointF& p) :_p(p)		{}
	
	bool	operator ()(const PointF& a, const PointF& b) const
		{
		    return _p.sqdist(a) < _p.sqdist(b);
		}

      private:
	const PointF&	_p;
    };

    struct Conform : public std::binary_function<PointF, HyperPlane, bool>
    {
	Conform(double thresh) :_thresh(thresh)		{}
	bool	operator ()(const PointF& p, const HyperPlane& line) const
		{
		    return line.sqdist(p) < _thresh;
		}

      private:
	const double	_thresh;
    };

    struct AboveOf : public std::unary_function<PointF, bool>
    {
	AboveOf(int v) :_v(v)				{}
	bool	operator ()(const PointF& p)	const	{return p[1] < _v;}

      private:
	const int	_v;
    };

    struct LeftOf : public std::unary_function<PointF, bool>
    {
	LeftOf(int u) :_u(u)				{}
	bool	operator ()(const PointF& p)	const	{return p[0] < _u;}

      private:
	const int	_u;
    };

    class PointSet
    {
      public:
	typedef std::vector<PointF>			Container;
	typedef Container::const_iterator		const_iterator;

	PointSet(const_iterator first, const_iterator middle,
		 const_iterator last, double rate)	;

	double		inlierRate()		const	{return _inlierRate;}
	const_iterator	begin()			const	{return _first;}
	const_iterator	end()			const	{return _last;}
	Container	sample(u_int npoints)	const	;
    
      private:
	const_iterator	_first;
	const_iterator	_middle;
	const_iterator	_last;
	double		_inlierRate;
    };

  public:
    MarkerDetector(u_int cropSize=10, float strengthTh=0.5,
		   float distTh=10.0, float alpha=1.0,
		   float th_low=2.0, float th_high=5.0,
		   float conformTh=0.8)					;

    const MarkerDetector&
		operator ()(const Image<u_char>& image,
			    PairList& pairs, bool refine)	const	;

    friend bool	operator <(const Marker& a, const Marker& b)		;
    friend bool	operator >(const Marker& a, const Marker& b)		;
    friend std::ostream&
		operator <<(std::ostream& out, const Marker& marker)	;

  private:
    bool	findBaseMarkers(const Marker& origin,
				MarkerList& candidates,
				bool horizontal,
				MarkerList::iterator& p,
				MarkerList::iterator& q)	const	;
    void	findMarkers(int i, int j,
			    MarkerList& candidatesH,
			    MarkerList& candidatesV,
			    ProjectiveMappingWithDistortion& H,
			    PairList& pairs)			const	;
    void	refineMarkerLocation(const Image<u_char>& edgeImage,
				     MarkerList& markers)	const	;
    void	refineMarkerLocation2(const Image<float>& smoothedImage,
				      MarkerList& markers)	const	;

    u_int	_cropSize;
    float	_strengthTh;
    double	_sqdistTh;
    float	_alpha;
    float	_th_low;
    float	_th_high;
    double	_sqconformTh;
};

inline bool
operator <(const MarkerDetector::Marker& a,
	   const MarkerDetector::Marker& b)
{
    return a.strength < b.strength;
}

inline bool
operator >(const MarkerDetector::Marker& a,
	   const MarkerDetector::Marker& b)
{
    return b < a;
}

inline std::ostream&
operator <<(std::ostream& out, const MarkerDetector::Marker& marker)
{
    return out << marker.strength
	       << '(' << marker[0] << ", " << marker[1] << ')';
}

inline std::istream&
operator >>(std::istream& in, Pair& pair)
{
    return in >> pair.first >> pair.second;
}
    
inline std::ostream&
operator <<(std::ostream& out, const Pair& pair)
{
    return out << pair.first << pair.second;
}

template <class T> std::istream&
operator >>(std::istream& in, std::list<T>& list)
{
    for (char c; in.get(c); )
	if (c == '\n')
	    break;
	else if (!isspace(c))
	{
	    in.putback(c);
	    T	element;
	    in >> element;
	    list.push_back(element);
	}

    return in;
}

template <class T> inline std::ostream&
operator <<(std::ostream& out, const std::list<T>& list)
{
    using namespace	std;
    
    copy(list.begin(), list.end(), ostream_iterator<T>(out));
    return out << endl;
}

}

