/*
 *  $BJ?@.(B14-19$BG/!JFH!K;:6H5;=QAm9g8&5f=j(B $BCx:n8"=jM-(B
 *  
 *  $BAO:n<T!'?"<G=SIW(B
 *
 *  $BK\%W%m%0%i%`$O!JFH!K;:6H5;=QAm9g8&5f=j$N?&0w$G$"$k?"<G=SIW$,AO:n$7!$(B
 *  $B!JFH!K;:6H5;=QAm9g8&5f=j$,Cx:n8"$r=jM-$9$kHkL)>pJs$G$9!%Cx:n8"=jM-(B
 *  $B<T$K$h$k5v2D$J$7$KK\%W%m%0%i%`$r;HMQ!$J#@=!$2~JQ!$Bh;0<T$X3+<($9$k(B
 *  $BEy$N9T0Y$r6X;_$7$^$9!%(B
 *  
 *  $B$3$N%W%m%0%i%`$K$h$C$F@8$8$k$$$+$J$kB;32$KBP$7$F$b!$Cx:n8"=jM-<T$*(B
 *  $B$h$SAO:n<T$O@UG$$rIi$$$^$;$s!#(B
 *
 *  Copyright 2002-2007.
 *  National Institute of Advanced Industrial Science and Technology (AIST)
 *
 *  Creator: Toshio UESHIBA
 *
 *  [AIST Confidential and all rights reserved.]
 *  This program is confidential. Any using, copying, changing or
 *  giving any information concerning with this program to others
 *  without permission by the copyright holder are strictly prohibited.
 *
 *  [No Warranty.]
 *  The copyright holder or the creator are not responsible for any
 *  damages caused by using this program.
 *
 *  $Id: IntensityCalibrator.h,v 1.2 2008-11-07 05:38:28 ueshiba Exp $
 */
/*!
  \file		IntensityCalibrator.h
  \brief	$B%/%i%9(B#TU::IntensityCalibrator$B$NDj5A$H<BAu(B
*/
#include <vector>
#include "TU/Image++.h"
#include "TU/Ransac.h"
#include "binarize.h"

namespace TU
{
/************************************************************************
*  class IntensityCalibrator						*
************************************************************************/
//! $B;2>HJ?LL$rMxMQ$7$F%+%a%i$N51EY%-%c%j%V%l!<%7%g%s$r9T$&%/%i%9(B
class IntensityCalibrator
{

  public:
  //! $BF10l:BI8E@$K$*$1$kBP>]2hA|$H;2>H2hA|$N51EY%Z%"(B
    template <class T> struct IntensityPair : public std::pair<T, T>
    {
      //! $BF10l:BI8E@$K$*$1$kBP>]2hA|$H;2>H2hA|$N51EY%Z%"$r@8@.$9$k!%(B
      /*!
	\param target	$BBP>]2hA|$N51EYCM(B
	\param ref	$B;2>H2hA|$N51EYCM(B
	\param uu	$B2hA|E@$N2#:BI8(B
	\param vv	$B2hA|E@$N=D:BI8(B
      */
	IntensityPair(const T& target, const T& ref, int uu, int vv)
	    :std::pair<T, T>(target, ref), u(uu), v(vv)		{}

      //! $B;2>H2hA|E@$N51EY$rJV$9!%(B
		operator T()	const	{return std::pair<T, T>::second;}
	
	std::pair<T, T>::first;				//!< $BBP>]2hA|E@$N51EY(B
	std::pair<T, T>::second;			//!< $B;2>H2hA|E@$N51EY(B
    
	int	u;					//!< $B2hA|E@$N2#:BI8(B
	int	v;					//!< $B2hA|E@$N=D:BI8(B
    };

  //! $BBP>]2hA|$N51EY$r;2>H2hA|$K9g$o$;$k$?$a$NJQ4978?t$rI=$9%/%i%9(B
  /*!
$B!!(B  $BBP>]2hA|$N51EY(BI$B$,;2>H2hA|$N51EY(BIref$B$H(B\f$I_{\TUsub{ref}} = a + b
    I\f$$B$J$k4X78$G7k$P$l$F$$$k$H8+$J$7$?$H$-$N%*%U%;%C%H(Ba$B$H%2%$%s(Bb$B$,!$(B
    $B$=$l$>$l%Y%/%H%k$NBh(B0$B@.J,$HBh(B1$B@.J,$K$J$k!%(B
  */
    class Slant : public Vector2f
    {
      public:
	Slant()							;

	template <class Iterator>
	void	update(Iterator begin, Iterator end)		;
	template <class Iterator>
	void	fit(Iterator begin, Iterator end)		;

      //! $BJQ4978?t$r7hDj$9$k$?$a$KI,MW$J:G>.E@?t$rJV$9!%(B
      /*
	\return		$BI,MW$J:G>.E@?t(B
      */
	u_int	ndataMin()				const	{return 2;}

	template <class T>
	float	dist(const std::pair<T, T>& x)		const	;
	template <class T>
	float	sqdist(const std::pair<T, T>& x)	const	;

      private:
	Matrix22f	_A;
	Vector2f	_b;
    };

  private:
    template <class T>
    class Conform : public std::binary_function<std::pair<T, T>, Slant, bool>
    {
      public:
	Conform(float thresh)	:_thresh(thresh)		{}
	bool	operator ()(const std::pair<T, T>& x, const Slant& slant) const
		{
		    return slant.sqdist(x) < _thresh;
		}
    
      private:
	const float	_thresh;
    };

    template <class T> class PointSet
    {
      public:
	typedef std::vector<IntensityPair<T> >		Container;
	typedef typename Container::const_iterator	const_iterator;

	PointSet(const_iterator first, const_iterator middle,
		 const_iterator last, double rate)		;

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
  //! $B51EY%-%c%j%V%l!<%7%g%s4o$r@8@.$9$k!%(B
  /*!
    \param inlierRate	$B0LCV9g$o$;$5$l$F$$$F51EYJQ49%b%G%k$K=>$&$H4|BT$5$l$k2hAG$N3d9g(B
    \param conformTh	$B51EYJQ49%b%G%k$XE,9g$7$F$$$k$H$_$J$5$l$k51EY5vMF8m:9(B
  */
    IntensityCalibrator(double inlierRate, double conformTh)
	:_inlierRate(inlierRate), _sqrConformTh(conformTh*conformTh)	{}

    template <class T>
    Vector2f		operator ()(const Image<T>& image,
				    const Image<T>& refImage)	const	;
    template <class T>
    Vector2f		operator ()(const Image<T>& image,
				    const Image<T>& refImage,
				    Image<u_char>& maskImage)	const	;
    template <class T>
    Vector2f		operator ()(const Image<T>& image,
				    const Image<T>& refImage,
				    Slant& slant)		const	;
    template <class T>
    Vector2f		operator ()(const Image<T>& image,
				    const Image<T>& refImage,
				    Slant& slant,
				    Image<u_char>& maskImage)	const	;

  private:
    template <class T> typename PointSet<T>::Container
	getInliers(const Image<T>& image,
		   const Image<T>& refImage, Slant& slant)	const	;

  private:
    double	_inlierRate;
    double	_sqrConformTh;
};

/************************************************************************
*  global functions							*
************************************************************************/
//! $B51EY%Z%"Cf$N;2>H2hA|$N51EY$NBg>.$rHf3S$9$k!%(B
template <class T> static inline bool
operator <(const IntensityCalibrator::IntensityPair<T>& a,
	   const IntensityCalibrator::IntensityPair<T>& b)
{
    return a.second < b.second;
}

//! $B51EY%Z%"Cf$N;2>H2hA|$N51EY$,0lCW$9$k$+D4$Y$k!%(B
template <class T> static inline bool
operator !=(const IntensityCalibrator::IntensityPair<T>& a,
	    const IntensityCalibrator::IntensityPair<T>& b)
{
    return a.second != b.second;
}

/************************************************************************
*  class IntensityCalibrator::Slant					*
************************************************************************/
//! $B51EYJQ4978?t$r@8@.$9$k!%(B
/*!
  $B%*%U%;%C%H$H%2%$%s$O!$$=$l$>$l(B0$B$H(B1$B$K=i4|2=$5$l$k!%(B
*/
inline
IntensityCalibrator::Slant::Slant()
    :_A(), _b()
{
    (*this)[0] = 0.0;
    (*this)[1] = 1.0;
}

//! $BBP>]2hA|$H;2>H2hA|$N51EY%Z%"$r?7$?$KM?$($FJQ4978?t$r99?7$9$k!%(B
/*!
  $B$3$l$^$G$KM?$($?51EY$NAH$N%G!<%?$bH?1G$5$l$k!%(B
  \param begin	$B:G=i$N51EY%Z%"$r<($9H?I|;R(B
  \param end	$B:G8e$N51EY%Z%"$N<!$r<($9H?I|;R(B
*/
template <class Iterator> void
IntensityCalibrator::Slant::update(Iterator begin, Iterator end)
{
    for (Iterator iter = begin; iter != end; ++iter)
    {
	Vector2f	x;
	x[0] = 1;
	x[1] = iter->first;

	_A += x % x;
	_b += iter->second * x;
    }
    float	det = _A[0][0] * _A[1][1] - _A[0][1] * _A[1][0];
    (*this)[0] = (_A[1][1] * _b[0] - _A[0][1] * _b[1]) / det;
    (*this)[1] = (_A[0][0] * _b[1] - _A[1][0] * _b[0]) / det;
}
    
//! $BBP>]2hA|$H;2>H2hA|$N51EY%Z%"$rM?$($FJQ4978?t$r7W;;$9$k!%(B
/*!
  $B$3$l$^$G$KM?$($?51EY$NAH$N%G!<%?$OH?1G$5$l$J$$!%(B
  \param begin	$B:G=i$N51EY%Z%"$r<($9H?I|;R(B
  \param end	$B:G8e$N51EY%Z%"$N<!$r<($9H?I|;R(B
*/
template <class Iterator> inline void
IntensityCalibrator::Slant::fit(Iterator begin, Iterator end)
{
    if (std::distance(begin, end) < 2)
	throw std::invalid_argument("IntensityCalibrator::Slant::fit: not enough input data!!");

    _A = 0.0;
    _b = 0.0;
    update(begin, end);
}

//! $BM?$($i$l$?BP>]2hA|$H;2>H2hA|$N51EY%Z%"$KBP$7$F%b%G%k$rEv$F$O$a$?$H$-$N8m:9$rJV$9!%(B
/*!
  $BBP>]2hA|$H;2>H2hA|$N51EY$r$=$l$>$l(BI, Iref$B$H$7$?$H$-!$(B
  \f$a + b I - I_{\TUsub{ref}}\f$$B$rJV$9!%$?$@$7!$(Ba$B$H(Bb$B$O$=$l$>$l%*%U%;%C%H$H%2%$%s!%(B
  \param x	$B51EY%Z%"(B
  \return	$B%b%G%k$NEv$F$O$a8m:9(B
*/
template <class T> inline float
IntensityCalibrator::Slant::dist(const std::pair<T, T>& x) const
{
    return (*this)[0] + (*this)[1] * x.first - x.second;
}

//! $BM?$($i$l$?BP>]2hA|$H;2>H2hA|$N51EY%Z%"$KBP$7$F%b%G%k$rEv$F$O$a$?$H$-$N(B2$B>h8m:9$rJV$9!%(B
/*!
  #dist()$B$GJV$5$l$kCM$N(B2$B>h$rJV$9!%(B
  \param x	$B51EY%Z%"(B
  \return	$B%b%G%k$NEv$F$O$a(B2$B>h8m:9(B
*/
template <class T> inline float
IntensityCalibrator::Slant::sqdist(const std::pair<T, T>& x) const
{
    const float	d = dist(x);
    return d*d;
}
    
/************************************************************************
*  class IntensityCalibrator::PointSet					*
************************************************************************/
template <class T> inline
IntensityCalibrator::PointSet<T>::PointSet(const_iterator first,
					   const_iterator middle,
					   const_iterator last, double rate)
    :_first(first), _middle(middle), _last(last), _inlierRate(rate)
{
    srand48(long(time(0)));
}

template <class T>
typename IntensityCalibrator::PointSet<T>::Container
IntensityCalibrator::PointSet<T>::sample(u_int npoints) const
{
    Container	samplePoints;
    int		n = lrand48() % (_middle - _first);
    samplePoints.push_back(*(_first + n));
    n = lrand48() % (_last - _middle);
    samplePoints.push_back(*(_middle + n));

    return samplePoints;
}

/************************************************************************
*  class IntensityCalibrator						*
************************************************************************/
//! $BBP>]2hA|$N51EY$r;2>H2hA|$KE,9g$5$;$kJQ4978?t$r5a$a$k!%(B
/*!
  \param image		$BBP>]2hA|(B
  \param refImage	$B;2>H2hA|(B
  \return		$B%*%U%;%C%H$H%2%$%s$r<}$a$?(B2$B<!85%Y%/%H%k(B
*/
template <class T> inline Vector2f
IntensityCalibrator::operator ()(const Image<T>& image,
				 const Image<T>& refImage) const
{
    Slant	slant;
    getInliers(image, refImage, slant);

    return slant;
}

//! $BBP>]2hA|$N51EY$r;2>H2hA|$KE,9g$5$;$kJQ4978?t$HE,9g2hAG%^%C%W$r5a$a$k!%(B
/*!
  \param image		$BBP>]2hA|(B
  \param refImage	$B;2>H2hA|(B
  \param maskImage	$BBP>]2hA|$H;2>H2hA|$N51EY$N4X78$,JQ49%b%G%k$KE,9g$9$k2hAG$NCM$r(B
			255$B$H$7$?%^%C%W(B
  \return		$B%*%U%;%C%H$H%2%$%s$r<}$a$?(B2$B<!85%Y%/%H%k(B
*/
template <class T> Vector2f
IntensityCalibrator::operator ()(const Image<T>& image,
				 const Image<T>& refImage,
				 Image<u_char>& maskImage) const
{
    typedef typename PointSet<T>::Container	Container;
    
    Slant		slant;
    const Container&	inliers = getInliers(image, refImage, slant);

    maskImage.resize(refImage.height(), refImage.width());
    maskImage = 0;
    for (typename Container::const_iterator iter = inliers.begin();
	 iter != inliers.end(); ++iter)
	maskImage[iter->v][iter->u] = 255;
    
    return slant;
}

//! $BBP>]2hA|$H;2>H2hA|$r?7$?$KM?$($FJQ4978?t$r99?7$9$k!%(B
/*!
  \param image		$BBP>]2hA|(B
  \param refImage	$B;2>H2hA|(B
  \param slant		$B99?7$9$Y$-JQ4978?t(B
  \return		$B%*%U%;%C%H$H%2%$%s$r<}$a$?(B2$B<!85%Y%/%H%k(B
*/
template <class T> inline Vector2f
IntensityCalibrator::operator ()(const Image<T>& image,
				 const Image<T>& refImage, Slant& slant) const
{
    typedef typename PointSet<T>::Container	Container;
    
    Slant		tmp;
    const Container&	inliers = getInliers(image, refImage, tmp);
    slant.update(inliers.begin(), inliers.end());
    
    return slant;
}

//! $BBP>]2hA|$H;2>H2hA|$r?7$?$KM?$($FJQ4978?t$r99?7$7!$$5$i$KE,9g2hAG%^%C%W$r5a$a$k!%(B
/*!
  \param image		$BBP>]2hA|(B
  \param refImage	$B;2>H2hA|(B
  \param slant		$B99?7$9$Y$-JQ4978?t(B
  \param maskImage	$BBP>]2hA|$H;2>H2hA|$N51EY$N4X78$,JQ49%b%G%k$KE,9g$9$k2hAG$NCM$r(B
			255$B$H$7$?%^%C%W(B
  \return		$B%*%U%;%C%H$H%2%$%s$r<}$a$?(B2$B<!85%Y%/%H%k(B
*/
template <class T> inline Vector2f
IntensityCalibrator::operator ()(const Image<T>& image,
				 const Image<T>& refImage, Slant& slant,
				 Image<u_char>& maskImage) const
{
    typedef typename PointSet<T>::Container	Container;
    
    Slant		tmp;
    const Container&	inliers = getInliers(image, refImage, tmp);
    slant.update(inliers.begin(), inliers.end());
    
    maskImage.resize(refImage.height(), refImage.width());
    maskImage = 0;
    for (typename Container::const_iterator iter = inliers.begin();
	 iter != inliers.end(); ++iter)
	maskImage[iter->v][iter->u] = 255;
    
    return slant;
}

template <class T> typename IntensityCalibrator::PointSet<T>::Container
IntensityCalibrator::getInliers(const Image<T>& image,
				const Image<T>& refImage, Slant& slant) const
{
    typedef std::vector<IntensityPair<T> >	IntensityPairs;

    IntensityPairs	intensityPairs;
    for (int v = 0; v < refImage.height(); ++v)
	for (int u = 0; u < refImage.width(); ++u)
	    intensityPairs.push_back(IntensityPair<T>(image[v][u],
						      refImage[v][u], u, v));
    typename IntensityPairs::const_iterator
	thresh = binarize(intensityPairs.begin(), intensityPairs.end());
    PointSet<T>
	pointSet(intensityPairs.begin(), thresh, intensityPairs.end(),
		 _inlierRate);
    return ransac(pointSet, slant, Conform<float>(_sqrConformTh));
}

}
