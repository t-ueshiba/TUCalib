/*
 *  平成14-19年（独）産業技術総合研究所 著作権所有
 *  
 *  創作者：植芝俊夫
 *
 *  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
 *  （独）産業技術総合研究所が著作権を所有する秘密情報です．著作権所有
 *  者による許可なしに本プログラムを使用，複製，改変，第三者へ開示する
 *  等の行為を禁止します．
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
  \brief	クラス#TU::IntensityCalibratorの定義と実装
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
//! 参照平面を利用してカメラの輝度キャリブレーションを行うクラス
class IntensityCalibrator
{

  public:
  //! 同一座標点における対象画像と参照画像の輝度ペア
    template <class T> struct IntensityPair : public std::pair<T, T>
    {
      //! 同一座標点における対象画像と参照画像の輝度ペアを生成する．
      /*!
	\param target	対象画像の輝度値
	\param ref	参照画像の輝度値
	\param uu	画像点の横座標
	\param vv	画像点の縦座標
      */
	IntensityPair(const T& target, const T& ref, int uu, int vv)
	    :std::pair<T, T>(target, ref), u(uu), v(vv)		{}

      //! 参照画像点の輝度を返す．
		operator T()	const	{return std::pair<T, T>::second;}
	
	std::pair<T, T>::first;				//!< 対象画像点の輝度
	std::pair<T, T>::second;			//!< 参照画像点の輝度
    
	int	u;					//!< 画像点の横座標
	int	v;					//!< 画像点の縦座標
    };

  //! 対象画像の輝度を参照画像に合わせるための変換係数を表すクラス
  /*!
　  対象画像の輝度Iが参照画像の輝度Irefと\f$I_{\TUsub{ref}} = a + b
    I\f$なる関係で結ばれていると見なしたときのオフセットaとゲインbが，
    それぞれベクトルの第0成分と第1成分になる．
  */
    class Slant : public Vector2f
    {
      public:
	Slant()							;

	template <class Iterator>
	void	update(Iterator begin, Iterator end)		;
	template <class Iterator>
	void	fit(Iterator begin, Iterator end)		;

      //! 変換係数を決定するために必要な最小点数を返す．
      /*
	\return		必要な最小点数
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
  //! 輝度キャリブレーション器を生成する．
  /*!
    \param inlierRate	位置合わせされていて輝度変換モデルに従うと期待される画素の割合
    \param conformTh	輝度変換モデルへ適合しているとみなされる輝度許容誤差
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
//! 輝度ペア中の参照画像の輝度の大小を比較する．
template <class T> static inline bool
operator <(const IntensityCalibrator::IntensityPair<T>& a,
	   const IntensityCalibrator::IntensityPair<T>& b)
{
    return a.second < b.second;
}

//! 輝度ペア中の参照画像の輝度が一致するか調べる．
template <class T> static inline bool
operator !=(const IntensityCalibrator::IntensityPair<T>& a,
	    const IntensityCalibrator::IntensityPair<T>& b)
{
    return a.second != b.second;
}

/************************************************************************
*  class IntensityCalibrator::Slant					*
************************************************************************/
//! 輝度変換係数を生成する．
/*!
  オフセットとゲインは，それぞれ0と1に初期化される．
*/
inline
IntensityCalibrator::Slant::Slant()
    :_A(), _b()
{
    (*this)[0] = 0.0;
    (*this)[1] = 1.0;
}

//! 対象画像と参照画像の輝度ペアを新たに与えて変換係数を更新する．
/*!
  これまでに与えた輝度の組のデータも反映される．
  \param begin	最初の輝度ペアを示す反復子
  \param end	最後の輝度ペアの次を示す反復子
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
    
//! 対象画像と参照画像の輝度ペアを与えて変換係数を計算する．
/*!
  これまでに与えた輝度の組のデータは反映されない．
  \param begin	最初の輝度ペアを示す反復子
  \param end	最後の輝度ペアの次を示す反復子
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

//! 与えられた対象画像と参照画像の輝度ペアに対してモデルを当てはめたときの誤差を返す．
/*!
  対象画像と参照画像の輝度をそれぞれI, Irefとしたとき，
  \f$a + b I - I_{\TUsub{ref}}\f$を返す．ただし，aとbはそれぞれオフセットとゲイン．
  \param x	輝度ペア
  \return	モデルの当てはめ誤差
*/
template <class T> inline float
IntensityCalibrator::Slant::dist(const std::pair<T, T>& x) const
{
    return (*this)[0] + (*this)[1] * x.first - x.second;
}

//! 与えられた対象画像と参照画像の輝度ペアに対してモデルを当てはめたときの2乗誤差を返す．
/*!
  #dist()で返される値の2乗を返す．
  \param x	輝度ペア
  \return	モデルの当てはめ2乗誤差
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
//! 対象画像の輝度を参照画像に適合させる変換係数を求める．
/*!
  \param image		対象画像
  \param refImage	参照画像
  \return		オフセットとゲインを収めた2次元ベクトル
*/
template <class T> inline Vector2f
IntensityCalibrator::operator ()(const Image<T>& image,
				 const Image<T>& refImage) const
{
    Slant	slant;
    getInliers(image, refImage, slant);

    return slant;
}

//! 対象画像の輝度を参照画像に適合させる変換係数と適合画素マップを求める．
/*!
  \param image		対象画像
  \param refImage	参照画像
  \param maskImage	対象画像と参照画像の輝度の関係が変換モデルに適合する画素の値を
			255としたマップ
  \return		オフセットとゲインを収めた2次元ベクトル
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

//! 対象画像と参照画像を新たに与えて変換係数を更新する．
/*!
  \param image		対象画像
  \param refImage	参照画像
  \param slant		更新すべき変換係数
  \return		オフセットとゲインを収めた2次元ベクトル
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

//! 対象画像と参照画像を新たに与えて変換係数を更新し，さらに適合画素マップを求める．
/*!
  \param image		対象画像
  \param refImage	参照画像
  \param slant		更新すべき変換係数
  \param maskImage	対象画像と参照画像の輝度の関係が変換モデルに適合する画素の値を
			255としたマップ
  \return		オフセットとゲインを収めた2次元ベクトル
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
