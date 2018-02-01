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
 *  $Id$  
 */
/*!
  \file		MarkerDetector.cc
  \brief	クラス#TU::MarkerDetectorの実装
*/
#include "MarkerDetector.h"
#include "TU/Ransac.h"
#include "TU/IntegralImage.h"
#include "TU/DericheConvolver.h"
#include "TU/EdgeDetector.h"
#ifdef _DEBUG
#  include "debug.h"
#endif
#ifdef USE_TBB
#  include <tbb/parallel_for.h>
#  include <tbb/blocked_range.h>
#endif

namespace TU
{
/************************************************************************
*   static functions							*
************************************************************************/
template <class P> static inline P
maxp(P a, P b)
{
    return (*a > *b ? a : b);
}
    
template <class P> static inline P
maxp(P a, P b, P c)
{
    return maxp(maxp(a, b), c);
}
    
template <class P> static inline P
maxp(P a, P b, P c, P d)
{
    return maxp(maxp(a, b, c), d);
}
    
template <class P> static inline P
minp(P a, P b)
{
    return (*a < *b ? a : b);
}
    
template <class P> static inline P
minp(P a, P b, P c)
{
    return minp(minp(a, b), c);
}
    
template <class P> static inline P
minp(P a, P b, P c, P d)
{
    return minp(minp(a, b, c), d);
}
    
//! 注目点を左上隅とする2x2領域中の最大点以外を0にする．
/*!
  \param u, v		注目点の2次元座標
  \param strengthOrg	元のテンプレート反応強度
  \param strength	最大点以外が0に抑制されたテンプレート反応強度
*/
template <class T> static inline void
suppressNonMaximum(int u, int v,
		   const Image<T>& strengthOrg, Image<T>& strength)
{
    const auto	a = &strengthOrg[v][u];
    const auto	b = &strengthOrg[v][u+1];
    const auto	c = &strengthOrg[v+1][u];
    const auto	d = &strengthOrg[v+1][u+1];
    const auto	p = maxp(a, b, c, d);

    if (p != a)
	strength[v][u] = 0;
    if (p != b)
	strength[v][u+1] = 0;
    if (p != c)
	strength[v+1][u] = 0;
    if (p != d)
	strength[v+1][u+1] = 0;
}

//! 注目点を左上隅とする2x2領域中の最小点以外を0にする．
/*!
  \param u, v		注目点の2次元座標
  \param strengthOrg	元のテンプレート反応強度
  \param strength	最小点以外が0に抑制されたテンプレート反応強度
*/
template <class T> static inline void
suppressNonMinimum(int u, int v,
		   const Image<T>& strengthOrg, Image<T>& strength)
{
    const T	*a = &strengthOrg[v][u],   *b = &strengthOrg[v][u+1],
		*c = &strengthOrg[v+1][u], *d = &strengthOrg[v+1][u+1],
		*p = minp(a, b, c, d);

    if (p != a)
	strength[v][u] = 0;
    if (p != b)
	strength[v][u+1] = 0;
    if (p != c)
	strength[v+1][u] = 0;
    if (p != d)
	strength[v+1][u+1] = 0;
}
    
/************************************************************************
*   class QuadricSurface						*
************************************************************************/
//! 双曲放物面を表すクラス
/*!
 2x2行列Aと2次元ベクトルbおよびスカラcを用いて
 \f[
 z = \TUbeginarray{cc} \TUtvec{x}{} & 1 \TUendarray
 \TUbeginarray{cc} \TUvec{A}{} & \TUvec{b}{} \\ \TUtvec{b}{} & c \TUendarray
 \TUbeginarray{c} \TUvec{x}{} \\ 1 \TUendarray
 \f]
 と表される2次曲面を表現する．
*/
class QuadricSurface : public Matrix33d
{
  public:
    template <class Iterator>
    QuadricSurface(Iterator begin, Iterator end)			;

    Point2d	center()					const	;
};

//! 与えられた画像点と輝度の対に当てはまる双曲放物面を生成する．
/*!
  テンプレートパラメータIteratorは，画像点の2次元座標を表す型Pと輝度を表す型I
  から成る対std::<P, I>を指す反復子である．
  \param begin	画像点と輝度の対の先頭を示す反復子
  \param end	画像点と輝度の対の末尾の次を示す反復子
*/
template <class Iterator>
QuadricSurface::QuadricSurface(Iterator begin, Iterator end)
    :Matrix33d()
{
    const Normalize<double>	normalize(make_first_iterator(begin),
					  make_first_iterator(end));
    Matrix<double, 6, 6>	M;
    Vector<double, 6>		a;
    for (auto iter = begin; iter != end; ++iter)
    {
	const auto	x = normalize(iter->first);
	Vector<double>	m(6);
	m[0] =	   x[0]*x[0];
	m[1] = 2.0*x[0]*x[1];
	m[2] = 2.0*x[0];
	m[3] =	   x[1]*x[1];
	m[4] = 2.0*x[1];
	m[5] = 1.0;

	M += m % m;
	a += iter->second * m;
    }
    solve(M, a);
    (*this)[0][0]		  = a[0];
    (*this)[0][1] = (*this)[1][0] = a[1];
    (*this)[0][2] = (*this)[2][0] = a[2];
    (*this)[1][1]		  = a[3];
    (*this)[1][2] = (*this)[2][1] = a[4];
    (*this)[2][2]		  = a[5];
    Matrix33d::operator =(evaluate(normalize.Tt() * *this * normalize.T()));

  //print(std::cout, *this);
  //print(std::cout, begin, end);
}

//! 双曲放物面の中心点の座標を求める．
/*!
  \return	中心点の2次元座標，すなわち\f$-\TUinv{A}{}\TUvec{b}{}\f$
*/
inline Point2d
QuadricSurface::center() const
{
    Point2d	c({-(*this)[2][0], -(*this)[2][1]});
    return solve(TU::slice(*this, 0, 2, 0, 2), c);
}

#ifdef USE_TBB
/************************************************************************
*   class Apply<T>							*
************************************************************************/
template <class T>
class Apply
{
  public:
    typedef void (*Func)(int, int, const Image<T>&, Image<T>&);
    
    Apply(const Image<float>& strengthOrg, Image<float>& strength, Func func)
	:_strengthOrg(strengthOrg), _strength(strength), _func(func)	{}
    
    void	operator ()(const tbb::blocked_range<size_t>& r) const
		{
		    for (auto v = r.begin(); v != r.end(); ++v)
			for (auto u = 0; u < _strength.width() - 1; ++u)
			    _func(u, v, _strengthOrg, _strength);
		}

  private:
    const Image<float>&	_strengthOrg;
    Image<float>&	_strength;
    Func		_func;
};
#endif
    
/************************************************************************
*   class MarkerDetector						*
************************************************************************/
//! 画像からキャリブレーションパターンの原点(の投影像)を検出する．
/*!
  \param image			入力画像
  \param cross			検出された原点(の投影像)の位置
  \param horizontal		パターンの向きが通常どおりならtrue,
				90deg.回転していればfalse
  \return			このマーカ点検出器
*/
const MarkerDetector&
MarkerDetector::operator ()(const Image<u_char>& image,
			    Point2f& cross, bool horizontal) const
{
  // 原点、Hタイプマーカおよび垂直マーカの候補点を探す。
    MarkerList	originCandidates, candidatesH, candidatesV;
    findMarkerCandidates(image, originCandidates,
			 candidatesH, candidatesV, horizontal);
    
  // 強度の強い順に原点の候補をとり，それを囲む４つの基準マーカ点を探す．
    for (MarkerList::const_iterator m  = originCandidates.begin();
				    m != originCandidates.end(); ++m)
    {
      // 原点に最も近いベースマーカ点対をそれぞれ横軸／縦軸上で求める．
	MarkerList::iterator	pH, qH, pV, qV;
	if (findBaseMarkers(*m, candidatesH, candidatesV, pH, qH, pV, qV))
	{
	    cross = *m;
	    
	    if (_params.edgeIntersection)
		refineMarkerLocationByEdgeIntersection(
		    createEdgeImage(image), cross);
	    else
		refineMarkerLocationByFittingQuadrics(
		    createSmoothedImage(image), cross);

	    return *this;
	}
    }

    throw std::runtime_error("MarkerDetector::operator (): the origin and surronding base markers not found!!");

    return *this;
}

//! 画像からキャリブレーションパターンの参照点の投影像(マーカ点)を検出する．
/*!
  \param image			入力画像
  \param corres			参照点と検出されたマーカ点の2次元座標対のリスト
  \param horizontal		パターンの向きが通常どおりならtrue,
				90deg.回転していればfalse
  \return			このマーカ点検出器
*/
const MarkerDetector&
MarkerDetector::operator ()(const Image<u_char>& image,
			    CorresList& corres, bool horizontal) const
{
  // 原点、HタイプマーカおよびVタイプマーカの候補点を探す。
    MarkerList	originCandidates, candidatesH, candidatesV;
    findMarkerCandidates(image, originCandidates,
			 candidatesH, candidatesV, horizontal);
    
  // 強度の強い順に原点の候補をとり，それを囲む４つの基準マーカ点を探す．
  // 歪曲中心が画像中央にある射影変換
    HomographyWithDistortion	H(Point2i({int(image.width() /2),
					   int(image.height()/2)}));
    auto	m = originCandidates.begin();
    for (; m != originCandidates.end(); ++m)
    {
	corres.clear();
	
      // 原点に最も近い基準マーカ点対をそれぞれ横軸／縦軸上で求める．
	MarkerList::iterator	pH, qH, pV, qV;
	if (findBaseMarkers(*m, candidatesH, candidatesV,  pH, qH, pV, qV))
	{
	  // 原点と４つの基準マーカ点から射影変換行列を計算する．
	    corres.push_back(make_pair(Point2f({ 0,  0}), *m));
	    corres.push_back(make_pair(Point2f({-1,  0}), *pH));
	    candidatesH.erase(pH);
	    corres.push_back(make_pair(Point2f({ 1,  0}), *qH));
	    candidatesH.erase(qH);
	    corres.push_back(make_pair(Point2f({ 0, -1}), *pV));
	    candidatesV.erase(pV);
	    corres.push_back(make_pair(Point2f({ 0,  1}), *qV));
	    candidatesV.erase(qV);
	    H.fit(corres.begin(), corres.end(), false);

	    auto	iter = corres.begin();
	    if ((H.square_distance(*iter++) < _params.sqdistTh) &&
		(H.square_distance(*iter++) < _params.sqdistTh) &&
		(H.square_distance(*iter++) < _params.sqdistTh) &&
		(H.square_distance(*iter)   < _params.sqdistTh))
		break;
	}
    }
    if (m == originCandidates.end())
	throw std::runtime_error("MarkerDetector::operator (): the origin and surronding base markers not found!!");

  // 基準マーカ点のまわりの４点のいずれかを出発点として他のマーカ点を探す．
    findMarkers( 2,  1, candidatesH, candidatesV, H, corres);
    findMarkers(-1,  2, candidatesH, candidatesV, H, corres);
    findMarkers(-2, -1, candidatesH, candidatesV, H, corres);
    findMarkers( 1, -2, candidatesH, candidatesV, H, corres);

  // マーカ点の位置をサブピクセル精度に補正する．
    if (_params.edgeIntersection)
    {
	const auto	edgeImage = createEdgeImage(image);
	
	for (auto p = corres.begin(); p != corres.end(); )
	    if (refineMarkerLocationByEdgeIntersection(edgeImage, p->second))
		++p;
	    else
		corres.erase(p++);
    }
    else
    {
	const auto	smoothedImage = createSmoothedImage(image);
	
	for (auto p = corres.begin(); p != corres.end(); )
	    if (refineMarkerLocationByFittingQuadrics(smoothedImage, p->second))
		++p;
	    else
		corres.erase(p++);
    }

    return *this;
}

//! キャリブレーションパターンの原点(の投影像)および参照点の投影像(マーカ点)の候補を検出する．
/*!
  \param image			入力画像
  \param originCandidates	検出された原点候補が返される
  \param candidatesH		検出されたHタイプマーカ点(左右の正方形が白色)候補
  \param candidatesV		検出されたVタイプマーカ点(上下の正方形が白色)候補
  \param horizontal		パターンの向きが通常どおりならtrue,
				90deg.回転していればfalse
*/
void
MarkerDetector::findMarkerCandidates(const Image<u_char>& image,
				     MarkerList& originCandidates,
				     MarkerList& candidatesH,
				     MarkerList& candidatesV,
				     bool horizontal) const
{
    using	std::less;
    using	std::greater;

  // 原点を検出するため，入力画像に十字テンプレートを適用．
    Image<float>	strengthOrg;
    IntegralImage<int>	integralImage(image);
    integralImage.crossVal(strengthOrg, _params.cropSize);
#ifdef _DEBUG
    strengthOrg *= 0.01;
    strengthOrg.save(std::cout);
#endif
    
  // 極大点／極小点を検出．
    auto		strength = strengthOrg;
#ifdef USE_TBB
    if (horizontal)
	tbb::parallel_for(tbb::blocked_range<size_t>(0,
						     strength.height() - 1, 1),
			  Apply<float>(strengthOrg, strength,
				       suppressNonMaximum<float>));
    else
	tbb::parallel_for(tbb::blocked_range<size_t>(0,
						     strength.height() - 1, 1),
			  Apply<float>(strengthOrg, strength,
				       suppressNonMinimum<float>));
#else
    for (size_t v = 0; v < strength.height() - 1; ++v)
    {
	if (horizontal)
	    for (size_t u = 0; u < strength.width() - 1; ++u)
		suppressNonMaximum(u, v, strengthOrg, strength);
	else
	    for (size_t u = 0; u < strength.width() - 1; ++u)
		suppressNonMinimum(u, v, strengthOrg, strength);
    }
#endif
    for (size_t v = 0; v < strength.height(); ++v)
	for (size_t u = 0; u < strength.width(); ++u)
	    if (strength[v][u] != 0)
		originCandidates.push_back(Marker(u, v, strength[v][u]));

  // マーカ点を検出するため，入力画像に×字テンプレートを適用．
    DiagonalIntegralImage<int>	diagonalIntegralImage(image);
    diagonalIntegralImage.crossVal(strengthOrg, _params.cropSize);
#ifdef _DEBUG
    strengthOrg *= 0.01;
    strengthOrg.save(std::cout);
#endif
    
  // 極大点を検出．
    strength = strengthOrg;
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<size_t>(0, strength.height() - 1, 1),
		      Apply<float>(strengthOrg, strength,
				   suppressNonMaximum<float>));
#else
    for (size_t v = 0; v < strength.height() - 1; ++v)
	for (size_t u = 0; u < strength.width() - 1; ++u)
	    suppressNonMaximum(u, v, strengthOrg, strength);
#endif
    for (size_t v = 0; v < strength.height(); ++v)
	for (size_t u = 0; u < strength.width(); ++u)
	    if (strength[v][u] != 0)
		candidatesH.push_back(Marker(u, v, strength[v][u]));

  // 極小点を検出．
    strength = strengthOrg;
#ifdef USE_TBB
    tbb::parallel_for(tbb::blocked_range<size_t>(0, strength.height() - 1, 1),
		      Apply<float>(strengthOrg, strength,
				   suppressNonMinimum<float>));
#else
    for (size_t v = 0; v < strength.height() - 1; ++v)
	for (size_t u = 0; u < strength.width() - 1; ++u)
	    suppressNonMinimum(u, v, strengthOrg, strength);
#endif
    for (size_t v = 0; v < strength.height(); ++v)
	for (size_t u = 0; u < strength.width(); ++u)
	    if (strength[v][u] != 0)
		candidatesV.push_back(Marker(u, v, strength[v][u]));
  //#ifdef _DEBUG
#if 0
    createImageWithMarkers(image, originCandidates, candidatesH, candidatesV)
	.save(std::cout);
#endif

  // 最大強度の一定割合に満たない弱い極大点を除去．
    MarkerList::const_iterator	m;
    if (horizontal)
    {
	m = max_element(originCandidates.begin(), originCandidates.end());
	originCandidates.erase(
	    std::remove_if(originCandidates.begin(), originCandidates.end(),
			   bind2nd(less<Marker>(),
						 Marker(0, 0,
							_params.strengthRatio *
							m->strength))),
	    originCandidates.end());
    }
    else
    {
	m = min_element(originCandidates.begin(), originCandidates.end());
	originCandidates.erase(
	    std::remove_if(originCandidates.begin(), originCandidates.end(),
			   bind2nd(greater<Marker>(),
				   Marker(0, 0,
					  _params.strengthRatio *
					  m->strength))),
	    originCandidates.end());
    }
    m = max_element(candidatesH.begin(), candidatesH.end());
    candidatesH.erase(
	std::remove_if(candidatesH.begin(), candidatesH.end(),
		       bind2nd(less<Marker>(),
			       Marker(0, 0,
				      _params.strengthRatio * m->strength))),
	candidatesH.end());
    m = min_element(candidatesV.begin(), candidatesV.end());
    candidatesV.erase(
	std::remove_if(candidatesV.begin(), candidatesV.end(),
		       bind2nd(greater<Marker>(),
			       Marker(0, 0,
				      _params.strengthRatio *
				      m->strength))),
	candidatesV.end());

  // 隣接点を除去してマーカ候補点を孤立化する．
#ifdef _DEBUG
    createImageWithMarkers(image, originCandidates, candidatesH, candidatesV)
	.save(std::cout);
#endif
    if (horizontal)
	originCandidates.sort(greater<Marker>());  // 正の強度の強い順にソート
    else
	originCandidates.sort(less<Marker>());	   // 負の強度の強い順にソート
}

//! マーカ点候補の中から原点に隣接する2つのHタイプまたはVタイプの基準マーカ点を選ぶ．
/*!
  \param origin		原点
  \param candidates	マーカ点候補
  \param horizontal	Hタイプ基準マーカ点を求めるならtrue, Vタイプならfalse
  \param p		原点の左／上にあるHタイプ／Vタイプ基準マーカ点を返す
  \param q		原点の右／下にあるHタイプ／Vタイプ基準マーカ点を返す
  \return		基準マーカ点がみつかればtrue, そうでなければfalse
*/
bool
MarkerDetector::findBaseMarkers(const Marker& origin,
				MarkerList& candidatesH,
				MarkerList& candidatesV,
				MarkerList::iterator& pH,
				MarkerList::iterator& qH,
				MarkerList::iterator& pV,
				MarkerList::iterator& qV) const
{
  // 原点に近い順にソート
    candidatesH.sort(NearerTo(origin));
    candidatesV.sort(NearerTo(origin));

    size_t	ntries = 0;
    for (auto cH  = candidatesH.begin(); cH != candidatesH.end(); ++cH)
    {
	if (++ntries > 10)
	    break;
	
	if (square_distance(*cH, origin) < _params.cropSize)	// 原点と近過ぎないか？
	    continue;	// 原点がマーカ点としても検出された場合に備える．

	pH = cH;
	const Point2f	rH(2.0*origin - *pH);	// 原点に関しpHと点対称な予測点
	qH = std::min_element(candidatesH.begin(), candidatesH.end(),
			      NearerTo(rH));
	if (square_distance(*qH, rH) > _params.sqdistTh) // 予測点から遠ければ…
	    continue;				// 別の候補点を探す．
	
	if ((*pH)[0] > (*qH)[0])	// pHはqHよりも左でなければならない
	    std::swap(pH, qH);

	for (auto cV  = candidatesV.begin(); cV != candidatesV.end(); ++cV)
	{
	  // 原点と近過ぎないか？	    
	    if (square_distance(*cV, origin) < _params.cropSize)
		continue;

	    pV = cV;
	    const Point2f	rV(2.0*origin - *pV);	// pVと点対称な予測点
	    qV = std::min_element(candidatesV.begin(), candidatesV.end(),
				  NearerTo(rV));

	  // 予測点から遠ければ…	    
	    if (square_distance(*qV, rV) > _params.sqdistTh)
		continue;				// 別の候補点を探す．

	  // pVはqVよりも上でなければならない
	    if ((*pV)[1] > (*qV)[1])
		std::swap(pV, qV);

	    if (consistentBaseMarkers(*pH, *qH, *pV, *qV))
		return true;
	}
    }

    return false;
}

//! 4つの基準マーカ点の位置関係が整合しているか調べる．
/*!
  \param pH		原点の左にあるHタイプ基準マーカ点
  \param qH		原点の右にあるHタイプ基準マーカ点
  \param pV		原点の上にあるVタイプ基準マーカ点
  \param qV		原点の下にあるVタイプ基準マーカ点
  \return		整合していればtrue, そうでなければfalse
*/
bool
MarkerDetector::consistentBaseMarkers(const Marker& pH, const Marker& qH,
				      const Marker& pV, const Marker& qV) const
{
    using namespace	std;
    
    const double	distH  = distance(pH, qH), distV = distance(pV, qV),
			cosHV  = ((pH - qH)*(pV - qV)) / (distH*distV);
#ifdef _DEBUG
    cerr << "distance ratio: " << max(distH/distV, distV/distH)
	 << ", theta: " << 180.0 / M_PI * acos(cosHV)
	 << endl;
#endif
    if (distH / distV > _params.distRatioTh ||
	distV / distH > _params.distRatioTh ||
	fabs(cosHV)   > _params.cosineTh)
	return false;
    return true;
}

//! 指定された起点から出発して隣接するマーカ点を再帰的に検出する．
/*!
  \param i, j		起点となる参照点の2次元座標
  \param candidatesH	Hタイプマーカ点候補
  \param candidatesV	Vタイプマーカ点候補
  \param H		参照点をマーカ点に写す変換．これまでに検出されたマーカ点から
			計算された値を与えると，新しく検出されたマーカ点を追加して更新
			された値が返される
  \param corres		参照点と検出されたマーカ点の対のリスト
*/
void
MarkerDetector::findMarkers(int i, int j,
			    MarkerList& candidatesH, MarkerList& candidatesV,
			    HomographyWithDistortion& H,
			    CorresList& corres) const
{
    auto&	candidates = (i % 2 ? candidatesH : candidatesV);
    const auto	r = H(Point2f({float(i), float(j)}));
    auto	p = std::min_element(candidates.begin(), candidates.end(),
				     NearerTo(r));
    if ((p != candidates.end()) && (square_distance(*p, r) < _params.sqdistTh))
    {
	corres.push_back(std::make_pair(Point2f({float(i), float(j)}), *p));
	candidates.erase(p);

      // 13点を越えたらレンズ歪み係数を含めた最適化を行う．
	H.fit(corres.begin(), corres.end(), corres.size() > 13);
	
	findMarkers(i+1, j+1, candidatesH, candidatesV, H, corres);
	findMarkers(i-1, j+1, candidatesH, candidatesV, H, corres);
	findMarkers(i-1, j-1, candidatesH, candidatesV, H, corres);
	findMarkers(i+1, j-1, candidatesH, candidatesV, H, corres);
    }
}

Image<u_char>
MarkerDetector::createEdgeImage(const Image<u_char>& image) const
{
    Image<float>		edgeH(image.width(), image.height()),
				edgeV(image.width(), image.height());
    DericheConvolver2<float>	convolver(_params.alpha);
    convolver.diffH(image.begin(), image.end(), edgeH.begin());
    convolver.diffV(image.begin(), image.end(), edgeV.begin());
    Image<float>	str;
    Image<u_char>	dir, edgeImage;
    EdgeDetector(_params.lowTh, _params.highTh)
	.strength(edgeH, edgeV, str)
	.direction4(edgeH, edgeV, dir)
	.suppressNonmaxima(str, dir, edgeImage)
	.hysteresisThresholding(edgeImage);

    return edgeImage;
}

Image<float>
MarkerDetector::createSmoothedImage(const Image<u_char>& image) const
{
    Image<float>		smoothedImage(image.width(), image.height());
    DericheConvolver2<float>	convolver(_params.alpha);
    convolver.smooth(image.begin(), image.end(), smoothedImage.begin());

    return smoothedImage;
}

//! パターンのエッジ画像への直線当てはめによりマーカ点の2次元座標を補正する．
/*!
  \param edgeImage	エッジ画像
  \param marker		マーカ点の2次元座標をピクセル精度で与えると，
			それらがサブピクセル精度に補正されて返される
  \return		補正に成功すればtrue, 失敗すればfalse
*/
bool
MarkerDetector::refineMarkerLocationByEdgeIntersection(
    const Image<u_char>& edgeImage, Point2f& marker) const
{
    using namespace	std;

  // マーカ候補点pを中心とする正方形ウィンドウを設定．
    const auto	u0	= int(marker[0]) - int(_params.winSize);
    const auto	v0	= int(marker[1]) - int(_params.winSize);
    const auto	winSize	= 2*_params.winSize + 1;
    if (u0 < 0 || u0 + winSize > edgeImage.width() || 
	v0 < 0 || v0 + winSize > edgeImage.height())
	return false;

    const auto	window = slice(edgeImage, v0, winSize, u0, winSize);

  // ウィンドウ内におけるエッジ点を検出．
    PointSet	edgePoints;
    for (size_t v = 0; v < size<0>(window); ++v)
	for (size_t u = 0; u < size<1>(window); ++u)
	    if (window[v][u])
		edgePoints.push_back(Point2f({float(u), float(v)}));
    
  // エッジ点を4つの象限に分類．
    auto	lowerLeft  = std::partition(edgePoints.begin(), edgePoints.end(),
					    [=](const Point2f& p)
					    { return p[1] < _params.winSize; });
    auto	upperRight = std::partition(edgePoints.begin(), lowerLeft,
					    [=](const Point2f& p)
					    { return p[0] < _params.winSize; });
    auto	lowerRight = std::partition(lowerLeft, edgePoints.end(),
					    [=](const Point2f& p)
					    { return p[0] < _params.winSize; });
    auto	lowerRightSize = std::distance(lowerRight, edgePoints.end());
    std::rotate(edgePoints.begin(), lowerRight, edgePoints.end());
    lowerRight = edgePoints.begin();
    auto	upperLeft = lowerRight + lowerRightSize;
    upperRight += lowerRightSize;
    lowerLeft  += lowerRightSize;

    if ((lowerRight != upperLeft) && (upperLeft != upperRight) &&
	(upperRight != lowerLeft) && (lowerLeft != edgePoints.end()))
    {
      // pの周りの4象限を点対称な2組に分け，それぞれからRANSACで直線を検出．
	const auto	conform = [=](const Point2f& p, const LineP2d& l)
				  { return l.square_distance(p)
				        < _params.sqconformTh; };
	Sampler		samplerP(lowerRight, upperLeft, upperRight);
	LineP2d		lineP;
	const auto	inliersP = ransac(samplerP, lineP, conform, 0.5);
	Sampler		samplerN(upperRight, lowerLeft, edgePoints.end());
	LineP2d		lineN;
	const auto	inliersN = ransac(samplerN, lineN, conform, 0.5);
#ifdef _DEBUG
	Image<RGB>	tmp = Y2RGB(window);
	for (const auto& point : edgePoints)
	    tmp(point) = RGB(255, 255, 0);
	for (const auto& point : inliersP)
	    tmp(point) = RGB(255, 0, 0);
	for (const auto& point : inliersN)
	    tmp(point) = RGB(0, 255, 0);
      //tmp.save(std::cout);
#endif
      // 2直線の交点を計算．
	Vector3d	delta = lineP ^ lineN;
	(delta[0] /= delta[2]) -= _params.winSize;
	(delta[1] /= delta[2]) -= _params.winSize;
	if (fabs(delta[0]) > 2.0 || fabs(delta[1]) > 2.0)
	{  // 修正量が2 pixelを越えればマーカ点でない。 
	   /* cerr << "correction at [" << marker[0] << ", " << marker[1]
	           << "] with more than 1 pixel!! ("
		   << delta[0] << ", " << delta[1] << ')' << endl;*/
	    return false;
	}
	else	// 修正量が2 pixel以内であれば修正する。
	    marker += delta(0, 2);
    }

    return true;
}

//! 輝度画像の局所ウィンドウに双曲放物面を当てはめることによりマーカ点の位置を補正する．
/*!
  \param smoothedImage	平滑化された輝度画像
  \param marker		マーカ点の2次元座標をピクセル精度で与えると，
			それらがサブピクセル精度に補正されて返される
  \return		補正に成功すればtrue, 失敗すればfalse
*/
bool
MarkerDetector::refineMarkerLocationByFittingQuadrics(
    const Image<float>& smoothedImage, Point2f& marker) const
{
  // マーカ候補点pを中心とする正方形ウィンドウを設定．
    const auto	u0	= int(marker[0]) - int(_params.winSize);
    const auto	v0	= int(marker[1]) - int(_params.winSize);
    const auto	winSize	= 2*_params.winSize + 1;
    if (u0 < 0 ||
	v0 < 0 ||
	u0 + winSize > smoothedImage.width() || 
	v0 + winSize > smoothedImage.height())
	return false;

    const auto	window = slice(smoothedImage, v0, winSize, u0, winSize);

  // 輝度分布に双曲放物面を当てはめ．
    std::vector<std::pair<Point2f, float> >	data;
    for (size_t v = 0; v < size<0>(window); ++v)
	for (size_t u = 0; u < size<1>(window); ++u)
	    data.push_back(make_pair(Point2f({float(u), float(v)}),
				     window[v][u]));
    QuadricSurface	quad(data.begin(), data.end());
	
  // 双曲放物面の中心を修正されたマーカ位置とし．修正量を求める．
    auto		delta = quad.center();
    delta[0] -= _params.winSize;
    delta[1] -= _params.winSize;
    if (fabs(delta[0]) > 2.0 || fabs(delta[1]) > 2.0)
    { // 修正量が2 pixelを越えればマーカ点でない．
      /*    cerr << "correction at [" << marker[0] << ", " << marker[1]
		 << "] with more than 1 pixel!! ("
		 << delta[0] << ", " << delta[1] << ')' << endl;*/
	return false;
    }
	
    marker += delta;	// 修正量が2 pixel以内であれば修正する。

    return true;
}

#ifdef _DEBUG
Image<RGB>
MarkerDetector::createImageWithMarkers(const Image<u_char>& image,
				       const MarkerList& originCandidates,
				       const MarkerList& candidatesH,
				       const MarkerList& candidatesV)
{
    Image<RGB>	tmp = Y2RGB(image);

    for (const auto& point : originCandidates)
	drawMarker(tmp, point, RGB(0, 255, 255));
    for (const auto& point : candidatesH)
	drawMarker(tmp, point, RGB(0, 255, 0));
    for (const auto& point : candidatesV)
	drawMarker(tmp, point, RGB(255, 0, 0));

    return tmp;
}
#endif

/************************************************************************
*  class MarkerDetector::Parameters					*
************************************************************************/
MarkerDetector::Parameters::Parameters()
    :cropSize(7),
     strengthRatio(0.5),
     sqdistTh(10.0*10.0),
     winSize(7),
     alpha(1.0),
     edgeIntersection(false),
     lowTh(2.0),
     highTh(5.0),
     sqconformTh(0.8*0.8),
     distRatioTh(2.0),
     cosineTh(0.7)
{
}
    
}
