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
#ifndef __MARKERDETECTOR_H
#define __MARKERDETECTOR_H

/*!
  \file		MarkerDetector.h
  \brief	クラス#TU::MarkerDetectorの定義と実装
*/
#include <vector>
#include <utility>
#include <list>
#include <algorithm>
#include <iterator>
#include "TU/Image++.h"
#include "ProjectivityWithDistortion.h"

namespace TU
{
/************************************************************************
*  class MarkerDetector							*
************************************************************************/
//! キャリブレーションパターン上の参照点の投影像(マーカ点)を検出するクラス
class MarkerDetector
{
  public:
  //! 参照平面上の参照点とその投影像(マーカ点)の2次元座標対
    using Corres		= std::pair<Point2f, Point2f>;
  //! 1枚の参照平面と1台のカメラ間で対応づけられた全ての#Corresから成るリスト
    using CorresList		= std::list<Corres>;
  //! 1枚の参照平面に対して，全カメラの#CorresListを収めた配列
    using CorresListArray	= Array<CorresList>;
  //! 全参照平面の#CorresListArrayを収めたリスト
    using CorresListArrayList	= std::list<CorresListArray>;
    
  //! マーカ点の検出パラメータ
    struct Parameters
    {
	Parameters()	;
	
      //! マーカ点を検出するためのテンプレートサイズ
      /*!
	一辺2*cropSize + 1の正方形テンプレート(十字印と×印)が各画像点に適用される．
      */
	size_t	cropSize;

      //! テンプレート反応強度の最大値に対する極大値の割合を表す0と1の間の値
      /*!
	反応強度が極大かつ最大値に対する割合がこの値を越える点のみがマーカ点候補
	として残される．
      */
	float	strengthRatio;

      //! マーカ点の予測位置と実際の検出位置の二乗距離の閾値
      /*!
	参照点に変換 #TU::HomographyWithDistortion を適用した予測位置と実際の
	検出位置の二乗距離がこの値以内ならば，検出された点を正しいマーカ点と見做す．
      */
	double	sqdistTh;

      //! サブピクセル位置決めにおける双曲放物面／2直線当てはめのウィンドウサイズ
	size_t	winSize;

      //! エッジ点検出または平滑化のための正のスムーシング係数
      /*!
	小さいほど平滑化の度合いが大きい.
      */
	float	alpha;

      //! サブピクセル位置決めの手法
      /*!
	エッジ点に当てはめた2直線の交点計算によって行うのであればtrue,
	輝度パターンへの双曲放物面当てはめによって行うのであればfalse
      */
	bool	edgeIntersection;
	
      //! エッジ点検出のためのヒステリシス閾値処理における低い閾値
      /*!
	edgeIntersection = true の場合のみ有効
      */
	float	lowTh;

      //! エッジ点検出のためのヒステリシス閾値処理における高い閾値
      /*!
	edgeIntersection = true の場合のみ有効
      */
	float	highTh;

      //! RANSACにおけるエッジ点への直線当てはめの二乗距離の閾値
      /*!
	edgeIntersection = true の場合のみ有効
      */
	double	sqconformTh;

      //! 原点検出の正しさを判定する際のHタイプ／Vタイプ基準マーカ点間の距離の比の閾値
	double	distRatioTh;

      //! 原点検出の正しさを判定する際のHタイプ／Vタイプ基準マーカ点間を結ぶ2直線が成す角度のcosine値の閾値．
	double	cosineTh;
    };

  private:
  //! マーカ点(パターンテンプレートへの反応強度付き)
    struct Marker : public Point2f
    {
      //! 2次元座標とテンプレートへの反応強度を与えてマーカ点を生成する．
	Marker(float u, float v, float str) :Point2f({u, v}), strength(str){}

	float	strength;		//!< パターンテンプレートに対する反応強度
    };

  //! マーカ点のリスト
    using MarkerList	= std::list<Marker>;

  //! ある基準点を固定し，与えられた2点のどちらがそれにより近いかを判定する関数オブジェクト
    struct NearerTo
    {
      //! 基準点を設定する．
	NearerTo(const Point2f& p) :_p(p)		{}

      //! 与えられた2点のどちらが基準点により近いかを判定する．
	bool	operator ()(const Point2f& a, const Point2f& b) const
		{
		    return square_distance(_p, a) < square_distance(_p, b);
		}

      private:
	const Point2f	_p;		//!< 基準点
    };

  //! RANSACで直線を検出する対象となる2次元点の集合
    using PointSet	= std::vector<Point2f>;
    using piterator	= PointSet::const_iterator;
    
    class Sampler
    {
      public:
	Sampler(piterator begin, piterator middle, piterator end)
	    :_begin(begin), _middle(middle), _end(end)	{}
	
	auto	begin()		const	{return _begin;}
	auto	end()		const	{return _end;}
	auto	size()		const	{ return std::distance(_begin, _end); }
	template <class OUT_, class GEN_>
	void	operator ()(OUT_ out, size_t, GEN_&& gen) const
		{
		    std::sample(_begin, _middle, out,
				1, std::forward<GEN_>(gen));
		    std::sample(_middle, _end, out, 1,
				std::forward<GEN_>(gen));
		}
    
      private:
	const piterator	_begin;		//!< この集合中の最初の点を指す反復子
	const piterator	_middle;
	const piterator	_end;		//!< この集合中の末尾の次の点を指す反復子
    };

    using HomographyWithDistortion = ProjectivityWithDistortion<double, 2, 2>;
    
  public:
    MarkerDetector()							;
    MarkerDetector(const Parameters& params)				;

    MarkerDetector&	setParameters(const Parameters& params)		;
    const Parameters&	getParameters()				const	;

    const MarkerDetector&
		operator ()(const Image<u_char>& image,
			    Point2f& cross,
			    bool horizontal=true)		const	;
    const MarkerDetector&
		operator ()(const Image<u_char>& image,
			    CorresList& corres,
			    bool horizontal=true)		const	;

    friend bool	operator <(const Marker& a, const Marker& b)		;
    friend bool	operator >(const Marker& a, const Marker& b)		;
    friend std::ostream&
		operator <<(std::ostream& out, const Marker& marker)	;

  private:
    void	findMarkerCandidates(const Image<u_char>& image,
				     MarkerList& originCandidates,
				     MarkerList& candidatesH,
				     MarkerList& candidatesV,
				     bool horizontal)		const	;
    bool	findBaseMarkers(const Marker& origin,
				MarkerList& candidatesH,
				MarkerList& candidatesV,
				MarkerList::iterator& pH,
				MarkerList::iterator& qH,
				MarkerList::iterator& pV,
				MarkerList::iterator& qV)	const	;
    bool	consistentBaseMarkers(const Marker& pH,
				      const Marker& qH,
				      const Marker& pV,
				      const Marker& qV)		const	;
    void	findMarkers(int i, int j,
			    MarkerList& candidatesH,
			    MarkerList& candidatesV,
			    HomographyWithDistortion& H,
			    CorresList& corres)			const	;
    Image<u_char>
		createEdgeImage(const Image<u_char>& image)	const	;
    Image<float>
		createSmoothedImage(const Image<u_char>& image)	const	;
    bool	refineMarkerLocationByEdgeIntersection(
		    const Image<u_char>& edgeImage,
		    Point2f& marker)				const	;
    bool	refineMarkerLocationByFittingQuadrics(
		    const Image<float>& smoothedImage,
		    Point2f& marker)				const	;
    static Image<RGB>
		createImageWithMarkers(
		    const Image<u_char>& image,
		    const MarkerList& originCandidates,
		    const MarkerList& candidatesH,
		    const MarkerList& candidatesV)			;

  private:
    Parameters	_params;
};

//! マーカ検出器を生成し，デフォルトのパラメータを設定する．
inline
MarkerDetector::MarkerDetector()
    :_params()
{
}
    
//! マーカ検出器を生成し，指定されたデフォルトのパラメータを設定する．
/*!
  \param params	マーカ検出器のパラメータ
*/
inline
MarkerDetector::MarkerDetector(const Parameters& params)
    :_params(params)
{
}
    
//! マーカ検出器のパラメータを設定する．
/*!
  \param params	マーカ検出器のパラメータ
  \return	このマーカ検出器
*/
inline MarkerDetector&
MarkerDetector::setParameters(const Parameters& params)
{
    _params = params;
    return *this;
}
    
//! マーカ検出器に設定されているパラメータを返す．
/*!
  \return	マーカ検出器に設定されているパラメータ
*/
inline const MarkerDetector::Parameters&
MarkerDetector::getParameters() const
{
    return _params;
}

//! 2つのマーカ点のテンプレートへの適合度を比較する．
inline bool
operator <(const MarkerDetector::Marker& a,
	   const MarkerDetector::Marker& b)
{
    return a.strength < b.strength;
}

//! 2つのマーカ点のテンプレートへの適合度を比較する．
inline bool
operator >(const MarkerDetector::Marker& a,
	   const MarkerDetector::Marker& b)
{
    return b < a;
}

//! 出力ストリームへマーカ点の2次元座標を書き出す(ASCII)．
inline std::ostream&
operator <<(std::ostream& out, const MarkerDetector::Marker& marker)
{
    return out << marker.strength
	       << '(' << marker[0] << ", " << marker[1] << ')';
}

//! 入力ストリームから参照点とその投影像(マーカ点)の組を読み込む(ASCII)．
inline std::istream&
operator >>(std::istream& in, MarkerDetector::Corres& corres)
{
    return in >> corres.first >> corres.second;
}
    
//! 出力ストリームへ参照点とその投影像(マーカ点)の組を書き出す(ASCII)．
inline std::ostream&
operator <<(std::ostream& out, const MarkerDetector::Corres& corres)
{
    return corres.second.put(corres.first.put(out));
}

//! 入力ストリームからリストにその要素を読み込む(ASCII)．
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

//! 出力ストリームへリスト中の全要素を書き出す(ASCII)．
template <class T> inline std::ostream&
operator <<(std::ostream& out, const std::list<T>& list)
{
    using namespace	std;
    
    copy(list.begin(), list.end(), ostream_iterator<T>(out));
    return out << endl;
}

}
#endif	//! __MARKERDETECTOR_H
