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
  \file		ProjectivityWithDistortion.h
  \brief	クラス#TU::ProjectivityWithDistortionの定義と実装
*/
#include "TU/Geometry++.h"

namespace TU
{
/************************************************************************
*  class ProjectivityWithDistortion<M>					*
************************************************************************/
//! 放射歪曲を伴う射影変換を表すクラス
/*!
  (m+1)x(n+1)行列T, 放射歪曲の中心u_0および放射歪曲係数dを用いてm次元空間の点xを
  \f[
  \TUbreve{u}{} = \TUvec{u}{0} + (1 + d r^2)(\TUvec{u}{} - \TUvec{u}{0}),~
  \TUbeginarray{c} \TUvec{u}{} \\ 1 \TUendarray \simeq
  \TUvec{T}{}
  \TUbeginarray{c} \TUvec{x}{} \\ 1 \TUendarray,~
  r = \TUnorm{\TUvec{u}{} - \TUvec{u}{0}}
  \f]
  に写す(m != nでも構わない)．
*/
template <class T, size_t DO=0, size_t DI=0>
class ProjectivityWithDistortion : public Projectivity<T, DO, DI>
{
  private:
    using super	= Projectivity<T, DO, DI>;
    
  public:
    constexpr static size_t	NPARAMS = super::NPARAMS+1;
    constexpr static size_t	DOF	= (super::NPARAMS == 0 ? 0 : NPARAMS-1);
    
    using			typename super::element_type;
    using			typename super::point_type;
    using			typename super::ppoint_type;
    using			typename super::vector_type;
    using derivative_type	= Array2<element_type, DO, NPARAMS>;
    
  //! 放射歪曲の中心を与えて変換オブジェクトを生成する．
  /*!
    \param u0	放射歪曲の中心
  */
    template <class T_, size_t D_>
    ProjectivityWithDistortion(const Array<T_, D_>& u0)
	:super(), _d(0.0), _u0(u0)					{}

    using		super::inDim;
    using		super::outDim;
    using		super::ndataMin;
    
    size_t		nparams()				const	;
    template <class ITER_>
    void		fit(ITER_ begin, ITER_ end, bool refine=true)	;
    template <class T_, size_t D_>
    point_type		operator ()(const Array<T_, D_>& x)	const	;
    template <class T_, size_t D_>
    ppoint_type		mapP(const Array<T_, D_>& x)		const	;
    template <class T_, size_t D_>
    derivative_type	derivative(const Array<T_, D_>& x)	const	;
    void		update(const vector_type& dt)			;
    
  private:
    element_type	_d;	//!< 放射歪曲係数
    point_type		_u0;	//!< 放射歪曲中心
};

//! この変換の自由度を返す．
/*!
  \return	変換の自由度（1 + #TU::Projectivity<T, DO, DI>::nparams()）
*/
template <class T, size_t DO, size_t DI> inline size_t
ProjectivityWithDistortion<T, DO, DI>::nparams() const
{
    return 1 + super::nparams();
}

//! 与えられた点対列の非同次座標から変換を計算する．
/*!
  \param begin		点対列の先頭を示す反復子
  \param end		点対列の末尾を示す反復子
  \param refine		非線形最適化の有(true)／無(false)を指定
*/
template <class T, size_t DO, size_t DI> template <class ITER_> inline void
ProjectivityWithDistortion<T, DO, DI>::fit(ITER_ begin, ITER_ end, bool refine)
{
    using namespace	std;

    super::fit(begin, end, false);

    _d = 0.0;		// initial value of non-linear refinement
    if (refine)
    {
	typename super::template Cost<ProjectivityWithDistortion, ITER_>
							cost(begin, end);
	ConstNormConstraint<ProjectivityWithDistortion>	constraint(*this);
	minimizeSquare(cost, constraint, *this);
    }
}

//! 与えられた点に変換を適用してその非同次座標を返す．
/*!
  \param x	点の非同次座標（#inDim()次元）または同次座標（#inDim()+1次元）
  \return	変換された点の非同次座標（#outDim()次元）
*/
template <class T, size_t DO, size_t DI> template <class T_, size_t D_>
inline auto
ProjectivityWithDistortion<T, DO, DI>::operator ()(const Array<T_, D_>& x) const
    -> point_type
{
    const point_type	du = super::operator ()(x) - _u0;
    return _u0 + (1.0 + _d * square(du)) * du;
}

//! 与えられた点に変換を適用してその同次座標を返す．
/*!
  \param x	点の非同次座標（#inDim()次元）または同次座標（#inDim()+1次元）
  \return	変換された点の同次座標（#outDim()+1次元）
*/
template <class T, size_t DO, size_t DI> template <class T_, size_t D_>
inline auto
ProjectivityWithDistortion<T, DO, DI>::mapP(const Array<T_, D_>& x) const
    -> ppoint_type
{
    ppoint_type	u(outDim()+1);
    slice(u, 0, outDim()) = (*this)(x);
    u[outDim()]		  = 1;
    return u;
}

//! 与えられた点におけるヤコビ行列を返す．
/*!
  ヤコビ行列とは射影変換行列成分に関する1階微分のことである．
  \param x	点の非同次座標（#inDim()次元）または同次座標（#inDim()+1次元）
  \return	#outDim()x#nparams()ヤコビ行列
*/
template <class T, size_t DO, size_t DI> template <class T_, size_t D_>
inline auto
ProjectivityWithDistortion<T, DO, DI>::derivative(const Array<T_, D_>& x) const
    -> derivative_type
{
    const point_type	du = super::operator ()(x) - _u0;
    const auto		r2 = square(du);
    derivative_type	J(outDim(), nparams());

  // _dに関する微分
    for (size_t i = 0; i < J.nrow(); ++i)
	J[i][0] = r2 * du[i];

  // 射影変換行列に関する微分
    const element_type	tmp = 1.0 + _d * square(du);
    auto		K   = evaluate((2.0 * _d * du) % du);
    for (size_t i = 0; i < K.nrow(); ++i)
	K[i][i] += tmp;
    slice(J, 0, outDim(), 1, J.ncol() - 1) = K * super::derivative(x);
    
    return J;
}

//! 変換を与えられた量だけ修正する．
/*!
  \param dm	修正量を表すベクトル（#nparams()次元）
*/
template <class T, size_t DO, size_t DI> inline void
ProjectivityWithDistortion<T, DO, DI>::update(const vector_type& dm)
{
    _d -= dm[0];
    super::update(slice(dm, 1, dm.size()-1));
}

//! 射影変換行列の各行を順番に並べたベクトルを返す．
/*!
  \return	射影変換行列の成分を並べたベクトル（#nparams()次元）
*/
template <class T, size_t DO, size_t DI>
inline typename Projectivity<T, DO, DI>::vector_type
serialize(const ProjectivityWithDistortion<T, DO, DI>& P)
{
    using super		= Projectivity<T, DO, DI>;
    using vector_type	= typename super::vector_type;
    
    vector_type	t(P.nparams());
    slice(t, 1, t.size()-1) = serialize(static_cast<const super&>(P));
    return t;
}
    
    
}
