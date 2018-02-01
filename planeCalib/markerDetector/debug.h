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
  \file		debug.h
  \brief	デバッグ用コードの定義と実装
*/
#include "TU/Image++.h"
#include "TU/Geometry++.h"

namespace TU
{
Image<RGB>	Y2RGB(const Image<u_char>& in)				;

//! 画像に×印のマーカを付ける．
/*!
  \param image	マーカを付ける画像
  \param p	マーカの位置
  \param val	マーカの色
*/
template <class S, class T> static void
drawMarker(Image<S>& image, const Point2<T>& p, const S& val)
{
    const int	d = 5;
    const int	u = int(p[0] + 0.5), v = int(p[1] + 0.5);

    for (int uu = u - d, vv = v - d; uu <= u + d; ++uu, ++vv)
    {
	if (0 <= uu && uu < image.width() && 0 <= vv && vv < image.height())
	    image[vv][uu] = val;
    }
    for (int uu = u - d + 1, vv = v - d; uu <= u + d; ++uu, ++vv)
    {
	if (0 <= uu && uu < image.width() && 0 <= vv && vv < image.height())
	    image[vv][uu] = val;
    }
    for (int uu = u - d, vv = v - d + 1; uu <= u + d - 1; ++uu, ++vv)
    {
	if (0 <= uu && uu < image.width() && 0 <= vv && vv < image.height())
	    image[vv][uu] = val;
    }
    for (int uu = u - d, vv = v + d; uu <= u + d; ++uu, --vv)
    {
	if (0 <= uu && uu < image.width() && 0 <= vv && vv < image.height())
	    image[vv][uu] = val;
    }
    for (int uu = u - d, vv = v + d - 1; uu <= u + d - 1; ++uu, --vv)
    {
	if (0 <= uu && uu < image.width() && 0 <= vv && vv < image.height())
	    image[vv][uu] = val;
    }
    for (int uu = u - d + 1, vv = v + d; uu <= u + d; ++uu, --vv)
    {
	if (0 <= uu && uu < image.width() && 0 <= vv && vv < image.height())
	    image[vv][uu] = val;
    }
}

template <class T> static std::ostream&
print(std::ostream& out, const Matrix<T>& m)
{
    out << '[';
    for (int i = 0; i < m.nrow(); )
    {
	out << '[';
	for (int j = 0; j < m.ncol(); )
	{
	    out << m[i][j];
	    if (++j != m.ncol())
		out << ',';
	}
	if (++i != m.nrow())
	    out << "],";
	else
	    out << ']';
    }
    out << ']' << std::endl;
    return out;
}

template <class Iterator> static std::ostream&
print(std::ostream& out, Iterator begin, Iterator end)
{
    using namespace	std;
    
    out << '{';
    for (Iterator iter = begin; iter != end; )
    {
	out << '[' << (iter->first)[0] << ',' << (iter->first)[1] << ','
	    << iter->second << ']';
	if (++iter != end)
	    out << ',';
    }
    out << '}' << endl;
    return out;
}
 
}
