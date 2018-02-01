/*
 *  平成14-19年（独）産業技術総合研究所 著作権所有
 *  
 *  創作者：植芝俊夫
 *
 *  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
 *  （独）産業技術総合研究所が著作権を所有する秘密情報です．創作者によ
 *  る許可なしに本プログラムを使用，複製，改変，第三者へ開示する等の著
 *  作権を侵害する行為を禁止します．
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
 *  without permission by the creator are strictly prohibited.
 *
 *  [No Warranty.]
 *  The copyright holders or the creator are not responsible for any
 *  damages in the use of this program.
 *  
 *  $Id: MyCanvasPane.cc,v 1.4 2012-09-01 07:23:38 ueshiba Exp $
 */
#include "MyCanvasPane.h"
#include <sstream>

namespace TU
{
namespace v
{
/************************************************************************
*  class MyCanvasPane							*
************************************************************************/
MyCanvasPane::MyCanvasPane(Window& parentWin, u_int width, u_int height,
			   const Image<u_char>& image, const Point2f& point)
    :CanvasPane(parentWin, width, height),
     _dc(*this, width, height), _image(image), _cross(point)
{
    _dc << cross;
}
    
void
MyCanvasPane::repaintUnderlay()
{
    _dc << _image << foreground(BGR(255, 0, 0)) << _cross;
}

}
}
