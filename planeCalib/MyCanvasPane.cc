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
			   const Image<u_char>& image,
			   const CorresList& corres)
    :CanvasPane(parentWin, width, height),
     _dc(*this, width, height), _image(image), _corres(corres)
{
}
    
void
MyCanvasPane::repaintUnderlay()
{
    _dc << _image << cross;

    int	n = 0;
    for (CorresList::const_iterator iter  = _corres.begin();
				    iter != _corres.end(); ++iter)
    {
	switch (n++)
	{
#ifdef UseOverlay
	  case 0:
	    _dc << foreground(0);
	    break;
	  case 1:
	  case 2:
	  case 3:
	  case 4:
	    _dc << foreground(1);
	    break;
	  default:
	    _dc << foreground(2);
	    break;
#else
	  case 0:
	    _dc << foreground(BGR(0, 255, 255));
	    break;
	  case 1:
	  case 2:
	  case 3:
	  case 4:
	    _dc << foreground(BGR(255, 0, 255));
	    break;
	  default:
	    _dc << foreground(BGR(255, 255, 0));
	    break;
#endif
	}
	_dc << iter->second;
      /*std::ostringstream	s;
	s << '(' << iter->first[0] << ',' << iter->first[1] << ')';
	_dc.draw(s.str().c_str(), iter->second[0], iter->second[1]);*/
    }
}

void
MyCanvasPane::drawEpipolarLine(const Vector3i& p, const Matrix33d& F)
{
    LineP2d	l = F * p;
    if (square(l) != 0.0)
	_dc << l;
}

void
MyCanvasPane::drawSelfEpipolarLine(const Vector3i& p,
				   const Matrix33d& Fh, const Matrix33d& Fv)
{
    LineP2d	l = (Fh[0] ^ Fh[1]) ^ p;
    if (square(l) != 0.0)
	_dc << l;
    l = (Fv[0] ^ Fv[1]) ^ p;
    if (square(l) != 0.0)
	_dc << l;
}

}
}
