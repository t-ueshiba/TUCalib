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
 *  $Id: MyCanvasPane.h,v 1.5 2012-09-01 07:23:38 ueshiba Exp $
 */
#include "TU/v/CanvasPane.h"
#include "TU/v/CanvasPaneDC.h"
#include "TU/v/ShmDC.h"

namespace TU
{
namespace v
{
/************************************************************************
*  class MyCanvasPane							*
************************************************************************/
class MyCanvasPane : public CanvasPane
{
  public:
    MyCanvasPane(Window& parentWin, u_int width, u_int height,
		 const Image<u_char>& image, const Point2f& point)	;

    CanvasPaneDC&	dc()						;
    
    void		resize(u_int w, u_int h)			;
    void		setZoom(u_int mul, u_int div)			;
    virtual void	repaintUnderlay()				;
    
  private:
  //ShmDC			_dc;
    CanvasPaneDC		_dc;
    const Image<u_char>&	_image;
    const Point2f&		_cross;
};

inline CanvasPaneDC&
MyCanvasPane::dc()
{
    return _dc;
}
    
inline void
MyCanvasPane::resize(u_int w, u_int h)
{
    _dc.setSize(w, h, _dc.mul(), _dc.div());
}

inline void
MyCanvasPane::setZoom(u_int mul, u_int div)
{
    _dc.setZoom(mul, div);
}

}
}
