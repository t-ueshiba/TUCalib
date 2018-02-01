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
#include "TU/v/CanvasPane.h"
#include "TU/v/CanvasPaneDC.h"
#include "MarkerDetector.h"

namespace TU
{
namespace v
{
/**************u**********************************************************
*  class MyCanvasPane							*
************************************************************************/
class MyCanvasPane : public CanvasPane
{
  public:
    typedef MarkerDetector::CorresList	CorresList;
    
  public:
    MyCanvasPane(Window& parentWin, u_int width, u_int height,
		 const Image<u_char>& image, const CorresList& corres)	;

    CanvasPaneDC&	dc()						;
    
    void		resize(u_int w, u_int h)			;
    void		setZoom(float zoom)				;
    virtual void	repaintUnderlay()				;
    
    void		drawEpipolarLine(const Vector3i& p,
					 const Matrix33d& F)		;
    void		drawSelfEpipolarLine(const Vector3i& p,
					     const Matrix33d& Fh,
					     const Matrix33d& Fv)	;
    
  private:
    CanvasPaneDC		_dc;
    const Image<u_char>&	_image;
    const CorresList&		_corres;
};

inline CanvasPaneDC&
MyCanvasPane::dc()
{
    return _dc;
}
    
inline void
MyCanvasPane::resize(u_int w, u_int h)
{
    _dc.setSize(w, h, _dc.zoom());
}

inline void
MyCanvasPane::setZoom(float zoom)
{
    _dc.setZoom(zoom);
}

}
}
