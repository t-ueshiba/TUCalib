/*
 *  $Id: MyCanvasPane.h,v 1.2 2007-03-08 05:11:09 ueshiba Exp $
 */
#include "TU/v/CanvasPane.h"
#include "TU/v/CanvasPaneDC.h"
#include "TU/v/ShmDC.h"
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
    MyCanvasPane(Window& parentWin, u_int width, u_int height,
		 const Image<u_char>& image, const PairList& pairs)	;

    CanvasPaneDC&	dc()						;
    
    void		display()					;
    void		resize(u_int w, u_int h)			;
    void		setZoom(u_int mul, u_int div)			;
    virtual void	repaintUnderlay(int x, int y, int w, int h)	;
    
    void		drawEpipolarLine(const Vector3i& p,
					 const Matrix<double>& F)	;
    void		drawSelfEpipolarLine(const Vector3i& p,
					     const Matrix<double>& Fh,
					     const Matrix<double>& Fv)	;
    
  private:
    ShmDC			_dc;
    const Image<u_char>&	_image;
    const PairList&		_pairs;
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
