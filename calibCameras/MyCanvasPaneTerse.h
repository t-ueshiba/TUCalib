/*
 *  $Id: MyCanvasPaneTerse.h,v 1.2 2007-03-08 05:11:09 ueshiba Exp $
 */
#include "MyCanvasPane.h"

namespace TU
{
namespace v
{
/************************************************************************
*  class MyCanvasPaneTerse						*
************************************************************************/
class MyCanvasPaneTerse : public MyCanvasPane
{
  public:
    MyCanvasPaneTerse(Window& parentWin, u_int width, u_int height,
		      const Image<u_char>& image, const PairList& pairs)
	:MyCanvasPane(parentWin, width, height, image, pairs)		{}

    virtual void	callback(CmdId, CmdVal)				;
};
    
}
}

