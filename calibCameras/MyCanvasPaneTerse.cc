/*
 *  $Id: MyCanvasPaneTerse.cc,v 1.1.1.1 2006-06-02 06:11:17 ueshiba Exp $
 */
#include "MyCanvasPaneTerse.h"

namespace TU
{
namespace v
{
/************************************************************************
*  class MyCanvasPaneTerse						*
************************************************************************/
void
MyCanvasPaneTerse::callback(CmdId id, CmdVal val)
{
    switch (id)
    {
      case Id_MouseButton1Press:
      case Id_MouseButton1Drag:
      case Id_MouseButton1Release:
	break;
      default:
	parent().callback(id, val);
	break;
    }
}
    
}
}
