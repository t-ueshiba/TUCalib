/*
 *  $Id: MyCanvasPane.cc,v 1.3 2007-05-23 01:40:56 ueshiba Exp $
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
			   const PairList& pairs)
    :CanvasPane(parentWin, width, height),
     _dc(*this, width, height), _image(image), _pairs(pairs)
{
}
    
void
MyCanvasPane::repaintUnderlay(int, int, int, int)
{
    display();
}

void
MyCanvasPane::display()
{
    _dc << _image << cross;

    int	n = 0;
    for (PairList::const_iterator iter = _pairs.begin(); iter != _pairs.end();
	 ++iter)
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
MyCanvasPane::drawEpipolarLine(const Vector3i& p, const Matrix<double>& F)
{
    LineP2<double>	l = F * p;
    if (l.square() != 0.0)
	_dc << l;
}

void
MyCanvasPane::drawSelfEpipolarLine(const Vector3i& p,
				   const Matrix<double>& Fh,
				   const Matrix<double>& Fv)
{
    LineP2<double>	l = (Fh[0] ^ Fh[1]) ^ p;
    if (l.square() != 0.0)
	_dc << l;
    l = (Fv[0] ^ Fv[1]) ^ p;
    if (l.square() != 0.0)
	_dc << l;
}

}
}
