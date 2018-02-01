/*
 *  $Id: debug.cc,v 1.1 2007-05-23 01:40:56 ueshiba Exp $
 */
#include "debug.h"
#if defined(__GNUG__) || defined(__INTEL_COMPILER)
#  include "TU/Image++.cc"
#endif

namespace TU
{
Image<RGB>
Y2RGB(const Image<u_char>& in)
{
    Image<RGB>	out(in.width(), in.height());
    for (int v = 0; v < out.height(); ++v)
	out[v].fill(&in[v][0]);
    return out;
}
    
}
