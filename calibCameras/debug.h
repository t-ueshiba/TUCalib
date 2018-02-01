/*
 *  $Id: debug.h,v 1.1 2007-05-23 01:40:56 ueshiba Exp $
 */
#include "TU/Image++.h"

namespace TU
{
Image<RGB>	Y2RGB(const Image<u_char>& in)				;
    
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
print(std::ostream& out, Iterator first, Iterator last)
{
    using namespace	std;
    
    out << '{';
    for (Iterator iter = first; iter != last; )
    {
	out << '[' << (iter->first)[0] << ',' << (iter->first)[1] << ','
	    << iter->second << ']';
	if (++iter != last)
	    out << ',';
    }
    out << '}' << endl;
    return out;
}
 
}
