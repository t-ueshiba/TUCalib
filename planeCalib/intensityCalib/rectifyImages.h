/*
 *  $Id: rectifyImages.h,v 1.1 2008-11-07 05:34:22 ueshiba Exp $
 */
#include "TU/Image++.h"
#include "TU/Warp.h"

namespace TU
{
template <class T> Array<Image<T> >
rectifyImages(const Array<Image<T> >& images,
	      const Matrix33d& Qt, u_int refview)
{
  // Set camera parameters for each image.
    Array<CameraWithDistortion>	cameras(images.dim());
    for (int i = 0; i < cameras.dim(); ++i)
	cameras[i].setProjection(images[i].P)
		  .setDistortion(images[i].d1, images[i].d2);
    
  // Rectify images.
    const Matrix33d&	Kreftinv = cameras[refview].Ktinv();
    const Matrix33d&	Rtref	 = cameras[refview].Rt();
    const Vector3d&	tref	 = cameras[refview].t();
    Vector3d		r	 = Qt[0] ^ Qt[1];
    r /= (r * (Qt[2] - tref));
    const Image<T>&	refImage = images[refview];
    Array<Image<T> >	rectifiedImages(images.dim());
    for (int i = 0; i < images.dim(); ++i)
    {
	const Matrix<double>&
		Tt = Matrix<double>::I(3) - r % (cameras[i].t() - tref);
	Warp	warp;
	warp.initialize(Kreftinv * Rtref * Tt *
			cameras[i].Rt().trns() * cameras[i].Kt(),
			cameras[i].intrinsic(),
			refImage.width(), refImage.height(),
			refImage.width(), refImage.height());
	warp(images[i], rectifiedImages[i]);
    }

    return rectifiedImages;
}

}
