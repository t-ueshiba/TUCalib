/*
 *  $Id: main.cc,v 1.1 2008-11-07 05:34:22 ueshiba Exp $
 */
#include <fstream>
#include <unistd.h>
#include "rectifyImages.h"
#include "IntensityCalibrator.h"
#include "TU/CorrectIntensity.h"
#ifdef UsePthread
#  include "TU/Thread++.h"
#endif

//! 本プログラムで定義されたクラスおよび関数を収める名前空間
namespace TU
{
/************************************************************************
*  static functions							*
************************************************************************/
//! コマンドの使い方を表示する．
static void
usage(const char* s)
{
    using namespace	std;

    cerr << "\nCalibrate intensity of the input images.\n"
	 << endl;
    cerr << " Usage: " << s << " [options] -p planes images0.pbm images1.ppm ... > offsetAndGains.dat\n"
	 << "        " << s << " -v [options] -p planes images0.pbm images1.ppm ... > verify.pbm\n"
	 << endl;
    cerr << " Options.\n"
	 << "  -p planes      input plane parameter file.\n"
	 << "  -r refview     reference view (default: 0).\n"
	 << "  -I inlierRate  rate of the consistent images point pairs (default: 0.4).\n"
	 << "  -C conformTh   conformance threshold (default: 3.0).\n"
	 << "  -v:            output intensity error images and mask images.\n"
	 << endl;
    cerr << " Other options.\n"
	 << "  -h:            print this.\n"
	 << endl;
}
 
template <class T> static void
restoreImages(std::istream& in, Array<Image<T> >& images)
{
    static int	n = 0;

    Image<T>	image;
    if (!image.restore(in))
	images.resize(n);
    else
    {
	++n;
	restoreImages(in, images);
	images[--n] = image;
    }
}

template <class T> static Image<float>
diff(const Image<T>& image0, const Image<T>& image1)
{
    Image<float>	diffImage(image0.width(), image0.height());
    for (int v = 0; v < diffImage.height(); ++v)
	for (int u = 0; u < diffImage.width(); ++u)
	    diffImage[v][u] = image0[v][u] - image1[v][u];
    return diffImage;
}
    
template <class T> static double
stdDev(const Image<T>& image0,
       const Image<T>& image1, const Image<u_char>& maskImage)
{
    double	sqr = 0.0;
    u_int	n = 0;
    for (int v = 0; v < maskImage.height(); ++v)
	for (int u = 0; u < maskImage.width(); ++u)
	    if (maskImage[v][u])
	    {
		double	diff = image0[v][u] - image1[v][u];
		sqr += (diff * diff);
		++n;
	    }
    return sqrt(sqr / n);
}
    
template <class T> static void
correctImage(Image<T>& image, const Image<T>& refImage,
	     const Vector2f& slant, const Image<u_char>& maskImage)
{
    using namespace	std;
    
    cerr << "  before correction: "
	 << stdDev(image, refImage, maskImage)
	 << endl;

    CorrectIntensity	correctIntensity(slant[0], slant[1]);
    correctIntensity(image);

    cerr << "  after correction:  "
	 << stdDev(image, refImage, maskImage)
	 << endl;
}

}
/************************************************************************
*  global functions							*
************************************************************************/
int
main(int argc, char* argv[])
{
    using namespace	std;
    using namespace	TU;

    typedef Array<Image<u_char> >	ImageArray;
    
  // Parse command options.
    const char*		planeFile = 0;
    u_int		refview = 0;
    double		inlierRate = 0.4, conformTh = 3.0;
    bool		verify = false;
    extern char*	optarg;
    for (int c; (c = getopt(argc, argv, "p:r:I:C:vh")) != EOF; )
	switch (c)
	{
	  case 'p':
	    planeFile = optarg;
	    break;
	  case 'r':
	    refview = atoi(optarg);
	    break;
	  case 'I':
	    inlierRate = atof(optarg);
	    break;
	  case 'C':
	    conformTh = atof(optarg);
	    break;
	  case 'v':
	    verify = true;
	    break;
	  case 'h':
	    usage(argv[0]);
	    return 1;
	}

    try
    {
	if (!planeFile)
	    throw runtime_error("Please specify a plane parameter file!!");
	ifstream	inPlane(planeFile);
	if (!inPlane)
	    throw runtime_error("Cannot open the plane parameter file!!");
	
      // Rectify the input images using plane parameters.
	extern int				optind;
	const int				nplanes = argc - optind;
	IntensityCalibrator			intensityCalib(inlierRate,
							       conformTh);
	Array<IntensityCalibrator::Slant>	slants;
	for (int j = 0; j < nplanes; ++j)
	{
	  // Restore observed images for each plane.
	    ifstream	in(argv[optind + j]);
	    if (!in)
		throw runtime_error("Failed to open the image file!!");
	    ImageArray	images;
	    restoreImages(in, images);

	  // Restore plane parameters.
	    Matrix33d	Qt;
	    inPlane >> Qt;

	  // Rectify plane images.
	    ImageArray	rectifiedImages = rectifyImages(images, Qt, refview);

	  // Correct intensities.
	    if (slants.dim() == 0)
		slants.resize(rectifiedImages.dim());
	    if (slants.dim() != rectifiedImages.dim())
		throw runtime_error("Inconsistent number of images between the planes!!");
	    for (int i = 0; i < rectifiedImages.dim(); ++i)
	    {
		if (i == refview)
		    continue;

	      // Calibrate intensity.
		Image<u_char>	maskImage;
		intensityCalib(rectifiedImages[i], rectifiedImages[refview],
			       slants[i], maskImage);

		if (verify)
		{
		    correctImage(rectifiedImages[i],
				 rectifiedImages[refview],
				 slants[i], maskImage);
		    
		    maskImage.save(cout, ImageBase::FLOAT);
		    diff(rectifiedImages[i],
			 rectifiedImages[refview]).save(cout, ImageBase::FLOAT);
		}
	    }
	}

	if (verify)
	    cerr << slants;
	else
	    cout << slants;
    }
    catch (exception& err)
    {
	cerr << err.what() << endl;
	return 1;
    }
    
    return 0;
}
