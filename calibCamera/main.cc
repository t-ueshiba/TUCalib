/*
 *  $Id: main.cc,v 1.1.1.1 2007-05-23 05:55:43 ueshiba Exp $
 */
#include <unistd.h>
#include <stdlib.h>
#include <iomanip>
#include <stdexcept>
#include "MyCmdWindow.h"

#define	DEFAULT_CALIB_FILE	"IEEE1394Camera.calib"
static const double		DEFAULT_PITCH		= 30.0;
static const u_int		DEFAULT_CROP_SIZE	= 10;
static const float		DEFAULT_STRENGTH_TH	=  0.5,
				DEFAULT_DIST_TH		=  7.0,
				DEFAULT_ALPHA		=  1.0,
				DEFAULT_TH_LOW		=  2.0,
				DEFAULT_TH_HIGH		=  5.0;

namespace TU
{
v::CmdDef*	createMenuCmds(Ieee1394Camera& camera)		;
v::CmdDef*	createCaptureCmds()				;
v::CmdDef*	createFeatureCmds(const Ieee1394Camera& camera)	;

/************************************************************************
*  static functions							*
************************************************************************/
static void
usage(const char* s)
{
    using namespace	std;

    cerr << "\nCalibrate an IEEE1394-based camera with a planar pattern.\n"
	 << endl;
    cerr << " Usage: " << s << " [-b] [-p pitch] [-c cameraBase]\n"
	 << endl;
    cerr << " General options.\n"
	 << "  -b:             IEEE1394b mode.        (default: off)\n"
	 << "  -p pitch:       pitch of grid pattern. (default: "
	 << DEFAULT_PITCH << ")\n"
	 << "  -c calibFile:   file name for storing a calibration result.\n"
	 << "                    (default: "
	 << DEFAULT_CALIB_FILE
	 << ")\n"
	 << endl;
    cerr << " Options for marker detection.\n"
	 << "  -C cropSize:    size of cropping square for template matching.\n"
	 << "                    (default: "
	 << DEFAULT_CROP_SIZE
	 << ")\n"
	 << "  -S strengthTh:  threshold for template matcing in ratio to "
	 << "the highest score.\n"
	 << "                    (default: "
	 << DEFAULT_STRENGTH_TH
	 << ")\n"
	 << "  -D distTh:      distance threshold for projective "
	 << "transformation.\n"
	 << "                    (default: "
	 << DEFAULT_DIST_TH
	 << ")\n"
	 << "  -A alpha:       size of Deriche edge detector for refining "
	 << "marker locations.\n"
	 << "                    (default: "
	 << DEFAULT_ALPHA
	 << ")\n"
	 << "  -L th_low:      lower threshold for edge detection.\n"
	 << "                    (default: "
	 << DEFAULT_TH_LOW
	 << ")\n"
	 << "  -H th_high:     higher threshold for edge detection.\n"
	 << "                    (default: "
	 << DEFAULT_TH_HIGH
	 << ")\n"
	 << endl;
    cerr << " Other options.\n"
	 << "  -h:             print this.\n"
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
    
    v::App		vapp(argc, argv);
    bool		i1394b	   = false;
    double		pitch	   = DEFAULT_PITCH;
    const char*		calibFile  = DEFAULT_CALIB_FILE;
    u_int		cropSize   = DEFAULT_CROP_SIZE;
    float		strengthTh = DEFAULT_STRENGTH_TH,
			distTh	   = DEFAULT_DIST_TH,
			alpha	   = DEFAULT_ALPHA,
			th_low	   = DEFAULT_TH_LOW,
			th_high	   = DEFAULT_TH_HIGH;

  // Parse command options.
    extern char*	optarg;
    for (int c; (c = getopt(argc, argv, "bp:c:C:S:D:A:L:H:h")) != EOF; )
	switch (c)
	{
	  case 'b':
	    i1394b = true;
	    break;
	  case 'p':
	    pitch = atof(optarg);
	    break;
	  case 'c':
	    calibFile = optarg;
	    break;
	  case 'C':
	    cropSize = atoi(optarg);
	    break;
	  case 'S':
	    strengthTh = atof(optarg);
	    break;
	  case 'D':
	    distTh = atof(optarg);
	    break;
	  case 'A':
	    alpha = atof(optarg);
	    break;
	  case 'L':
	    th_low = atof(optarg);
	    break;
	  case 'H':
	    th_high = atof(optarg);
	    break;
	  case 'h':
	    usage(argv[0]);
	    return 1;
	}
    extern int	optind;
    u_int64	uniqId = 0;
    if (optind < argc)
	uniqId = strtoull(argv[optind], 0, 0);
    
  // Main job.
    try
    {
	Ieee1394Camera	camera(Ieee1394Camera::Monocular, i1394b, uniqId, 1);
	cerr << "camera's uniqId = "
	     << hex << setw(16) << setfill('0')
	     << camera.globalUniqueId() << endl;

	MarkerDetector	markerDetector(cropSize, strengthTh, distTh,
				       alpha, th_low, th_high);

	v::CmdDef*	MenuCmds    = createMenuCmds(camera);
	v::CmdDef*	CaptureCmds = createCaptureCmds();
	v::CmdDef*	FeatureCmds = createFeatureCmds(camera);
	v::MyCmdWindow	myWin(vapp, MenuCmds, CaptureCmds, FeatureCmds,
			      camera, markerDetector, pitch, calibFile);
	vapp.run();
    }
    catch (exception& err)
    {
	cerr << err.what() << endl;
	return 1;
    }

    return 0;
}
