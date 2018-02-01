/*
 *  $Id: main.cc,v 1.2 2007-03-08 05:11:09 ueshiba Exp $
 */
#include <unistd.h>
#include <stdlib.h>
#include <iomanip>
#include <stdexcept>
#include "MyCmdWindow.h"

#define	DEFAULT_CAMERA_BASE	"/usr/local/etc/cameras/IEEE1394Camera"
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

    cerr << "\nCalibrate IEEE1394-based multi-camera system with a planar pattern.\n"
	 << endl;
    cerr << " Usage: " << s << " [-b] [-p pitch] [-c cameraBase]\n"
	 << endl;
    cerr << " General options.\n"
	 << "  -b:             IEEE1394b mode.        (default: off)\n"
	 << "  -p pitch:       pitch of grid pattern. (default: "
	 << DEFAULT_PITCH << ")\n"
	 << "  -c cameraBase:  basename of camera {conf|calib} files.\n"
	 << "                    (default: "
	 << DEFAULT_CAMERA_BASE
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

/************************************************************************
*  class CameraArray							*
************************************************************************/
class CameraArray : public Array<Ieee1394Camera*>
{
  public:
    CameraArray(const char* cameraConfigFile, bool i1394b)	;
    ~CameraArray()						;
};

CameraArray::CameraArray(const char* cameraConfigFile, bool i1394b)
    :Array<Ieee1394Camera*>()
{
    using namespace	std;

    ifstream	in(cameraConfigFile);
    if (!in)
	throw invalid_argument("Cannot open camera configuration file!!");

    u_int	delay, ncameras;
    in >> delay >> ncameras;			// Read delay value and #cameras.
    resize(ncameras);
    
    for (int i = 0; i < dim(); ++i)
    {
	string	s;
	in >> s;				// Read global unique ID.
	u_int64	uniqId = strtoull(s.c_str(), 0, 0);
	(*this)[i] = new Ieee1394Camera(Ieee1394Camera::Monocular,
					i1394b, uniqId, delay);

	in >> *(*this)[i];			// Read camera parameters.
    }
}

CameraArray::~CameraArray()
{
    for (int i = 0; i < dim(); ++i)
	delete (*this)[i];
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
    string		cameraBase(DEFAULT_CAMERA_BASE);
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
	    cameraBase = optarg;
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
    
  // Main job.
    try
    {
	string		str = cameraBase + ".conf";
	CameraArray	cameras(str.c_str(), i1394b);
	if (cameras.dim() < 2)
	    throw runtime_error("At least *two* cameras required!!");
	
	for (int i = 0; i < cameras.dim(); ++i)
	    cerr << "camera " << i << ": uniqId = "
		 << hex << setw(16) << setfill('0')
		 << cameras[i]->globalUniqueId() << endl;

	MarkerDetector	markerDetector(cropSize, strengthTh, distTh,
				       alpha, th_low, th_high);

	v::CmdDef*	MenuCmds    = createMenuCmds(*cameras[0]);
	v::CmdDef*	CaptureCmds = createCaptureCmds();
	v::CmdDef*	FeatureCmds = createFeatureCmds(*cameras[0]);
	v::MyCmdWindow	myWin(vapp, MenuCmds, CaptureCmds, FeatureCmds,
			      cameras, markerDetector, pitch, cameraBase);
	vapp.run();
    }
    catch (exception& err)
    {
	cerr << err.what() << endl;
	return 1;
    }

    return 0;
}
