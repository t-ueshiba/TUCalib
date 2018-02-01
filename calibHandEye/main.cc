/*
 *  平成14-19年（独）産業技術総合研究所 著作権所有
 *  
 *  創作者：植芝俊夫
 *
 *  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
 *  （独）産業技術総合研究所が著作権を所有する秘密情報です．創作者によ
 *  る許可なしに本プログラムを使用，複製，改変，第三者へ開示する等の著
 *  作権を侵害する行為を禁止します．
 *  
 *  このプログラムによって生じるいかなる損害に対しても，著作権所有者お
 *  よび創作者は責任を負いません。
 *
 *  Copyright 2002-2007.
 *  National Institute of Advanced Industrial Science and Technology (AIST)
 *
 *  Creator: Toshio UESHIBA
 *
 *  [AIST Confidential and all rights reserved.]
 *  This program is confidential. Any using, copying, changing or
 *  giving any information concerning with this program to others
 *  without permission by the creator are strictly prohibited.
 *
 *  [No Warranty.]
 *  The copyright holders or the creator are not responsible for any
 *  damages in the use of this program.
 *  
 *  $Id: main.cc,v 1.17 2012-09-01 07:23:38 ueshiba Exp $
 */
#include <cstdlib>
#include <iomanip>
#include <stdexcept>
#include "MyCmdWindow.h"

#define DEFAULT_OUTPUT_BASE	"calibHandEye"

static const double		DEFAULT_PITCH		= 30.0;

namespace TU
{
/************************************************************************
*  static functions							*
************************************************************************/
template <class T> static inline T	square(T val)	{return val * val;}
    
static void
usage(const char* s)
{
    using namespace		std;

    MarkerDetector::Parameters	params;
    
    cerr << "\nCalibrate IEEE1394-based multi-camera system with a planar pattern.\n"
	 << endl;
    cerr << " Usage: " << s << " [-b] [-p pitch] [-c cameraBase]\n"
	 << endl;
    cerr << " Configuration options.\n"
	 << "  -c cameraName:  prefix of camera {conf|calib} file\n"
	 << "                    (default: \""
	 << DEFAULT_CAMERA_NAME
	 << "\")\n"
	 << "  -d configDirs:  list of directories for camera {conf|calib} file\n"
	 << "                    (default: \""
	 << DEFAULT_CONFIG_DIRS
	 << "\")\n"
	 << "  -B:             IEEE1394b mode. (default: off)\n"
	 << endl;
    cerr << " Calibration options.\n"
	 << "  -r:             turn on non-linear refinement in calibration (default: off)\n"
	 << endl;
    cerr << " Options for marker detection.\n"
	 << "  -R:             90 deg. rotated calib. pattern (default: off)\n"
	 << "  -C cropSize:    size of cropping square for template matching.\n"
	 << "                    (default: "
	 << params.cropSize
	 << ")\n"
	 << "  -S strengthTh:  threshold for template matcing in ratio to "
	 << "the highest score.\n"
	 << "                    (default: "
	 << params.strengthRatio
	 << ")\n"
	 << "  -D strengthRatio: distance threshold for projective "
	 << "transformation.\n"
	 << "                    (default: "
	 << std::sqrt(params.sqdistTh)
	 << ")\n"
	 << "  -W winSize:     window size for refining marker location.\n"
	 << "                    (default: "
	 << params.winSize
	 << ")\n"
	 << "  -A alpha:       size of Deriche edge detector for refining "
	 << "marker locations.\n"
	 << "                    (default: "
	 << params.alpha
	 << ")\n"
	 << "  -L th_low:      lower threshold for edge detection.\n"
	 << "                    (default: "
	 << params.lowTh
	 << ")\n"
	 << "  -H th_high:     higher threshold for edge detection.\n"
	 << "                    (default: "
	 << params.highTh
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

  // Main job.
    try
    {
	HRP2				hrp2(argc, argv);
	hrp2.setup(false, false);	// RIGHT hand, IMMEDIATE mode.
	
	v::App				vapp(argc, argv);
	const char*			configDirs = DEFAULT_CONFIG_DIRS;
	const char*			cameraName = DEFAULT_CAMERA_NAME;
	Ieee1394Node::Speed		speed	   = Ieee1394Node::SPD_400M;
	bool				horizontal = false;
	bool				dump	   = false;
	MarkerDetector::Parameters	params;

      // Parse command options.
	extern char*	optarg;
	for (int c; (c = getopt(argc, argv, "c:d:BuRC:S:D:W:A:L:H:h")) != -1; )
	    switch (c)
	    {
	      case 'c':
		cameraName = optarg;
		break;
	      case 'd':
		configDirs = optarg;
		break;
	      case 'B':
		speed = Ieee1394Node::SPD_800M;
		break;
	      case 'u':
		dump = true;
		break;
	      case 'R':
		horizontal = true;
		break;
	      case 'C':
		params.cropSize = atoi(optarg);
		break;
	      case 'S':
		params.strengthRatio = atof(optarg);
		break;
	      case 'D':
		params.sqdistTh = square(atof(optarg));
		break;
	      case 'W':
		params.winSize = atoi(optarg);
		break;
	      case 'A':
		params.alpha = atof(optarg);
		break;
	      case 'L':
		params.lowTh = atof(optarg);
		break;
	      case 'H':
		params.highTh = atof(optarg);
		break;
	      case 'h':
		usage(argv[0]);
		return 1;
	    }
	string	outputBase = DEFAULT_OUTPUT_BASE;
	extern int	optind;
	if (optind < argc)
	    outputBase = argv[optind];
    
	Ieee1394CameraArray	cameras(cameraName, configDirs, speed);
	for (int i = 0; i < cameras.size(); ++i)
	    cameras[i]->embedTimestamp();	// 画像にtimestampを埋め込む

	v::MyCmdWindow	myWin(hrp2, vapp, cameras, params,
			      outputBase, horizontal, dump);
	vapp.run();
    }
    catch (exception& err)
    {
	cerr << err.what() << endl;
	return 1;
    }
    
    return 0;
}
