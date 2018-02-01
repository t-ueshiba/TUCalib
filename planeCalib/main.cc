/*
 *  $Id$  
 */
/*!
  \mainpage	planeCalibGUI - 既知の平面パターンを1台もしくはそれ以上のIIDCカメラに提示することによってカメラの内部・外部パラメータを推定するプロブラム
  \anchor	planeCalibGUI

  \section copyright 著作権
  平成14-19年（独）産業技術総合研究所 著作権所有

  創作者：植芝俊夫

  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
  （独）産業技術総合研究所が著作権を所有する秘密情報です．著作権所有
  者による許可なしに本プログラムを使用，複製，改変，第三者へ開示する
  等の行為を禁止します．
   
  このプログラムによって生じるいかなる損害に対しても，著作権所有者お
  よび創作者は責任を負いません。

  Copyright 2002-2007.
  National Institute of Advanced Industrial Science and Technology (AIST)

  Creator: Toshio UESHIBA

  [AIST Confidential and all rights reserved.]
  This program is confidential. Any using, copying, changing or
  giving any information concerning with this program to others
  without permission by the copyright holder are strictly prohibited.

  [No Warranty.]
  The copyright holder or the creator are not responsible for any
  damages caused by using this program.

  \section abstract 概要
  planeCalibGUIは，既知の平面パターンを1台もしくはそれ以上のIIDCカ
  メラに提示することによって，GUIにより対話的にカメラの内部・外部パラメー
  タを推定するプロブラムである．

  \file		main.cc
  \brief	メイン関数
*/
#include <unistd.h>
#include <cstdlib>
#include <iomanip>
#include <stdexcept>
#include "TU/v/vIIDC++.h"
#include "MyCmdWindow.h"

static const double		DEFAULT_PITCH = 30.0;

//! 本プログラムで定義されたクラスおよび関数を収める名前空間
namespace TU
{
/************************************************************************
*  static functions							*
************************************************************************/
//! コマンドの使用法を表示する．
/*!
  \param s	コマンド名
*/
static void
usage(const char* s)
{
    using namespace	std;

    MarkerDetector::Parameters	params;
    
    cerr << "\nCalibrate IIDC-based multi-camera system with a planar pattern.\n"
	 << endl;
    cerr << " Usage: " << s << " [-b] [-p pitch] [-c cameraName]\n"
	 << endl;
    cerr << " configuration options.\n"
	 << "  -c cameraName:    prefix of camera {conf|calib} file\n"
	 << "                      (default: \""
	 << IIDCCameraArray::DEFAULT_CAMERA_NAME
	 << "\")\n"
	 << "  -B:               IEEE1394b mode. (default: off)\n"
	 << endl;
    cerr << " Options for marker detection.\n"
	 << "  -R:               90 deg. rotated calib. pattern "
	 << "(default: off)\n"
	 << "  -C cropSize:      size of cropping square for template matching.\n"
	 << "                      (default: "
	 << params.cropSize
	 << ")\n"
	 << "  -S strengthRatio: ratio to the highest template matching score.\n"
	 << "                      (default: "
	 << params.strengthRatio << ")\n"
	 << "  -D distTh:        distance threshold for projective "
	 << "transformation.\n"
	 << "                      (default: "
	 << std::sqrt(params.sqdistTh)
	 << ")\n"
	 << "  -W winSize:       window size for refining marker location.\n"
	 << "                      (default: "
	 << params.winSize
	 << ")\n"
	 << "  -e:               refine marker locations by detecting "
	 << "edge intersections.\n"
	 << "                      (default: off)\n"
	 << "  -A alpha:         size of Deriche edge detector for refining "
	 << "marker locations.\n"
	 << "                      (default: "
	 << params.alpha
	 << ")\n"
	 << "  -L lowTh:         lower threshold for edge detection.\n"
	 << "                      (default: "
	 << params.lowTh
	 << ")\n"
	 << "  -H highTh:        higher threshold for edge detection.\n"
	 << "                      (default: "
	 << params.highTh
	 << ")\n"
	 << endl;
    cerr << " Other options.\n"
	 << "  -p pitch:         pitch of grid pattern. (default: "
	 << DEFAULT_PITCH
	 << ")\n"
	 << "  -h:               print this.\n"
	 << endl;
}

static float	square(float val)	{ return val*val; }
    
}
/************************************************************************
*  global functions							*
************************************************************************/
//! メイン関数
int
main(int argc, char* argv[])
{
    using namespace	std;
    using namespace	TU;
    
    v::App		vapp(argc, argv);
    const char*		cameraName = IIDCCameraArray::DEFAULT_CAMERA_NAME;
    IIDCCamera::Speed	speed	   = IIDCCamera::SPD_400M;
    bool		horizontal = true;
    double		pitch	   = DEFAULT_PITCH;
    MarkerDetector::Parameters
			params;

  // Parse command options.
    extern char*	optarg;
    for (int c; (c = getopt(argc, argv, "c:BRC:S:D:W:A:eL:H:p:h")) != -1; )
	switch (c)
	{
	  case 'c':
	    cameraName = optarg;
	    break;
	  case 'B':
	    speed = IIDCCamera::SPD_800M;
	    break;
	  case 'R':
	    horizontal = false;
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
	  case 'e':
	    params.edgeIntersection = true;
	    break;
	  case 'L':
	    params.lowTh = atof(optarg);
	    break;
	  case 'H':
	    params.highTh = atof(optarg);
	    break;
	  case 'p':
	    pitch = atof(optarg);
	    break;
	  case 'h':
	    usage(argv[0]);
	    return 1;
	}
    
  // Main job.
    try
    {
      // IIDCカメラのオープン．
	IIDCCameraArray	cameras;
	cameras.restore(cameraName, speed);
	
	for (int i = 0; i < cameras.size(); ++i)
	    cerr << "camera " << i << ": uniqId = "
		 << hex << setw(16) << setfill('0')
		 << cameras[i].globalUniqueId() << endl;

	v::MyCmdWindow<IIDCCameraArray, u_char>	myWin(vapp, cameras, params,
						      pitch, horizontal);
	vapp.run();
    }
    catch (exception& err)
    {
	cerr << err.what() << endl;
	return 1;
    }

    return 0;
}
