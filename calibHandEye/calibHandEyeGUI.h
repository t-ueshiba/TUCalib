/*
 *  平成14-19年（独）産業技術総合研究所 著作権所有
 *  
 *  創作者：植芝俊夫
 *
 *  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
 *  （独）産業技術総合研究所が著作権を所有する秘密情報です．著作権所有
 *  者による許可なしに本プログラムを使用，複製，改変，第三者へ開示する
 *  等の行為を禁止します．
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
 *  without permission by the copyright holder are strictly prohibited.
 *
 *  [No Warranty.]
 *  The copyright holder or the creator are not responsible for any
 *  damages caused by using this program.
 *
 *  $Id: planeCalibGUI.h 1254 2012-12-05 09:51:49Z ueshiba $  
 */
#ifndef __calibCameras_h
#define __calibCameras_h

#include "TU/v/TUv++.h"
#include "TU/Ieee1394++.h"

/************************************************************************
*  global data and definitions						*
************************************************************************/
#define OFFSET_ONOFF	0x100
#define OFFSET_AUTO	0x200

namespace TU
{
namespace v
{
enum
{
    c_Frame,

  // File menu
    c_RestoreConfig,
    c_SaveConfig,

  // Camera selection.
    c_SelectionSchemes,
    c_TripletSelection,
    c_CameraSelection,

  // Camera feasures.
    c_Brightness = Ieee1394Camera::BRIGHTNESS,
    c_AutoExposure,
    c_Sharpness,
    c_WhiteBalance_UB,
    c_WhiteBalance_VR,
    c_Hue,
    c_Saturation,
    c_Gamma,
    c_Shutter,
    c_Gain,
    c_Iris,
    c_Focus,
    c_Temperature,
    c_Zoom,

  // Camera video format.
    c_Format,
    c_YUV444_160x120	= Ieee1394Camera::YUV444_160x120,
    c_YUV422_320x240	= Ieee1394Camera::YUV422_320x240,
    c_YUV411_640x480	= Ieee1394Camera::YUV411_640x480,
    c_YUV422_640x480	= Ieee1394Camera::YUV422_640x480,
    c_RGB24_640x480	= Ieee1394Camera::RGB24_640x480,
    c_MONO8_640x480	= Ieee1394Camera::MONO8_640x480,
    c_MONO16_640x480	= Ieee1394Camera::MONO16_640x480,
    c_YUV422_800x600	= Ieee1394Camera::YUV422_800x600,
    c_RGB24_800x600	= Ieee1394Camera::RGB24_800x600,
    c_MONO8_800x600	= Ieee1394Camera::MONO8_800x600,
    c_YUV422_1024x768	= Ieee1394Camera::YUV422_1024x768,
    c_RGB24_1024x768	= Ieee1394Camera::RGB24_1024x768,
    c_MONO8_1024x768	= Ieee1394Camera::MONO8_1024x768,
    c_MONO16_800x600	= Ieee1394Camera::MONO16_800x600,
    c_MONO16_1024x768	= Ieee1394Camera::MONO16_1024x768,
    c_YUV422_1280x960	= Ieee1394Camera::YUV422_1280x960,
    c_RGB24_1280x960	= Ieee1394Camera::RGB24_1280x960,
    c_MONO8_1280x960	= Ieee1394Camera::MONO8_1280x960,
    c_YUV422_1600x1200	= Ieee1394Camera::YUV422_1600x1200,
    c_RGB24_1600x1200	= Ieee1394Camera::RGB24_1600x1200,
    c_MONO8_1600x1200	= Ieee1394Camera::MONO8_1600x1200,
    c_MONO16_1280x960	= Ieee1394Camera::MONO16_1280x960,
    c_MONO16_1600x1200	= Ieee1394Camera::MONO16_1600x1200,

  // # of model planes.
    c_NPlanes,

  // Common camera centers or not.
    c_CommonCenters,
    
  // Actions for calibration.
    c_Extract,
    c_Calibrate,
    c_Check,
    c_NextPosition
};

/************************************************************************
*  global functions							*
************************************************************************/
Ieee1394Camera::Feature		id2feature(CmdId id)			;

}
}

#endif	// !__calibCameras_h
