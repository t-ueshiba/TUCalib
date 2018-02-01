/*
 *  $Id: calibCameras.h,v 1.2 2007-03-08 05:11:09 ueshiba Exp $
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

  // Actions for calibration.
    c_Extract,
    c_Calibrate
};

/************************************************************************
*  global functions							*
************************************************************************/
Ieee1394Camera::Feature		id2feature(v::CmdId id)			;
}

#endif	// !__calibCameras_h
