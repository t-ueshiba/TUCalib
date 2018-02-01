/*
 *  $Id: createFeatureCmds.cc,v 1.1.1.1 2006-06-02 06:11:17 ueshiba Exp $
 */
#include "calibCameras.h"

namespace TU
{
using namespace	v;
/************************************************************************
*  local data								*
************************************************************************/
struct Feature
{
    Ieee1394Camera::Feature	feature;
    const char*			name;
    int				prop[3];
};
static Feature		feature[] =
{
    {Ieee1394Camera::BRIGHTNESS,	"Brightness     "},
    {Ieee1394Camera::AUTO_EXPOSURE,	"Auto exposure  "},
    {Ieee1394Camera::SHARPNESS,		"Sharpness      "},
    {Ieee1394Camera::WHITE_BALANCE,	"White bal.(U/B)"},
    {Ieee1394Camera::WHITE_BALANCE,	"White bal.(V/R)"},
    {Ieee1394Camera::HUE,		"Hue            "},
    {Ieee1394Camera::SATURATION,	"Saturation     "},
    {Ieee1394Camera::GAMMA,		"Gamma          "},
    {Ieee1394Camera::SHUTTER,		"Shutter        "},
    {Ieee1394Camera::GAIN,		"Gain           "},
    {Ieee1394Camera::IRIS,		"Iris           "},
    {Ieee1394Camera::FOCUS,		"Focus          "},
    {Ieee1394Camera::TEMPERATURE,	"Temperature    "},
    {Ieee1394Camera::ZOOM,		"Zoom           "}
};
static const int	NFEATURES = sizeof(feature)/sizeof(feature[0]);
static CmdDef		featureCmds[3*NFEATURES + 1];

/************************************************************************
*  global functions							*
************************************************************************/
Ieee1394Camera::Feature
id2feature(CmdId id)
{
    return feature[id - c_Brightness].feature;
}

CmdDef*
createFeatureCmds(const Ieee1394Camera& camera)
{
    u_int		ncmds = 0, y = 0;
    for (int i = 0; i < NFEATURES; ++i)
    {
	u_int	inq = camera.inquireFeatureFunction(feature[i].feature);
	if (inq & Ieee1394Camera::Presence)
	{
	    u_int	x = 1;
	    
	    if (inq & Ieee1394Camera::OnOff)
	    {
	      // Create toggle button for turning on/off this feature.
		featureCmds[ncmds].type	      = C_ToggleButton;
		featureCmds[ncmds].id	      = c_Brightness+i+OFFSET_ONOFF;
		featureCmds[ncmds].val	      = camera.isTurnedOn(feature[i]
								  .feature);
		featureCmds[ncmds].title      = "On";
		featureCmds[ncmds].prop       = noProp;
		featureCmds[ncmds].attrs      = CA_None;
		featureCmds[ncmds].gridx      = x++;
		featureCmds[ncmds].gridy      = y;
		featureCmds[ncmds].gridWidth  = 1;
		featureCmds[ncmds].gridHeight = 1;
		featureCmds[ncmds].size	      = 0;
		++ncmds;
	    }

	    if (inq & Ieee1394Camera::Manual)
	    {
		if (inq & Ieee1394Camera::Auto)
		{
		  // Create toggle button for setting manual/auto mode.
		    featureCmds[ncmds].type	  = C_ToggleButton;
		    featureCmds[ncmds].id	  = c_Brightness+i+OFFSET_AUTO;
		    featureCmds[ncmds].val	  = camera.isAuto(feature[i]
								  .feature);
		    featureCmds[ncmds].title	  = "Auto";
		    featureCmds[ncmds].prop       = noProp;
		    featureCmds[ncmds].attrs      = CA_None;
		    featureCmds[ncmds].gridx      = x;
		    featureCmds[ncmds].gridy      = y;
		    featureCmds[ncmds].gridWidth  = 1;
		    featureCmds[ncmds].gridHeight = 1;
		    featureCmds[ncmds].size	  = 0;
		    ++ncmds;
		}
		
	      // Create sliders for setting values.
		u_int	min, max;
		camera.getMinMax(feature[i].feature, min, max);
		feature[i].prop[0] = min;
		feature[i].prop[1] = max - min;
		feature[i].prop[2] = 1;
		featureCmds[ncmds].type	      = C_Slider;
		featureCmds[ncmds].id	      = c_Brightness+i;
		featureCmds[ncmds].title      = feature[i].name;
		featureCmds[ncmds].prop       = feature[i].prop;
		featureCmds[ncmds].attrs      = CA_None;
		featureCmds[ncmds].gridx      = 0;
		featureCmds[ncmds].gridy      = y;
		featureCmds[ncmds].gridWidth  = 1;
		featureCmds[ncmds].gridHeight = 1;
		featureCmds[ncmds].size	      = 0;
		if (feature[i].feature == Ieee1394Camera::WHITE_BALANCE)
		{
		    ++ncmds;
		    ++i;
		    ++y;
		    feature[i].prop[0] = min;
		    feature[i].prop[1] = max - min;
		    feature[i].prop[2] = 1;
		    featureCmds[ncmds].type	  = C_Slider;
		    featureCmds[ncmds].id	  = c_Brightness+i;
		    featureCmds[ncmds].title      = feature[i].name;
		    featureCmds[ncmds].prop       = feature[i].prop;
		    featureCmds[ncmds].attrs      = CA_None;
		    featureCmds[ncmds].gridx      = 0;
		    featureCmds[ncmds].gridy      = y;
		    featureCmds[ncmds].gridWidth  = 1;
		    featureCmds[ncmds].gridHeight = 1;
		    featureCmds[ncmds].size	  = 0;
		    u_int	ub, vr;
		    camera.getWhiteBalance(ub, vr);
		    featureCmds[ncmds-1].val = ub;
		    featureCmds[ncmds  ].val = vr;
		}
		else
		    featureCmds[ncmds].val
			= camera.getValue(feature[i].feature);

		++ncmds;
	    }

	    ++y;
	}
    }
    featureCmds[ncmds].type = C_EndOfList;

    return featureCmds;
}
 
}
