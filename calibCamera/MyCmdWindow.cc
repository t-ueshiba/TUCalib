/*
 *  $Id: MyCmdWindow.cc,v 1.1.1.1 2007-05-23 05:55:43 ueshiba Exp $
 */
#include <unistd.h>
#include <sys/time.h>
#include <sstream>
#include <iomanip>
#include "MyCmdWindow.h"
#include "TU/v/Confirm.h"
#include "TU/v/Notify.h"

namespace TU
{
template <class INTRINSIC> INTRINSIC
calibCameraWithPlanes(const PairListList& data, bool doRefinement);
    
namespace v
{
/************************************************************************
*  static functions							*
************************************************************************/
static void
countTime(int& nframes, struct timeval& start)
{
    if (nframes == 10)
    {
	struct timeval	end;
	gettimeofday(&end, NULL);
	double	interval = (end.tv_sec  - start.tv_sec) +
	    (end.tv_usec - start.tv_usec) / 1.0e6;
	std::cerr << nframes / interval << " frames/sec" << std::endl;
	nframes = 0;
    }
    if (nframes++ == 0)
	gettimeofday(&start, NULL);
}

/************************************************************************
*  class MyCmdWindow							*
************************************************************************/
MyCmdWindow::MyCmdWindow(App&			parentApp,
			 const CmdDef		menuCmds[],
			 const CmdDef		captureCmds[],
			 const CmdDef		featureCmds[],
			 Ieee1394Camera&	camera,
			 const MarkerDetector&	markerDetector,
			 double			pitch,
			 const char*		calibFile)
    :CmdWindow(parentApp, "Camera calibration",
	       Colormap::RGBColor, 256, 0, 0),
     _camera(camera),
     _image(),
     _markerDetector(markerDetector),
     _data(),
     _pitch(pitch),
     _calibFile(calibFile),
     _menuCmd(*this, menuCmds),
     _captureCmd(*this, captureCmds),
     _featureCmd(*this, featureCmds),
     _fileSelection(*this),
     _canvas(*this, _camera.width(), _camera.height(), _image, _crosses),
     _timer(*this, 0)
{
    using namespace	std;
    
    _menuCmd.place(0, 0, 2, 1);
    _captureCmd.place(0, 1, 1, 1);
    _featureCmd.place(1, 1, 1, 1);
    _canvas.place(0, 2, 2, 1);
    show();

    continuousShot();
}

void
MyCmdWindow::callback(CmdId id, CmdVal val)
{
    using namespace	std;

    Vector3i	p;
    p[0] = _canvas.dc().dev2logU(val.u);
    p[1] = _canvas.dc().dev2logV(val.v);
    p[2] = 1;
    
    switch (id)
    {
      case M_Exit:
	app().exit();
	break;

      case M_Open:
      {
	stopContinuousShot();

	try
	{
	    ifstream	in;
	    if (_fileSelection.open(in))
	    {
		_image.restore(in);
		extractCrosses();
	    }
	}
	catch (exception& err)
	{
	    Notify	notify(*this);
	    notify << err.what();
	    notify.show();
	}
	
	continuousShot();
      };
        break;
      
      case M_Save:
      {
	stopContinuousShot();

	ofstream	out;
	_fileSelection.open(out);
	_image.save(out, ImageBase::U_CHAR);

	continuousShot();
      }
        break;
      
      case c_YUV444_160x120:
      case c_YUV422_320x240:
      case c_YUV411_640x480:
      case c_YUV422_640x480:
      case c_RGB24_640x480:
      case c_MONO8_640x480:
      case c_MONO16_640x480:
      case c_YUV422_800x600:
      case c_RGB24_800x600:
      case c_MONO8_800x600:
      case c_YUV422_1024x768:
      case c_RGB24_1024x768:
      case c_MONO8_1024x768:
      case c_MONO16_800x600:
      case c_MONO16_1024x768:
      case c_YUV422_1280x960:
      case c_RGB24_1280x960:
      case c_MONO8_1280x960:
      case c_YUV422_1600x1200:
      case c_RGB24_1600x1200:
      case c_MONO8_1600x1200:
      case c_MONO16_1280x960:
      case c_MONO16_1600x1200:
	_camera.setFormatAndFrameRate(Ieee1394Camera::uintToFormat(id),
				      Ieee1394Camera::uintToFrameRate(val));
	_canvas.resize(_camera.width(), _camera.height());
	break;

      case c_Brightness:
      case c_AutoExposure:
      case c_Sharpness:
      case c_Hue:
      case c_Saturation:
      case c_Gamma:
      case c_Shutter:
      case c_Gain:
      case c_Iris:
      case c_Focus:
      case c_Zoom:
	_camera.setValue(id2feature(id), val);
        break;
      
      case c_WhiteBalance_UB:
	_camera.setWhiteBalance(val, _featureCmd.getValue(c_WhiteBalance_VR));
	break;
      case c_WhiteBalance_VR:
	_camera.setWhiteBalance(_featureCmd.getValue(c_WhiteBalance_UB), val);
	break;
      
      case c_Brightness	     + OFFSET_ONOFF:
      case c_AutoExposure    + OFFSET_ONOFF:
      case c_Sharpness	     + OFFSET_ONOFF:
      case c_WhiteBalance_UB + OFFSET_ONOFF:
      case c_WhiteBalance_VR + OFFSET_ONOFF:
      case c_Hue	     + OFFSET_ONOFF:
      case c_Saturation	     + OFFSET_ONOFF:
      case c_Gamma	     + OFFSET_ONOFF:
      case c_Shutter	     + OFFSET_ONOFF:
      case c_Gain	     + OFFSET_ONOFF:
      case c_Iris	     + OFFSET_ONOFF:
      case c_Focus	     + OFFSET_ONOFF:
      case c_Zoom	     + OFFSET_ONOFF:
      {
	Ieee1394Camera::Feature feature = id2feature(id - OFFSET_ONOFF);
	if (val)
	    _camera.turnOn(feature);
	else
	    _camera.turnOff(feature);
      }
        break;
      
      case c_Brightness	     + OFFSET_AUTO:
      case c_AutoExposure    + OFFSET_AUTO:
      case c_Sharpness	     + OFFSET_AUTO:
      case c_WhiteBalance_UB + OFFSET_AUTO:
      case c_WhiteBalance_VR + OFFSET_AUTO:
      case c_Hue	     + OFFSET_AUTO:
      case c_Saturation	     + OFFSET_AUTO:
      case c_Gamma	     + OFFSET_AUTO:
      case c_Shutter	     + OFFSET_AUTO:
      case c_Gain	     + OFFSET_AUTO:
      case c_Iris	     + OFFSET_AUTO:
      case c_Focus	     + OFFSET_AUTO:
      case c_Zoom	     + OFFSET_AUTO:
      {
	Ieee1394Camera::Feature feature = id2feature(id - OFFSET_AUTO);
	if (val)
	    _camera.setAutoMode(feature);
	else
	{
	    _camera.setManualMode(feature);
	    if (feature == Ieee1394Camera::WHITE_BALANCE)
		_camera.setWhiteBalance(_featureCmd.getValue(c_WhiteBalance_UB),
					_featureCmd.getValue(c_WhiteBalance_VR));
	    else
		_camera.setValue(feature,
				 _featureCmd.getValue(id - OFFSET_AUTO));
	}
      }
        break;

      case c_Extract:
	stopContinuousShot();
	extractCrosses();
	continuousShot();
        break;

      case c_Calibrate:
      	cout << _data;

	try
	{
	    Calib_t::Intrinsic	intrinsic
		= calibCameraWithPlanes<Calib_t::Intrinsic>(_data, true);
	    ofstream	out(_calibFile);
	    if (!out)
		throw runtime_error("Cannot open the output calibration file!!");
	    Vector<double>	t(3);
	    Matrix<double>	Rt = Matrix<double>::I(3);
	    Calib_t		calib(t, Rt,
				      intrinsic.k(),
				      intrinsic.principal()[0],
				      intrinsic.principal()[1],
				      intrinsic.aspect(),
				      intrinsic.skew(),
				      intrinsic.d1(),
				      intrinsic.d2());
	    _image.P  = calib.P();
	    _image.d1 = calib.d1();
	    _image.d2 = calib.d2();
	    out << _image.P << _image.d1 << ' ' << _image.d2 << '\n' << endl;

	    _data.clear();
	    _captureCmd.setString(c_NPlanes, "0 plane(s)");
	}
	catch (exception& err)
	{
	    Notify	notify(*this);
	    notify << err.what();
	    notify.show();
	}
	break;
    }
}

void
MyCmdWindow::tick()
{
    static int			nframes = 0;
    static struct timeval	start;
    countTime(nframes, start);

    _camera.snap();
    _camera >> _image;
    _canvas.display();
}

void
MyCmdWindow::continuousShot()
{
    _camera.continuousShot();
    _timer.start(1);
}
    
void
MyCmdWindow::stopContinuousShot()
{
    _timer.stop();
    _camera.stopContinuousShot();
}
    
void
MyCmdWindow::extractCrosses()
{
    using namespace	std;
    
    try
    {
	_markerDetector(_image, _crosses, false);
	_canvas.display();

	Confirm	confirm(*this);
	confirm << "Reference points extracted. Accept?";
	if (confirm.ok())
	{
	    for (PairList::iterator iter  = _crosses.begin();
				    iter != _crosses.end(); ++iter)
		iter->first *= _pitch;
	    _data.push_back(_crosses);
	    _crosses.clear();
	}
	ostringstream	s;
	s << _data.size() << " plane(s)";
	_captureCmd.setString(c_NPlanes, s.str().c_str());
    }
    catch (exception& err)
    {
	Notify	notify(*this);
	notify << err.what();
	notify.show();
    }
}

}
}
