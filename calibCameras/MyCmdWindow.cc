/*
 *  $Id: MyCmdWindow.cc,v 1.4 2007-05-23 01:40:56 ueshiba Exp $
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
template <class CAMERA>
Array<CAMERA>	calibCamerasWithPlanes(const PairListArrayList& data,
				       bool doRefinement);
    
namespace v
{
/************************************************************************
*  static functions							*
************************************************************************/
inline int	min(int a, int b)	{return (a < b ? a : b);}
    
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
MyCmdWindow::MyCmdWindow(App&				parentApp,
			 const CmdDef			menuCmds[],
			 const CmdDef			captureCmds[],
			 const CmdDef			featureCmds[],
			 const Array<Ieee1394Camera*>&	cameras,
			 const MarkerDetector&		markerDetector,
			 double				pitch,
			 const std::string&		cameraBase)
    :CmdWindow(parentApp, "Multi-camera calibration",
#ifdef UseOverlay
	       Colormap::IndexedColor, 64, 3, 1
#else
	       Colormap::RGBColor, 256, 0, 0
#endif
	      ),
     _cameras(cameras),
     _images(_cameras.dim()),
     _markerDetector(markerDetector),
     _crosses(_cameras.dim()),
     _data(),
     _Fh(3, 3),
     _Fv(3, 3),
     _q(),
     _pitch(pitch),
     _cameraBase(cameraBase),
     _menuCmd(*this, menuCmds),
     _captureCmd(*this, captureCmds),
     _featureCmd(*this, featureCmds),
     _fileSelection(*this),
     _canvasC(*this, 
	      _cameras[min(0, cameras.dim()-1)]->width()  / 2,
	      _cameras[min(0, cameras.dim()-1)]->height() / 2,
	      _images [min(0, cameras.dim()-1)],
	      _crosses[min(0, cameras.dim()-1)]),
     _canvasH(*this,
	      _cameras[min(1, cameras.dim()-1)]->width()  / 2,
	      _cameras[min(1, cameras.dim()-1)]->height() / 2,
	      _images [min(1, cameras.dim()-1)],
	      _crosses[min(1, cameras.dim()-1)]),
     _canvasV(*this,
	      _cameras[min(2, cameras.dim()-1)]->width()  / 2,
	      _cameras[min(2, cameras.dim()-1)]->height() / 2,
	      _images [min(2, cameras.dim()-1)],
	      _crosses[min(2, cameras.dim()-1)]),
     _timer(*this, 0)
{
    using namespace	std;
    
    _menuCmd.place(0, 0, 2, 1);
    _captureCmd.place(0, 1, 1, 1);
    _featureCmd.place(1, 1, 1, 2);
    _canvasC.place(0, 3, 1, 1);
    _canvasH.place(1, 3, 1, 1);
    _canvasV.place(0, 2, 1, 1);
#ifdef UseOverlay
    colormap().setUnderlayValue(0, BGR(0, 255, 255));
    colormap().setUnderlayValue(1, BGR(255, 0, 255));
    colormap().setUnderlayValue(2, BGR(255, 255, 0));
    colormap().setOverlayValue(1, BGR(0, 255, 0));
#endif
    show();

    _canvasC.setZoom(1, 2);
    _canvasH.setZoom(1, 2);
    _canvasV.setZoom(1, 2);

    string	s = _cameraBase + ".calib";
    ifstream	in(s.c_str());
    if (in)			// If a calibration file exists,...
    {
	for (int i = 0; i < _images.dim(); ++i)
	    in >> _images[i].P
	       >> _images[i].d1 >> _images[i].d2;  // Read calibration data.
    }
    else			// If no calibration files, set default values.
    {
	_images[1].P[0][3] = -1.0;
	if (_images.dim() > 2)
	    _images[2].P[1][3] = -1.0;
    }
    _Fh = computeFMatrix(_images[0].P, _images[1].P);
    if(_images.dim() > 2)
	_Fv = computeFMatrix(_images[0].P, _images[2].P);
    
    continuousShot();
}

void
MyCmdWindow::callback(CmdId id, CmdVal val)
{
    using namespace	std;

    Vector3i	p;
    p[0] = _canvasC.dc().dev2logU(val.u);
    p[1] = _canvasC.dc().dev2logV(val.v);
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
		for (int i = 0; i < _images.dim(); ++i)
		    _images[i].restore(in);
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
	if (_fileSelection.open(out))
	{
	    for (int i = 0; i < _images.dim(); ++i)
		_images[i].save(out, ImageBase::U_CHAR);
	}

	continuousShot();
      }
        break;
      
      case M_SaveAs:
      {
	stopContinuousShot();

	string		s = _cameraBase + ".conf";
	ofstream	out(s.c_str());
	if (out)
	{
	    out << _cameras[0]->delay() << ' ' << _cameras.dim() << endl;
	    for (int i = 0; i < _cameras.dim(); ++i)
		out << "0x" << setw(16) << setfill('0')
		    << hex << _cameras[i]->globalUniqueId() << ' '
		    << dec << *_cameras[i];
	}

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
	for (int i = 0; i < _cameras.dim(); ++i)
	    _cameras[i]
	      ->setFormatAndFrameRate(Ieee1394Camera::uintToFormat(id),
				      Ieee1394Camera::uintToFrameRate(val));
	_canvasC.resize(_cameras[0]->width(), _cameras[0]->height());
	_canvasH.resize(_cameras[0]->width(), _cameras[0]->height());
	_canvasV.resize(_cameras[0]->width(), _cameras[0]->height());
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
	for (int i = 0; i < _cameras.dim(); ++i)
	    _cameras[i]->setValue(id2feature(id), val);
        break;
      
      case c_WhiteBalance_UB:
	for (int i = 0; i < _cameras.dim(); ++i)
	    _cameras[i]
	      ->setWhiteBalance(val, _featureCmd.getValue(c_WhiteBalance_VR));
	break;
      case c_WhiteBalance_VR:
	for (int i = 0; i < _cameras.dim(); ++i)
	    _cameras[i]
	      ->setWhiteBalance(_featureCmd.getValue(c_WhiteBalance_UB), val);
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
	    for (int i = 0; i < _cameras.dim(); ++i)
		_cameras[i]->turnOn(feature);
	else
	    for (int i = 0; i < _cameras.dim(); ++i)
		_cameras[i]->turnOff(feature);
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
	    for (int i = 0; i < _cameras.dim(); ++i)
		_cameras[i]->setAutoMode(feature);
	else
	    for (int i = 0; i < _cameras.dim(); ++i)
	    {
		_cameras[i]->setManualMode(feature);
		if (feature == Ieee1394Camera::WHITE_BALANCE)
		    _cameras[i]->
		      setWhiteBalance(_featureCmd.getValue(c_WhiteBalance_UB),
				      _featureCmd.getValue(c_WhiteBalance_VR));
		else
		    _cameras[i]->
		      setValue(feature,
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
	    Array<Calib_t>
			calib = calibCamerasWithPlanes<Calib_t>(_data, true);
	    string	s = _cameraBase + ".calib";
	    ofstream	out(s.c_str());
	    if (!out)
		throw runtime_error("Cannot open the output calibration file!!");

	    for (int i = 0; i < _images.dim(); ++i)
	    {
		_images[i].P  = calib[i].P();
		_images[i].d1 = calib[i].d1();
		_images[i].d2 = calib[i].d2();
		    
		out << _images[i].P
		    << _images[i].d1 << ' ' << _images[i].d2 << '\n'
		    << endl;
	    }
	    _Fh = computeFMatrix(_images[0].P, _images[1].P);
	    if (_images.dim() > 2)
		_Fv = computeFMatrix(_images[0].P, _images[2].P);
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

      case Id_MouseButton1Drag:
#ifdef UseOverlay
	_canvasC.dc() << overlay << foreground(0);
	_canvasC.drawSelfEpipolarLine(_q, _Fh, _Fv);
	_canvasH.dc() << overlay << foreground(0);
	_canvasH.drawEpipolarLine(_q, _Fh);
	_canvasV.dc() << overlay << foreground(0);
	_canvasV.drawEpipolarLine(_q, _Fv);
#else
	_canvasC.display();
	_canvasH.display();
	_canvasV.display();
#endif
      case Id_MouseButton1Press:
#ifdef UseOverlay
	_canvasC.dc() << overlay << foreground(1);
	_canvasC.drawSelfEpipolarLine(p, _Fh, _Fv);
	_canvasC.dc() << underlay;
	_canvasH.dc() << overlay << foreground(1);
	_canvasH.drawEpipolarLine(p, _Fh);
	_canvasH.dc() << underlay;
	_canvasV.dc() << overlay << foreground(1);
	_canvasV.drawEpipolarLine(p, _Fv);
	_canvasV.dc() << underlay;
#else
	_timer.stop();
	_canvasC.dc() << foreground(BGR(0, 255, 0));
	_canvasC.drawSelfEpipolarLine(p, _Fh, _Fv);
	_canvasH.dc() << foreground(BGR(0, 255, 0));
	_canvasH.drawEpipolarLine(p, _Fh);
	_canvasV.dc() << foreground(BGR(0, 255, 0));
	_canvasV.drawEpipolarLine(p, _Fv);
#endif
	_q = p;
	break;

      case Id_MouseButton1Release:
#ifdef UseOverlay
	_canvasC.dc() << overlay << clear << underlay;
	_canvasH.dc() << overlay << clear << underlay;
	_canvasV.dc() << overlay << clear << underlay;
#else
	_canvasC.display();
	_canvasH.display();
	_canvasV.display();
	_timer.start(1);
#endif
	break;
    }
}

void
MyCmdWindow::tick()
{
    static int			nframes = 0;
    static struct timeval	start;
    countTime(nframes, start);

    for (int i = 0; i < _cameras.dim(); ++i)
	_cameras[i]->snap();
    for (int i = 0; i < _cameras.dim(); ++i)
    {
      	MyCanvasPane&	canvas = (i == 0 ? _canvasC :
				  i == 1 ? _canvasH : _canvasV);
	*_cameras[i] >> _images[i];
	canvas.display();
    }
}

void
MyCmdWindow::continuousShot()
{
    for (int i = 0; i < _cameras.dim(); ++i)
	_cameras[i]->continuousShot();
    _timer.start(1);
}
    
void
MyCmdWindow::stopContinuousShot()
{
    _timer.stop();
    for (int i = 0; i < _cameras.dim(); ++i)
	_cameras[i]->stopContinuousShot();
}
    
void
MyCmdWindow::extractCrosses()
{
    using namespace	std;
    
    try
    {
	for (int i = 0; i < _cameras.dim(); ++i)
	{
	    _markerDetector(_images[i], _crosses[i], true);
	    MyCanvasPane&	canvas = (i == 0 ? _canvasC :
					  i == 1 ? _canvasH : _canvasV);
	    canvas.display();
	}
	Confirm	confirm(*this);
	confirm << "Reference points extracted. Accept?";
	if (confirm.ok())
	{
	    for (int i = 0; i < _crosses.dim(); ++i)
		for (PairList::iterator iter  = _crosses[i].begin();
					iter != _crosses[i].end(); ++iter)
		    iter->first *= _pitch;
	    _data.push_back(_crosses);
	    for (int i = 0; i < _crosses.dim(); ++i)
		_crosses[i].clear();
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
    
Matrix<double>
MyCmdWindow::computeFMatrix(const Matrix<double>& P0, const Matrix<double>& P1)
{
    SVDecomposition<double>	svd(P0);
    Vector<double>		c = svd.Ut()[3];
    Matrix<double>		P0inv = svd.Ut()[0]%svd.Vt()[0]/svd[0]
				      + svd.Ut()[1]%svd.Vt()[1]/svd[1]
				      + svd.Ut()[2]%svd.Vt()[2]/svd[2];
    return (P1 * c).skew() * P1 * P0inv;
}

}
}
