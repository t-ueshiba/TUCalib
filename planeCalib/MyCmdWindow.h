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
 *  $Id$  
 */
#include <sstream>
#include <iomanip>
#include "TU/v/App.h"
#include "TU/v/CmdWindow.h"
#include "TU/v/CmdPane.h"
#include "TU/v/Timer.h"
#include "TU/v/FileSelection.h"
#include "TU/v/Confirm.h"
#include "TU/v/Notify.h"
#include "TU/Camera++.h"
#include "CameraCalibrator.h"
#include "MyCanvasPaneTerse.h"
#include "planeCalibGUI.h"

namespace TU
{
typedef Camera<IntrinsicWithDistortion<Intrinsic<double> > >	Calib_t;
    
namespace v
{
/************************************************************************
*  class MyCmdWindow							*
************************************************************************/
template <class CAMERAS, class PIXEL>
class MyCmdWindow : public CmdWindow
{
  private:
    typedef typename CAMERAS::value_type	camera_type;
    
  public:
    typedef MarkerDetector::CorresList		CorresList;
    typedef MarkerDetector::CorresListArray	CorresListArray;
    typedef MarkerDetector::CorresListArrayList	CorresListArrayList;
    
  public:
    MyCmdWindow(App&					parentApp,
		CAMERAS&				cameras,
		const MarkerDetector::Parameters&	params,
		double					pitch,
		bool					horizontal)	;

    virtual void	callback(CmdId, CmdVal)				;
    virtual void	tick()						;
    
  private:
    void		stopContinuousShot()				;
    void		continuousShot()				;
    void		extractCrosses()				;
    void		calibrate()					;
    Matrix33d		computeFMatrix(const Matrix34d& P0,
				       const Matrix34d& P1)		;
        
    CAMERAS&				_cameras;
    const MarkerDetector::Parameters&	_params;
    Array<Image<PIXEL> >		_images;
    CorresListArray			_crosses;
    CorresListArrayList			_data;
    Matrix33d				_Fh;
    Matrix33d				_Fv;
    Vector3i				_q;	// preveous cursor location
    const double			_pitch;
    const bool				_horizontal;
    CmdPane				_menuCmd;
    CmdPane				_captureCmd;
    CmdPane				_featureCmd;
    MyCanvasPane			_canvasC;
    MyCanvasPaneTerse			_canvasH;
    MyCanvasPaneTerse			_canvasV;
    Timer				_timer;
};

template <class CAMERAS, class PIXEL>    
MyCmdWindow<CAMERAS, PIXEL>::MyCmdWindow(
    App& parentApp, CAMERAS& cameras, const MarkerDetector::Parameters& params,
    double pitch, bool horizontal)
    :CmdWindow(parentApp, "Multi-camera calibration",
#ifdef UseOverlay
	       Colormap::IndexedColor, 64, 3, 1
#else
	       Colormap::RGBColor, 256, 0, 0
#endif
	      ),
     _cameras(cameras),
     _images(_cameras.size()),
     _params(params),
     _crosses(_cameras.size()),
     _data(),
     _Fh(3, 3),
     _Fv(3, 3),
     _q(),
     _pitch(pitch),
     _horizontal(horizontal),
     _menuCmd(*this, createMenuCmds(_cameras[0])),
     _captureCmd(*this, createCaptureCmds()),
     _featureCmd(*this, createFeatureCmds(_cameras[0], _cameras.size())),
     _canvasC(*this, 
	      _cameras[std::min<size_t>(0, cameras.size()-1)].width()/2,
	      _cameras[std::min<size_t>(0, cameras.size()-1)].height()/2,
	      _images [std::min<size_t>(0, cameras.size()-1)],
	      _crosses[std::min<size_t>(0, cameras.size()-1)]),
     _canvasH(*this,
	      _cameras[std::min<size_t>(1, cameras.size()-1)].width()/2,
	      _cameras[std::min<size_t>(1, cameras.size()-1)].height()/2,
	      _images [std::min<size_t>(1, cameras.size()-1)],
	      _crosses[std::min<size_t>(1, cameras.size()-1)]),
     _canvasV(*this,
	      _cameras[std::min<size_t>(2, cameras.size()-1)].width()/2,
	      _cameras[std::min<size_t>(2, cameras.size()-1)].height()/2,
	      _images [std::min<size_t>(2, cameras.size()-1)],
	      _crosses[std::min<size_t>(2, cameras.size()-1)]),
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

    refreshFeatureCmds(_cameras, _featureCmd);
    
    _canvasC.setZoom(0.5);
    _canvasH.setZoom(0.5);
    _canvasV.setZoom(0.5);

    ifstream	in(_cameras.calibFile().c_str());
    if (in)			// If a calibration file exists,...
    {
	for (int i = 0; i < _images.size(); ++i)
	    in >> _images[i].P
	       >> _images[i].d1 >> _images[i].d2;  // Read calibration data.
    }
    else			// If no calibration files, set default values.
    {
	if (_images.size() > 1)
	    _images[1].P[0][3] = -1.0;
	if (_images.size() > 2)
	    _images[2].P[1][3] = -1.0;
    }
    if (_images.size() > 1)
	_Fh = computeFMatrix(_images[0].P, _images[1].P);
    if (_images.size() > 2)
	_Fv = computeFMatrix(_images[0].P, _images[2].P);
    
    continuousShot();
}

template <class CAMERAS, class PIXEL> void
MyCmdWindow<CAMERAS, PIXEL>::callback(CmdId id, CmdVal val)
{
    using namespace	std;

    Vector3i	p;
    p[0] = _canvasC.dc().dev2logU(val.u());
    p[1] = _canvasC.dc().dev2logV(val.v());
    p[2] = 1;

    try
    {
	if (setFormat(_cameras, id, val, *this))
	{
	    return;
	}
	else if (setFeature(_cameras, id, val, _featureCmd))
	{
	    return;
	}
	
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
		FileSelection	fileSelection(*this);
		ifstream	in;
		if (fileSelection.open(in))
		{
		    for (int i = 0; i < _images.size(); ++i)
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

	    FileSelection	fileSelection(*this);
	    ofstream		out;
	    if (fileSelection.open(out))
	    {
		for (int i = 0; i < _images.size(); ++i)
		    _images[i].save(out);
	    }

	    continuousShot();
	  }
	    break;
      
	  case M_SaveAs:
	  {
	    stopContinuousShot();

	    ofstream	out(_cameras.configFile().c_str());
	    if (out)
		out << _cameras;

	    continuousShot();
	  }
	    break;

	  case c_Extract:
	    stopContinuousShot();
	    extractCrosses();
	    continuousShot();
	    break;

	  case c_Calibrate:
	    cout << _data;
	    calibrate();
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
	    _canvasC.repaintUnderlay();
	    _canvasH.repaintUnderlay();
	    _canvasV.repaintUnderlay();
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
	    _canvasC.repaintUnderlay();
	    _canvasH.repaintUnderlay();
	    _canvasV.repaintUnderlay();
	    _timer.start(1);
#endif
	    break;
	}
    }
    catch (const std::exception& err)
    {
	std::cerr << err.what() << std::endl;
    }
}

template <class CAMERAS, class PIXEL> void
MyCmdWindow<CAMERAS, PIXEL>::tick()
{
    countTime();

    for (int i = 0; i < _cameras.size(); ++i)
	_cameras[i].snap();
    for (int i = 0; i < _cameras.size(); ++i)
    {
      	MyCanvasPane&	canvas = (i == 0 ? _canvasC :
				  i == 1 ? _canvasH : _canvasV);
	_cameras[i] >> _images[i];
	canvas.repaintUnderlay();
    }
    if (_cameras.size() < 3)
    {
	_canvasV.dc() << clear;

	if (_cameras.size() < 2)
	    _canvasH.dc() << clear;
    }
}

template <class CAMERAS, class PIXEL> void
MyCmdWindow<CAMERAS, PIXEL>::continuousShot()
{
    for (int i = 0; i < _cameras.size(); ++i)
	_cameras[i].continuousShot(true);
    _timer.start(1);
}
    
template <class CAMERAS, class PIXEL> void
MyCmdWindow<CAMERAS, PIXEL>::stopContinuousShot()
{
    _timer.stop();
    for (int i = 0; i < _cameras.size(); ++i)
	_cameras[i].continuousShot(false);
}
    
template <class CAMERAS, class PIXEL> void
MyCmdWindow<CAMERAS, PIXEL>::extractCrosses()
{
    using namespace	std;
    
    try
    {
	MarkerDetector	markerDetector(_params);
	for (int i = 0; i < _cameras.size(); ++i)
	{
	    markerDetector(_images[i], _crosses[i], _horizontal);
	    MyCanvasPane&	canvas = (i == 0 ? _canvasC :
					  i == 1 ? _canvasH : _canvasV);
	    canvas.repaintUnderlay();
	}
	Confirm	confirm(*this);
	confirm << "Reference points extracted. Accept?";
	if (confirm.ok())
	{
#ifdef _DEBUG
	    ostringstream	s;
	    s << "calibImages-" << _data.size() << ".pbm";
	    ofstream		out(s.str().c_str());
	    if (out)
		for (int i = 0; i < _images.size(); ++i)
		    _images[i].save(out);
#endif
	    for (int i = 0; i < _crosses.size(); ++i)
		for (CorresList::iterator iter  = _crosses[i].begin();
					  iter != _crosses[i].end(); ++iter)
		    iter->first *= _pitch;
	    _data.push_back(_crosses);
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

    for (int i = 0; i < _crosses.size(); ++i)
	_crosses[i].clear();
}

template <class CAMERAS, class PIXEL> void
MyCmdWindow<CAMERAS, PIXEL>::calibrate()
{
    using namespace	std;
    
    try
    {
	CameraCalibrator<double>	cameraCalibrator;
	Array<Calib_t>			calibs;
	cameraCalibrator.planeCalib(_data.begin(), _data.end(), calibs,
				    _menuCmd.getValue(c_CommonCenters), true);
	Notify	notify(*this);
	notify << "Reprojection error: "
	       << cameraCalibrator.reprojectionError() << "(pix.)";
	notify.show();

	ofstream	out(_cameras.calibFile().c_str());
	if (!out)
	    throw runtime_error("Cannot open the output calibration file!!");

	for (int i = 0; i < _images.size(); ++i)
	{
	    _images[i].P  = calibs[i].P();
	    _images[i].d1 = calibs[i].d1();
	    _images[i].d2 = calibs[i].d2();
		    
	    out << _images[i].P
		<< _images[i].d1 << ' ' << _images[i].d2 << '\n'
		<< endl;
	}
	if (_images.size() > 1)
	    _Fh = computeFMatrix(_images[0].P, _images[1].P);
	if (_images.size() > 2)
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
}
    
template <class CAMERAS, class PIXEL> Matrix33d
MyCmdWindow<CAMERAS, PIXEL>::computeFMatrix(const Matrix34d& P0,
					    const Matrix34d& P1)
{
    SVDecomposition<double>	svd(P0);
    const Vector<double>&	c = svd.Ut()[3];
    const Matrix<double>&	P0inv = svd.Ut()[0]%svd.Vt()[0]/svd[0]
				      + svd.Ut()[1]%svd.Vt()[1]/svd[1]
				      + svd.Ut()[2]%svd.Vt()[2]/svd[2];
    return skew(P1 * c) * P1 * P0inv;
}
 
}
}
