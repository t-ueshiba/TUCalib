/*
 *  $Id: MyCmdWindow.h,v 1.3 2007-05-23 01:40:56 ueshiba Exp $
 */
#include <string>
#include "TU/v/App.h"
#include "TU/v/CmdWindow.h"
#include "TU/v/CmdPane.h"
#include "TU/v/FileSelection.h"
#include "TU/v/Timer.h"
#include "MyCanvasPaneTerse.h"
#include "calibCameras.h"

namespace TU
{
namespace v
{
typedef CameraWithDistortion	Calib_t;
    
/************************************************************************
*  class MyCmdWindow							*
************************************************************************/
class MyCmdWindow : public CmdWindow
{
  public:
    MyCmdWindow(App&				parentApp,
		const CmdDef			menuCmds[],
		const CmdDef			captureCmds[],
		const CmdDef			feautreCmds[],
		const Array<Ieee1394Camera*>&	cameras,
		const MarkerDetector&		markerDetector,
		double				pitch,
		const std::string&		cameraBase)		;

    virtual void	callback(CmdId, CmdVal)				;
    virtual void	tick()						;
    
  private:
    void		stopContinuousShot()				;
    void		continuousShot()				;
    void		extractCrosses()				;
    Matrix<double>	computeFMatrix(const Matrix<double>& P0,
				       const Matrix<double>& P1)	;
        
    const Array<Ieee1394Camera*>&	_cameras;
    Array<Image<u_char> >		_images;
    const MarkerDetector&		_markerDetector;
    PairListArray			_crosses;
    PairListArrayList			_data;
    Matrix<double>			_Fh;
    Matrix<double>			_Fv;
    Vector3i				_q;	// preveous cursor location
    const double			_pitch;
    const std::string&			_cameraBase;
    CmdPane				_menuCmd;
    CmdPane				_captureCmd;
    CmdPane				_featureCmd;
    FileSelection			_fileSelection;
    MyCanvasPane			_canvasC;
    MyCanvasPaneTerse			_canvasH;
    MyCanvasPaneTerse			_canvasV;
    Timer				_timer;
};
 
}
}
