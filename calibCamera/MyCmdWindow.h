/*
 *  $Id: MyCmdWindow.h,v 1.1.1.1 2007-05-23 05:55:43 ueshiba Exp $
 */
#include <string>
#include "TU/v/App.h"
#include "TU/v/CmdWindow.h"
#include "TU/v/CmdPane.h"
#include "TU/v/FileSelection.h"
#include "TU/v/Timer.h"
#include "MyCanvasPane.h"
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
    MyCmdWindow(App&			parentApp,
		const CmdDef		menuCmds[],
		const CmdDef		captureCmds[],
		const CmdDef		feautreCmds[],
		Ieee1394Camera&		camera,
		const MarkerDetector&	markerDetector,
		double			pitch,
		const char*		calibFile)			;

    virtual void	callback(CmdId, CmdVal)				;
    virtual void	tick()						;
    
  private:
    void		stopContinuousShot()				;
    void		continuousShot()				;
    void		extractCrosses()				;
        
    Ieee1394Camera&		_camera;
    Image<u_char>		_image;
    const MarkerDetector&	_markerDetector;
    PairList			_crosses;
    PairListList		_data;
    const double		_pitch;
    const char* const		_calibFile;
    CmdPane			_menuCmd;
    CmdPane			_captureCmd;
    CmdPane			_featureCmd;
    FileSelection		_fileSelection;
    MyCanvasPane		_canvas;
    Timer			_timer;
};
 
}
}
