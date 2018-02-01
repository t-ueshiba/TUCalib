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
 *  $Id: MyCmdWindow.h,v 1.12 2012-09-01 07:23:38 ueshiba Exp $
 */
#include <string>
#include "TU/v/App.h"
#include "TU/v/CmdWindow.h"
#include "TU/v/CmdPane.h"
#include "TU/v/Timer.h"
#include "TU/Ieee1394CameraArray.h"
#include "TU/HRP2++.h"
#include "calibHandEye.h"
#include "calibHandEyeGUI.h"
#include "Rectify.h"
#include "MarkerDetector.h"
#include "MyCanvasPane.h"

namespace TU
{
namespace v
{
/************************************************************************
*  class MyCmdWindow							*
************************************************************************/
class MyCmdWindow : public CmdWindow
{
  public:
    MyCmdWindow(HRP2&				  hrp2,
		App&				  parentApp,
		const Ieee1394CameraArray&	  cameras,
		const MarkerDetector::Parameters& params,
		const std::string&		  outputBase,
		bool				  horizontal,
		bool				  dump)			;

    virtual void	callback(CmdId, CmdVal)				;
    virtual void	tick()						;
    
  private:
    void	continuousShot()					;
    void	stopContinuousShot()					;
    void	setPoseAndPlay(const Matrix44d& pose)			;
    void	doCalibration(const Correses& correses)			;
    void	evalCalibration(const Correses& correses)	   const;
    bool	getTargetPosition3D(Point3d& X, u_int64_t captime) const;
    bool	detectTargets(Point2Array& targets, bool interactive)	;
    bool	epicheck(const Point2Array& targets)		   const;
    
    const Ieee1394CameraArray&		_cameras;
    Image<u_char>			_images[3];
    MarkerDetector			_markerDetector;
    bool				_refine;

    HRP2&				_hrp2;
    double				_duration;

  //! 校正点の(3D座標, カメラ台数分の2D座標)の最新データ
    Corres				_corres;	

  //! 全フレームにおける校正点の(3D座標, カメラ台数分の2D座標)のデータ
    Correses				_correses;

  //! 右手第６軸から見た校正点位置
    Vector4d				_X6;

    Array<Matrix44d>			_targetPoses;
    Array<Matrix44d>::const_iterator	_targetPose;

  //！ 画像間の基礎行列
    Array2<Array<Matrix33d> >		_F;
    
    const std::string&			_outputBase;
    const bool				_horizontal;
    const bool				_dump;
    CmdPane				_menuCmd;
    CmdPane				_captureCmd;
    CmdPane				_featureCmd;
    MyCanvasPane			_canvasC;
    MyCanvasPane			_canvasH;
    MyCanvasPane			_canvasV;
    Timer				_timer;
};
 
}
}
