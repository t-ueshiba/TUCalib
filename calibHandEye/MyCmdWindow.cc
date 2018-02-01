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
 *  $Id: MyCmdWindow.cc,v 1.17 2012-09-01 07:23:38 ueshiba Exp $
 */
#include <unistd.h>
#include <sys/time.h>
#include <sstream>
#include <iomanip>
#include "MyCmdWindow.h"
#include "TU/v/FileSelection.h"
#include "TU/v/Confirm.h"
#include "TU/v/Notify.h"

namespace TU
{
typedef Camera<IntrinsicWithDistortion<Intrinsic<double> > > projection_type;
    
namespace v
{
CmdDef*	createMenuCmds(Ieee1394Camera& camera)		;
CmdDef*	createCaptureCmds()				;
CmdDef*	createFeatureCmds(const Ieee1394Camera& camera)	;

/************************************************************************
*  static functions							*
************************************************************************/
inline int	min(int a, int b)	{return (a < b ? a : b);}
    
static void
countTime()
{
    static int		nframes = 0;
    static timeval	start;
    
    if (nframes == 10)
    {
	timeval	end;
	gettimeofday(&end, NULL);
	double	interval = (end.tv_sec  - start.tv_sec)
			 + (end.tv_usec - start.tv_usec) / 1.0e6;
      	std::cerr << nframes / interval << " frames/sec" << std::endl;
	nframes = 0;
    }
    if (nframes++ == 0)
	gettimeofday(&start, NULL);
}

static std::ostream&
printTime(std::ostream& out, u_int64_t time)
{
    u_int32_t	usec = time % 1000;
    u_int32_t	msec = (time / 1000) % 1000;
    u_int32_t	sec  = time / 1000000;
    return out << sec << '.' << msec << '.' << usec;
}

/************************************************************************
*  class MyCmdWindow							*
************************************************************************/
MyCmdWindow::MyCmdWindow(HRP2&					hrp2,
			 App&					parentApp,
			 const Ieee1394CameraArray&		cameras,
			 const MarkerDetector::Parameters&	params,
			 const std::string&			outputBase,
			 bool					horizontal,
    			 bool					dump)
    :CmdWindow(parentApp, "Multi-camera calibration",
	       Colormap::RGBColor, 256, 0, 0),
     _cameras(cameras),
     _images(),
     _markerDetector(params),
     _hrp2(hrp2),
     _duration(15.0),
     _corres(_cameras.size()),
     _correses(),
     _targetPoses(),
     _targetPose(_targetPoses.begin()),
     _F(_cameras.size(), _cameras.size()),
     _outputBase(outputBase),
     _horizontal(horizontal),
     _dump(dump),
     _menuCmd(*this, createMenuCmds(*_cameras[0])),
     _captureCmd(*this, createCaptureCmds()),
     _featureCmd(*this, createFeatureCmds(*_cameras[0])),
     _canvasC(*this, 
	      _cameras[min(0, cameras.size()-1)]->width(),
	      _cameras[min(0, cameras.size()-1)]->height(),
	      _images [min(0, cameras.size()-1)],
	      _corres.second[min(0, cameras.size()-1)]),
     _canvasH(*this,
	      _cameras[min(1, cameras.size()-1)]->width(),
	      _cameras[min(1, cameras.size()-1)]->height(),
	      _images [min(1, cameras.size()-1)],
	      _corres.second[min(1, cameras.size()-1)]),
     _canvasV(*this,
	      _cameras[min(2, cameras.size()-1)]->width(),
	      _cameras[min(2, cameras.size()-1)]->height(),
	      _images [min(2, cameras.size()-1)],
	      _corres.second[min(2, cameras.size()-1)]),
     _timer(*this, 0)
{
    using namespace	std;
    
    _menuCmd.place(0, 0, 2, 1);
    _captureCmd.place(0, 1, 1, 1);
    _featureCmd.place(1, 1, 1, 2);
    _canvasC.place(0, 3, 1, 1);
    _canvasH.place(1, 3, 1, 1);
    _canvasV.place(0, 2, 1, 1);
    show();

  // カメラからの画像にタイムスタンプを埋め込ませる．
    for (u_int i = 0; i < _cameras.size(); ++i)
	cameras[i]->embedTimestamp();
    
  // カメラキャリブレーションデータを読み込む．
    ifstream	in(_cameras.calibFile().c_str());
    if (!in)
	throw runtime_error("Cannot open the calibration parameter file!!");
    for (u_int i = 0; i < _cameras.size(); ++i)
    {
	in >> _images[i].P >> _images[i].d1 >> _images[i].d2;
	_images[i].resize(_cameras[i]->height(), _cameras[i]->width());
    }
    in.close();

    if (_dump)
    {
      // ムービーヘッダをstdoutに出力する．
	cout << 'M' << _cameras.size() << endl;
	for (u_int i = 0; i < _cameras.size(); ++i)
	    _images[i].saveHeader(cout);
    }

  // ロボットの把持中心と姿勢を読み込む．
    in.open((_cameras.fullName() + ".hrp2").c_str());
    if (!in)
	throw runtime_error("Cannot open the robot pose parameter file!!");
    in >> _X6 >> _targetPoses;
    _targetPose = _targetPoses.begin();
    in.close();

    cerr << "--- Target point ---\n" << _X6
	 << "\n--- Robot poses ---\n" << _targetPoses;
	
  // 右手の把持中心を校正点に設定
    Vector3d	graspOffset = _X6(0, 3);
    if (!hrp2.SetGraspCenter(0, graspOffset.data()))
	throw runtime_error("HRP2Client::SetGraspCenter() failed!!");

  // 画像間の基礎行列を設定
    for (u_int i = 0; i < _F.nrow(); ++i)
    {
	const Matrix34d&	P0 = _images[i].P;
	SVDecomposition<double>	svd(P0);
	Vector<double>		c0 = svd.Ut()[3];
	Matrix<double>		P0inv = svd.Ut()[0] % (svd.Vt()[0]/svd[0])
				      + svd.Ut()[1] % (svd.Vt()[1]/svd[1])
				      + svd.Ut()[2] % (svd.Vt()[2]/svd[2]);
	for (u_int j = 0; j < _F.ncol(); ++j)
	{
	    const Matrix34d&	P1 = _images[j].P;
	    _F[i][j] = (P1 * c0).skew() * P1 * P0inv;
	}
    }

    continuousShot();
}

void
MyCmdWindow::callback(CmdId id, CmdVal val)
{
    using namespace	std;

    try
    {
	switch (id)
	{
	  case M_Exit:
	    app().exit();
	    break;

	  case M_Open:
	  {
	    FileSelection	fileSelection(*this);
	    ifstream		in;
	    if (!fileSelection.open(in))
		throw runtime_error("Cannot open the robot pose parameter file!!");
	    in >> _X6;
	    cerr << "--- Target point ---\n" << _X6 << endl;
	    in >> _targetPoses;
	    cerr << "--- Robot poses ---\n" << _targetPoses;
	  }
	    break;

	  case M_Save:
	  {
	    stopContinuousShot();

	    FileSelection	fileSelection(*this);
	    ofstream		out;
	    if (fileSelection.open(out))
	    {
		for (u_int i = 0; i < _cameras.size(); ++i)
		    _images[i].save(out, ImageBase::U_CHAR);
	    }

	    continuousShot();
	  }
	    break;

	  case M_SaveAs:
	  {
	    stopContinuousShot();

	    ofstream	out(_cameras.configFile().c_str());
	    if (out)
	    {
		out << _cameras[0]->delay() << ' ' << _cameras.size() << endl;
		for (u_int i = 0; i < _cameras.size(); ++i)
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
	    for (u_int i = 0; i < _cameras.size(); ++i)
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
	    for (u_int i = 0; i < _cameras.size(); ++i)
		_cameras[i]->setValue(id2feature(id), val);
	    break;
      
	  case c_WhiteBalance_UB:
	    for (u_int i = 0; i < _cameras.size(); ++i)
		_cameras[i]
		    ->setWhiteBalance(val,
				      _featureCmd.getValue(c_WhiteBalance_VR));
	    break;
	  case c_WhiteBalance_VR:
	    for (u_int i = 0; i < _cameras.size(); ++i)
		_cameras[i]
		    ->setWhiteBalance(_featureCmd.getValue(c_WhiteBalance_UB),
				      val);
	    break;
      
	  case c_Brightness	 + OFFSET_ONOFF:
	  case c_AutoExposure    + OFFSET_ONOFF:
	  case c_Sharpness	 + OFFSET_ONOFF:
	  case c_WhiteBalance_UB + OFFSET_ONOFF:
	  case c_WhiteBalance_VR + OFFSET_ONOFF:
	  case c_Hue		 + OFFSET_ONOFF:
	  case c_Saturation	 + OFFSET_ONOFF:
	  case c_Gamma		 + OFFSET_ONOFF:
	  case c_Shutter	 + OFFSET_ONOFF:
	  case c_Gain		 + OFFSET_ONOFF:
	  case c_Iris		 + OFFSET_ONOFF:
	  case c_Focus		 + OFFSET_ONOFF:
	  case c_Zoom		 + OFFSET_ONOFF:
	  {
	    Ieee1394Camera::Feature feature = id2feature(id - OFFSET_ONOFF);
	    if (val)
		for (u_int i = 0; i < _cameras.size(); ++i)
		    _cameras[i]->turnOn(feature);
	    else
		for (u_int i = 0; i < _cameras.size(); ++i)
		    _cameras[i]->turnOff(feature);
	  }
	    break;
      
	  case c_Brightness	 + OFFSET_AUTO:
	  case c_AutoExposure    + OFFSET_AUTO:
	  case c_Sharpness	 + OFFSET_AUTO:
	  case c_WhiteBalance_UB + OFFSET_AUTO:
	  case c_WhiteBalance_VR + OFFSET_AUTO:
	  case c_Hue		 + OFFSET_AUTO:
	  case c_Saturation	 + OFFSET_AUTO:
	  case c_Gamma		 + OFFSET_AUTO:
	  case c_Shutter	 + OFFSET_AUTO:
	  case c_Gain		 + OFFSET_AUTO:
	  case c_Iris		 + OFFSET_AUTO:
	  case c_Focus		 + OFFSET_AUTO:
	  case c_Zoom		 + OFFSET_AUTO:
	  {
	    Ieee1394Camera::Feature feature = id2feature(id - OFFSET_AUTO);
	    if (val)
		for (u_int i = 0; i < _cameras.size(); ++i)
		    _cameras[i]->setAutoMode(feature);
	    else
		for (u_int i = 0; i < _cameras.size(); ++i)
		{
		    _cameras[i]->setManualMode(feature);
		    if (feature == Ieee1394Camera::WHITE_BALANCE)
			_cameras[i]->setWhiteBalance(
			    _featureCmd.getValue(c_WhiteBalance_UB),
			    _featureCmd.getValue(c_WhiteBalance_VR));
		    else
			_cameras[i]->
			    setValue(feature,
				     _featureCmd.getValue(id - OFFSET_AUTO));
		}
	  }
	    break;

	  case c_NextPosition:
	    if (val)
	    {
		if (_targetPose == _targetPoses.begin())
		    _hrp2.go_clothinit();

		setPoseAndPlay(*_targetPose++);
	    }
	    break;
	}
    }
    catch (exception& err)
    {
	Notify	notify(*this);
	notify << err.what();
	notify.show();
    }
}

void
MyCmdWindow::tick()
{
    using namespace	std;
    
    countTime();
    
  // 画像をsnapしてcaptureし，さらに撮影時刻を取得する．
    for (u_int i = 0; i < _cameras.size(); ++i)
	_cameras[i]->snap();
    for (u_int i = 0; i < _cameras.size(); ++i)
	*_cameras[i] >> _images[i];
    u_int64_t	captime = _cameras[0]->getTimestamp();	// 撮影時刻を取得

    if (_captureCmd.getValue(c_NextPosition))		// 動作中ならば...
    {
#ifdef TRACK
	if (!_hrp2.isCompleted())	// GenerateMotion が未完了ならば...
	{
	  //cerr << "GenerateMotion completed!" << endl;
	    
	  // 校正点の3D/2D座標の取得に成功し，かつ直前の校正点の3D位置と
	  // 25mm以上離れていれば，新しいデータとして受理する．
	    Point3d	X;
	    if (getTargetPosition3D(X, captime)	     &&		// 3D座標
		detectTargets(_corres.second, false) &&		// 2D座標
		(_corres.first.sqdist(X) > 25*25))
	    {
		_corres.first = X;
		_correses.push_back(_corres);	// 新しい3D-2D座標のペアを登録

		if (_dump)
		    for (u_int i = 0; i < _cameras.size(); ++i)
			_images[i].saveData(std::cout);
	    }
	}
	else				// GenerateMotion が完了していれば...
	{
	    if (_targetPose != _targetPoses.end())  // 終点に未到達ならば...
	    {
		if (_targetPose == _targetPoses.begin())
		    _hrp2.go_clothinit();

		setPoseAndPlay(*_targetPose++);
	    }
	    else				// 終点に到達していれば...
	    {
		if (_dump)
		{
		    ofstream	out((_outputBase + ".dat").c_str());
		    if (!out)
			throw runtime_error("Failed to open output file for saving the list of point data!!");
		    out << _correses;		// 対応点データを保存する．
		}

		doCalibration(_correses);	// キャリブレーションを実行

		_targetPose = _targetPoses.begin();	// 目標点を始点に
		_correses.clear();
		_hrp2.go_clothinit();

	      // 校正点の直前の2D位置を無効化
		for (u_int i = 0; i < _corres.second.size(); ++i)
		    _corres.second[i] = Point2f(0, 0);
		
		_captureCmd.setValue(c_NextPosition, 0);
	    }
	}
#else
      // ロボットが動作中かつハンドが目標点に到達していれば，
      // NextPositionボタンをoffにする．
	if (_hrp2.isCompleted())	// GenerateMotion が完了していれば...
	{
	  // 校正点の3D/2D座標の取得に成功したら，それを登録する．
	    if (getTargetPosition3D(_corres.first, captime) &&
		detectTargets(_corres.second, true))
		_correses.push_back(_corres);
		
	    if (_targetPose == _targetPoses.end())  // 終点に到達していれば...
	    {
		doCalibration(_correses);	// キャリブレーションを実行

		_targetPose = _targetPoses.begin();
		_correses.clear();
		_hrp2.go_clothinit();
	    }

	  // 校正点の直前の2D位置を無効化
	    for (u_int i = 0; i < _corres.second.size(); ++i)
		_corres.second[i] = Point2f(0, 0);

	    _captureCmd.setValue(c_NextPosition, 0);
	}
#endif
    }
    
  // 画像を表示する．
    _canvasC.repaintUnderlay();
    _canvasH.repaintUnderlay();
    if (_cameras.size() > 2)
	_canvasV.repaintUnderlay();
}

void
MyCmdWindow::continuousShot()
{
    for (u_int i = 0; i < _cameras.size(); ++i)
	_cameras[i]->continuousShot();
    _timer.start(1);
}
    
void
MyCmdWindow::stopContinuousShot()
{
    _timer.stop();
    for (u_int i = 0; i < _cameras.size(); ++i)
	_cameras[i]->stopContinuousShot();
}

void
MyCmdWindow::setPoseAndPlay(const Matrix44d& pose)
{
    using namespace	std;
    
  // ロボットに次の校正点の姿勢を与える．
    cerr << dec;
    cerr << "--- Target-" << &pose - _targetPoses.begin()
	 << '/' << _targetPoses.size() - 1 << " ---\n"
	 << pose;
    if (!_hrp2.SetTargetPose(0, const_cast<double*>(pose.data()), _duration))
    {
	cerr << "HRP2Client::SetTargetPose(): failed!" << endl;
	return;
    }

  // 校正点へ到達する軌道を計算して移動する(ブロックされない)．
    _hrp2.GenerateMotion(false);
}

void
MyCmdWindow::doCalibration(const Correses& correses)
{
    using namespace	std;

    typedef Rigidity<projection_type::matrix44_type>	rigidity_type;
    
  // 各画像のキャリブレーションデータからカメラモデルを生成する．
    Array<projection_type>	projections(_cameras.size());
    for (u_int i = 0; i < projections.size(); ++i)
    {
	projections[i].setProjection(_images[i].P);
	projections[i].setDistortion(_images[i].d1, _images[i].d2);
    }
    
  // triangulation誤差が大きな対応点を除去する．
  /*
    for (Correses::iterator corres  = correses.begin();
			    corres != correses.end(); ++corres)
    {
	Point3d	X;
	if (triangulate(X, projections.begin(), projections.end(),
			corres->second.begin(), true) < 1.0)
	    correses.erase(corres);
    }
  */
  // ワールド座標系 -> カメラ座標系間の剛体変換を求める。
    const rigidity_type&
	Dcw = calibHandEye(projections.begin(), projections.end(),
			   correses.begin(), correses.end(),
			   false);

  // 各カメラをワールド座標系における記述に改める。
    for (u_int i = 0; i < projections.size(); ++i)
    {
	_images[i].P  = projections[i].P() * Dcw;
	_images[i].d1 = projections[i].d1();
	_images[i].d2 = projections[i].d2();
    }

  // ロボットから読み取った3D座標とtriangulationによって求めた3D座標を比較．
  //evalCalibration(correses);
	    
  // キャリブレーション結果を保存する。 
    ofstream	out(_cameras.calibFile().c_str());
    if (!out)
	throw runtime_error("Cannot open the output calibration file!!");
    for (u_int i = 0; i < _cameras.size(); ++i)
    {
	out << _images[i].P << _images[i].d1 << ' ' << _images[i].d2 << '\n'
	    << endl;
    }
}

void
MyCmdWindow::evalCalibration(const Correses& correses) const
{
    using namespace	std;
    
    Array<projection_type>	projections(_cameras.size());
    for (u_int i = 0; i < projections.size(); ++i)
    {
	projections[i].setProjection(_images[i].P);
	projections[i].setDistortion(_images[i].d1, _images[i].d2);
    }
    
  // ロボットから読み取った3D座標とtriangulationによって計算した3D座標を比較．
    double	totalErr = 0.0;
    for (Correses::const_iterator corres = correses.begin();
	 corres != correses.end(); ++corres)
    {
	Point3d	X;
	triangulate(X, projections.begin(), projections.end(),
		    corres->second.begin(), true);
	double	sqErr = X.sqdist(corres->first);
	cerr << "true: [";
	corres->first.put(cerr);
	cerr << "], measured: [";
	X.put(cerr);
	cerr << "], err: " << sqrt(sqErr) << endl;
	totalErr += sqErr;
    }
    cerr << "RMS err: " << sqrt(totalErr / distance(correses.begin(),
						    correses.end()))
	 << endl;
}

bool
MyCmdWindow::getTargetPosition3D(Point3d& X, u_int64_t captime) const
{
    using namespace	std;
    
    HRP2::TimedPose	Dw6;		// 第6軸 -> ワールドの変換
    if (!_hrp2.GetRealPose("RARM_JOINT6", captime, Dw6))
    {
	cerr << "No pose available at ";
	printTime(cerr, captime) << endl;

	return false;
    }
    else
    {
      //printTime(cerr, captime) << " is approximated by ";
      //printTime(cerr, Dw6.t) << endl;
    }
    
    Matrix44d	D6g = Matrix44d::I(4);	// グリッパ -> 第6軸の変換
    D6g[0][3] = _X6[0];
    D6g[1][3] = _X6[1];
    D6g[2][3] = _X6[2];
    Matrix44d	Dwg = Dw6 * D6g;
    double	scale = 1000.0 / Dwg[3][3];	// meter -> mm
    X[0] = scale * Dwg[0][3];
    X[1] = scale * Dwg[1][3];
    X[2] = scale * Dwg[2][3];

    return true;
}
    
bool
MyCmdWindow::detectTargets(Point2Array& targets, bool interactive)
{
    using namespace	std;

    bool	success = true;

  // 各カメラについてターゲット点を検出
    for (u_int i = 0; i < _cameras.size(); ++i)
    {
      // ターゲット点の探索ウィンドウを設定
	const int	win = 100;		  // 探索ウィンドウサイズ
	Point2f		origin(0, 0);		  // 探索ウィンドウの左上隅
	int		w = _images[i].width(),	  // 探索ウィンドウの幅
			h = _images[i].height();  // 探索ウインドウの高さ
	if (!interactive && targets[i] != origin)
	{
	    origin[0] = std::min(std::max(0, int(targets[i][0]) - win/2),
				 w - win);
	    origin[1] = std::min(std::max(0, int(targets[i][1]) - win/2),
				 h - win);
	    w = win;
	    h = win;
	}

      // ターゲット点を検出
	try
	{
	    _markerDetector(_images[i](origin[0], origin[1], w, h),
			    targets[i], _horizontal);
	    targets[i] += origin;	// ウィンドウ座標系 -> 画像座標系
	}
	catch (exception& err)
	{
	    success = false;			// 検出失敗の印をつける
	    targets[i] = Point2f(0, 0);		// 2D座標にも印をつける
	    
	    if (interactive)
	    {
		Notify	notify(*this);
		notify << '[' << i << ']' << err.what();
		notify.show();
	    }
	    else
		cerr << '[' << i << ']' << err.what() << endl;
	}
    }

    if (!success)		// いずれかの画像で検出に失敗していたら...
	return false;		// falseを返す．
    
    if (interactive)
    {
	Confirm	confirm(*this);
	confirm << "Targets extracted. Accept?";
	return confirm.ok();
    }

    return epicheck(targets);
}

bool
MyCmdWindow::epicheck(const Point2Array& targets) const
{
    for (u_int i = 0; i < targets.size(); ++i)
	for (u_int j = i + 1; j < targets.size(); ++j)
	{
	    LineP2d	l1 = _F[i][j] * targets[i].homogeneous();
	    if (l1.sqdist(targets[j]) > 2*2)
		return false;
	}
    
    return true;
}

}
}
