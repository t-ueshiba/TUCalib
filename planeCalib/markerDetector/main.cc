/*
 *  $Id$
 */
/*!
  \mainpage	markerDetector - キャリブレーションパターンを撮影した画像から参照点の投影像(マーカ点)を検出するプログラム
  \anchor	markerDetector

  \section copyright 著作権
  平成14-19年（独）産業技術総合研究所 著作権所有

  創作者：植芝俊夫

  本プログラムは（独）産業技術総合研究所の職員である植芝俊夫が創作し，
  （独）産業技術総合研究所が著作権を所有する秘密情報です．著作権所有
  者による許可なしに本プログラムを使用，複製，改変，第三者へ開示する
  等の行為を禁止します．
   
  このプログラムによって生じるいかなる損害に対しても，著作権所有者お
  よび創作者は責任を負いません。

  Copyright 2002-2007.
  National Institute of Advanced Industrial Science and Technology (AIST)

  Creator: Toshio UESHIBA

  [AIST Confidential and all rights reserved.]
  This program is confidential. Any using, copying, changing or
  giving any information concerning with this program to others
  without permission by the copyright holder are strictly prohibited.

  [No Warranty.]
  The copyright holder or the creator are not responsible for any
  damages caused by using this program.

  \section functions 機能
  本プログラムの基本的な機能は，キャリブレーションパターンが描かれた 参
  照平面を撮影した画像を入力し，それからパターン上の格子点(参照点と呼ぶ)
  が投影された点(マーカ点)を検出して，その座標をもとの参照点の座標とと
  もに出力することである．

  - [<b>機能</b>] 参照平面の画像を標準入力から入力し，参照点とその投影
	像(マーカ点)の座標を標準出力に出力する．
  - [<b>入力</b>] 参照平面を撮影した1枚またはそれ以上のpbm形式の画像
	(グレースケールまたはカラー)．
  - [<b>出力</b>] 対応する参照点とマーカ点の2次元座標対のリストであり，
	各画像毎に次のような1行の文字列で出力される:
	\f[
	  \begin{array}{ccccccccccccc}
	   x_1 & y_1 & u_1 & v_1 & x_2 & y_2 & u_2 & v_2 &
	    \cdots & x_N & y_N & u_N & v_N
	  \end{array}
	\f]
	各数値間は空白によって区切られている．4つの数値が1組になってお
	り，例えば'\f$x_1~~y_1~~u_1~~v_1\f$'は参照平面に固定された2次元
	座標系における参照点\f$(x_1, y_1)\f$がマーカ点\f$(u_1, v_1)\f$
	として画像上で観測されたことを表している．よって，N個のマーカ点
	が検出されれば，4N個の数値が1行に出力される．\n
	I枚の画像を入力すれば，I行のデータが出力される．ただし，検出さ
	れるマーカ点の個数は画像によって異なるので，1行に含まれる数値の
	個数は一定ではない．\n
	複数の参照平面間の区切りを示すため，すべての行が出力された後に
	空行がもう1つ出力される．
  - [<b>コマンド呼び出しの形式</b>] 入力画像が1枚の場合は
	\verbatim
    markerDetector [options] < img.pbm > markers.dat
	\endverbatim
	複数の場合は
	\verbatim
    cat img0.pbm img1.pbm ... | markerDetector [options] > markers.dat
	\endverbatim
	とすることにより，検出されたマーカ点の座標が
	<tt>markers.dat</tt>に書き込まれる．
  - [<b>コマンドオプション</b>]
	本コマンドは，以下のオプションを持つ:
    - [<tt>-p pitch</tt>] キャリブレーションパターンの参照点の間隔を指
	    定する．すなわち，パターン原点から直近の4つの参照点までの距
	    離であり，これはパターンを構成する正方形の対角線の長さの半
	    分に等しい．(default: 30)
    - [<tt>-v</tt>] 検出されたマーカ点の座標ではなく，それらを入力画像
	  に重畳表示したカラー画像を標準出力に出力する．検出されたマー
	  カ点が正しいものであるか確認するために用いる．\n
	  なお，複数の画像を入力した場合には，それらに対応する複
	  数の重畳画像が連続して出力される．
    - [<tt>-o</tt>] 全てのマーカ点ではなく，原点のみを検出する．
    - [<tt>-C cropSize</tt>] マーカ点の候補を検出するために，画像の各点
	  に十字印と×印のテンプレートが適用される．本オプションは，そ
	  のテンプレートのサイズを指定する．(default: 7)
    - [<tt>-S strengthRatio</tt>] マーカ点の候補を検出するために，画像
	  の各点にテンプレートを適用し，その反応強度が極大かつ最大反応
	  強度に対してある割合以上になる点のみが残される．本オプション
	  は，0と1の間でこの割合を指定する．(default: 0.5)
    - [<tt>-D distTh</tt>] 参照点からマーカ点への変換が「射影変換+画像
	  中央を中心とした放射歪曲」なるモデルに従うかによって最終的な
	  マーカ点を決める．本オプションは，このモデルによる予測位置と
	  実際の検出位置の距離の閾値を指定する．(default: 5.0)
    - [<tt>-e</tt>] マーカ位置をサブピクセルで求めるために，(1)輝度パター
	  ンに双曲放物面を当てはめて鞍点を検出する, (2) ×印を成す輝度
	  パターンに2本の直線を当てはめて交点を検出する, の2つの方法を
	  実装している．本オプションを指定すると後者(2)のアルゴリズムが
	  実行される．
    - [<tt>-W winSize</tt>] サブピクセル位置決めにおける双曲放物面当て
	  はめ／2直線当てはめのウィンドウサイズを指定する．(default:
	  7)
    - [<tt>-A alpha</tt>] エッジ点検出または平滑化のための正のスムーシ
	  ング係数．(default: 1.0，小さいほど平滑化の度合いが大きい)
    - [<tt>-L lowTh</tt>] エッジ点検出のためのヒステリシス閾値処理にお
	  ける低い閾値を指定する．{\tt -e}オプションを指定した場合のみ
	  有効．(default: 2.0)
    - [<tt>-H highTh</tt>] エッジ点検出のためのヒステリシス閾値処理にお
	  ける高い閾値を指定する．{\tt -e}オプションを指定した場合のみ
	  有効．(default: 5.0)

  \file		main.cc
  \brief	メイン関数
*/
#include <unistd.h>
#ifdef WIN32
#  include <io.h>
#  include <fcntl.h>
#endif
#include "MarkerDetector.h"
#include "debug.h"

#ifndef EOF
#  define EOF	(-1)
#endif

//! 本プログラムで定義されたクラスおよび関数を収める名前空間
namespace TU
{
/************************************************************************
*  static constants							*
************************************************************************/
static const double		DefaultPitch		= 30.0;

/************************************************************************
*  static functions							*
************************************************************************/
//! コマンドの使い方を表示する．
static void
usage(const char* s)
{
    using namespace	std;

    MarkerDetector::Parameters	params;

    cerr << "\nDetect markers from images of a calibration board.\n"
	 << endl;
    cerr << " Usage: " << s << " [options] < image.pbm > markers.dat\n"
	 << "        " << s << " -v [options] < image.pbm > verify.pbm\n"
	 << endl;
    cerr << " General options.\n"
	 << "  -p pitch:         pitch of the grid pattern. (default: "
	 << DefaultPitch << ")\n"
	 << "  -v:               output images with detected markers "
	 << "superimposed.\n"
	 << "  -o:               detect the origin of the grid pattern only.\n"
	 << endl;
    cerr << " Options for marker detection.\n"
	 << "  -R:               90 deg. rotated calib. pattern "
	 << "(default: off)\n"
	 << "  -C cropSize:      size of cropping square for template "
	 << "matching.\n"
	 << "                      (default: "
	 << params.cropSize
	 << ")\n"
	 << "  -S strengthRatio: ratio to the highest template matching score.\n"
	 << "                      (default: "
	 << params.strengthRatio
	 << ")\n"
	 << "  -D distTh:        distance threshold for projective "
	 << "transformation.\n"
	 << "                      (default: "
	 << std::sqrt(params.sqdistTh)
	 << ")\n"
	 << "  -e:               refine marker locations by detecting "
	 << "edge intersections.\n"
	 << "                      (default: off)\n"
	 << "  -W winSize:       window size for refining marker location.\n"
	 << "                      (default: "
	 << params.winSize
	 << ")\n"
	 << "  -A alpha:         size of Deriche edge detector for refining "
	 << "marker locations.\n"
	 << "                      (default: "
	 << params.alpha
	 << ")\n"
	 << "  -L lowTh:         lower threshold for edge detection.\n"
	 << "                      (default: "
	 << params.lowTh
	 << ")\n"
	 << "  -H highTh:        higher threshold for edge detection.\n"
	 << "                      (default: "
	 << params.highTh
	 << ")\n"
	 << endl;
    cerr << " Other options.\n"
	 << "  -h:               print this.\n"
	 << endl;
}
 
}
/************************************************************************
*  global functions							*
************************************************************************/
//! メイン関数
int
main(int argc, char* argv[])
{
    using namespace	std;
    using namespace	TU;

    double			pitch = DefaultPitch;
    bool			verify = false,
				origin = false, horizontal = true;
    MarkerDetector::Parameters	params;
    extern char*		optarg;
    for (int c; (c = getopt(argc, argv, "p:voRC:S:D:W:A:L:H:eh")) != EOF; )
	switch (c)
	{
	  case 'p':
	    pitch = atof(optarg);
	    break;
	  case 'v':
	    verify = true;
	    break;
	  case 'o':
	    origin = true;
	    break;
	  case 'R':
	    horizontal = false;
	    break;
	  case 'C':
	    params.cropSize = atoi(optarg);
	    break;
	  case 'S':
	    params.strengthRatio = atof(optarg);
	    break;
	  case 'D':
	    params.sqdistTh = square(atof(optarg));
	    break;
	  case 'W':
	    params.winSize = atoi(optarg);
	    break;
	  case 'A':
	    params.alpha = atof(optarg);
	    break;
	  case 'e':
	    params.edgeIntersection = true;
	    break;
	  case 'L':
	    params.lowTh = atof(optarg);
	    break;
	  case 'H':
	    params.highTh = atof(optarg);
	    break;
	  case 'h':
	    usage(argv[0]);
	    return 1;
	}

    try
    {
#ifdef WIN32
	if (_setmode(_fileno(stdin), _O_BINARY) == -1)
	    throw runtime_error("Cannot set stdin to binary mode!!"); 
	if (verify && _setmode(_fileno(stdout), _O_BINARY) == -1)
	    throw runtime_error("Cannot set stdout to binary mode!!"); 
#endif
	MarkerDetector	markerDetector(params);
	for (Image<u_char> image; image.restore(cin); )
	{
	    if (origin)
	    {
		Point2f	cross;
		markerDetector(image, cross, horizontal);

		if (verify)
		{
		    Image<RGB>	tmp = Y2RGB(image);
		    drawMarker(tmp, cross, RGB(0, 255, 255));
		    tmp.save(cout);
		}
		else
		    cout << cross;
	    }
	    else
	    {
		typedef MarkerDetector::CorresList	CorresList;
		
		CorresList	corresList;

		markerDetector(image, corresList, horizontal);

		for (CorresList::iterator iter  = corresList.begin();
		     iter != corresList.end(); ++iter)
		    iter->first *= pitch;

		if (verify)
		{
		    Image<RGB>	tmp = Y2RGB(image);
		    CorresList::const_iterator	iter = corresList.begin();
		    drawMarker(tmp, iter++->second, RGB(0, 255, 255));
		    drawMarker(tmp, iter++->second, RGB(255, 0, 255));
		    drawMarker(tmp, iter++->second, RGB(255, 0, 255));
		    drawMarker(tmp, iter++->second, RGB(255, 0, 255));
		    drawMarker(tmp, iter++->second, RGB(255, 0, 255));
		    while (iter != corresList.end())
			drawMarker(tmp, iter++->second, RGB(255, 255, 0));
		    tmp.save(cout);
		}
		else
		    cout << corresList;
	    }
	}
	if (!verify)
	    cout << endl;
    }
    catch (exception& err)
    {
	cerr << err.what() << endl;
	return 1;
    }
    
    return 0;
}
