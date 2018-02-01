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
#ifndef __TU_PLANECALIBGUI_H
#define __TU_PLANECALIBGUI_H

#include <sys/time.h>
#include "TU/v/TUv++.h"

/************************************************************************
*  global data and definitions						*
************************************************************************/
namespace TU
{
namespace v
{
enum
{
  // # of model planes.
    c_NPlanes,

  // Common camera centers or not.
    c_CommonCenters,
    
  // Actions for calibration.
    c_Extract,
    c_Calibrate,
    c_Check
};

/************************************************************************
*  global functions							*
************************************************************************/
template <class CAMERA> CmdDef*
createMenuCmds(CAMERA& camera)
{
    static MenuDef fileMenu[] =
    {
	{"Open images",		M_Open,		false, noSub},
	{"Save images",		M_Save,		false, noSub},
	{"Save camera conf.",	M_SaveAs,	false, noSub},
	{"-",			M_Line,		false, noSub},
	{"Quit",		M_Exit,		false, noSub},
	EndOfMenu
    };

    static CmdDef menuCmds[] =
    {
	{C_MenuButton,   M_File,	  0, "File",   fileMenu, CA_None,
	 0, 0, 1, 1, 0},
	{C_MenuButton,   M_Format,	  0, "Format", noProp,	 CA_None,
	 1, 0, 1, 1, 0},
	{C_ToggleButton, c_CommonCenters, 0, "Common camera centers",
	 noProp, CA_None, 2, 0, 1, 1, 0},
	EndOfCmds
    };

    menuCmds[1].prop = createFormatMenu(camera);

    return menuCmds;
}

inline CmdDef*
createCaptureCmds()
{
    static CmdDef CaptureCmds[] =
    {
	{C_Button,  c_Extract, 0, "Extract reference points",
	 noProp, CA_None, 0, 0, 1, 1, 0},
	{C_Label,  c_NPlanes, 0, "0 plane(s)",
	 noProp, CA_None, 1, 0, 1, 1, 0},
	{C_Button,  c_Calibrate, 0, "Do calibration",
	 noProp, CA_None, 2, 0, 1, 1, 0},
	EndOfCmds
    };
    
    return CaptureCmds;
}
 
inline void
countTime()
{
    static int		nframes = 0;
    static timeval	start;
    
    if (nframes == 10)
    {
	timeval	end;
	gettimeofday(&end, NULL);
	double	interval = (end.tv_sec  - start.tv_sec) +
	    (end.tv_usec - start.tv_usec) / 1.0e6;
	std::cerr << nframes / interval << " frames/sec" << std::endl;
	nframes = 0;
    }
    if (nframes++ == 0)
	gettimeofday(&start, NULL);
}

}
}

#endif	// !__TU_PLANECALIBGUI_H
