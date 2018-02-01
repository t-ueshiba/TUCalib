/*
 *  $Id: createCaptureCmds.cc,v 1.1.1.1 2006-06-02 06:11:17 ueshiba Exp $
 */
#include "calibCameras.h"

namespace TU
{
using namespace v;
/************************************************************************
*  local data								*
************************************************************************/
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

/************************************************************************
*  global functions							*
************************************************************************/
CmdDef*
createCaptureCmds()
{
    return CaptureCmds;
}
 
}
