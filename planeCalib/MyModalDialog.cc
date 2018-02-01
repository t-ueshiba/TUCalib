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
#include "MyModalDialog.h"

namespace TU
{
namespace v
{
/************************************************************************
*  static data								*
************************************************************************/
enum {c_Message, c_OK_More, c_OK_Done, c_Discard};

static CmdDef	Cmds[] =
{
    {C_Label,  c_Message, 0, "", noProp, CA_NoBorder, 0, 0, 3, 1, 20},
    {C_Button, c_OK_More, 0, "OK, capture more planes.", noProp,
     CA_DefaultButton, 0, 1, 1, 1, 0},
    {C_Button, c_OK_Done, 0, "OK, capture no more planes.", noProp, CA_None,
     1, 1, 1, 1, 0},
    {C_Button, c_Discard, 0, "Discard", noProp, CA_None, 2, 1, 1, 1, 0},
    EndOfCmds
};
        
/************************************************************************
*  class ModalDialog							*
************************************************************************/
MyModalDialog::MyModalDialog(Window& parentWindow)
    :ModalDialog(parentWindow, "Confirm", Cmds), _choice(OK_More)
{
}

MyModalDialog::~MyModalDialog()
{
}

MyModalDialog::Choice
MyModalDialog::choice()
{
    pane().setString(c_Message, str().c_str());
    show();
    return _choice;
}

void
MyModalDialog::callback(CmdId id, CmdVal val)
{
    switch (id)
    {
      case c_OK_More:
	_choice = OK_More;
	hide();
	break;
      case c_OK_Done:
	_choice = OK_Done;
	hide();
	break;
      case c_Discard:
	_choice = Discard;
	hide();
	break;
    }
}

}
}
