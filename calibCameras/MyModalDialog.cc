/*
 *  $Id: MyModalDialog.cc,v 1.1.1.1 2006-06-02 06:11:17 ueshiba Exp $
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
