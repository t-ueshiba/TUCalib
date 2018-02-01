/*
 *  $Id: MyModalDialog.h,v 1.1.1.1 2006-06-02 06:11:17 ueshiba Exp $
 */
#include <sstream>
#include "TU/v/ModalDialog.h"

namespace TU
{
namespace v
{
/************************************************************************
*  class ModalDialog							*
************************************************************************/
class MyModalDialog : public ModalDialog, public std::ostringstream
{
  public:
    enum Choice	{OK_More, OK_Done, Discard};
    
    MyModalDialog(Window& parentWindow)				;
    virtual		~MyModalDialog()			;

    Choice		choice()				;
    
    virtual void	callback(CmdId id, CmdVal val)		;
    
  private:
    Choice		_choice;
};
    
}
}
