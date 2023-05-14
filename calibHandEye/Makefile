#
#  $Id: Makefile,v 1.18 2012-09-01 07:23:38 ueshiba Exp $
#
#################################
#  User customizable macros	#
#################################
DEST		= $(PREFIX)/bin
INCDIRS		= -I$(PREFIX)/include -I$(X11HOME)/include \
		  -I$(PREFIX)/include/rtm/idl \
		  -I$(HOME)/calib/planeCalib \
		  -I$(HOME)/calib/planeCalib/markerDetector \
		  -I$(HOME)/stereo/corrStereo

PROGRAM		= $(shell basename $(PWD))
LIBS		= -lTUIeee1394++ -lTUHRP2++ -lTUv++ -lTUTools++ \
		  -lXaw3d -lXt -lXext -lXmu -lX11 -lraw1394 \
		  -lHRP2Client -lHRP2Corba `rtm-config --libs` -ltbb
ifeq ($(OSTYPE), darwin)
#  LIBS	       +=
  LIBS	       += -framework IOKit -framework CoreFoundation -framework CoreServices
endif

CPPFLAGS	= -DHAVE_LIBTUTOOLS__ -DTRACK -DUSE_TBB #-DTUMinimizePP_DEBUG
CFLAGS		= -g
NVCCFLAGS	= -g
ifeq ($(CXX), icpc)
  CFLAGS	= -O3
  NVCCFLAGS	= -O		# -O2以上にするとコンパイルエラーになる．
  CPPFLAGS     += -DSSE3
endif
CCFLAGS		= $(CFLAGS)

LINKER		= $(CXX)

#########################
#  Macros set by mkmf	#
#########################
.SUFFIXES:	.cu
SUFFIX		= .cc:sC .cu:sC .cpp:sC
EXTHDRS		= /usr/local/include/TU/Array++.h \
		/usr/local/include/TU/Camera++.h \
		/usr/local/include/TU/Geometry++.h \
		/usr/local/include/TU/Image++.h \
		/usr/local/include/TU/List.h \
		/usr/local/include/TU/Manip.h \
		/usr/local/include/TU/Minimize.h \
		/usr/local/include/TU/Vector++.h \
		/usr/local/include/TU/algorithm.h \
		/usr/local/include/TU/iterator.h \
		/usr/local/include/TU/pair.h \
		/usr/local/include/TU/range.h \
		/usr/local/include/TU/tuple.h \
		/usr/local/include/TU/type_traits.h \
		/usr/local/include/TU/v/App.h \
		/usr/local/include/TU/v/CanvasPane.h \
		/usr/local/include/TU/v/CanvasPaneDC.h \
		/usr/local/include/TU/v/CmdPane.h \
		/usr/local/include/TU/v/CmdWindow.h \
		/usr/local/include/TU/v/Colormap.h \
		/usr/local/include/TU/v/Confirm.h \
		/usr/local/include/TU/v/DC.h \
		/usr/local/include/TU/v/Dialog.h \
		/usr/local/include/TU/v/FileSelection.h \
		/usr/local/include/TU/v/Menu.h \
		/usr/local/include/TU/v/ModalDialog.h \
		/usr/local/include/TU/v/Notify.h \
		/usr/local/include/TU/v/ShmDC.h \
		/usr/local/include/TU/v/TUv++.h \
		/usr/local/include/TU/v/Timer.h \
		/usr/local/include/TU/v/Widget-Xaw.h \
		/usr/local/include/TU/v/XDC.h
HDRS		= MyCanvasPane.h \
		MyCmdWindow.h \
		RigidTransformation.h \
		calibHandEye.h \
		calibHandEyeGUI.h
SRCS		= MyCanvasPane.cc \
		MyCmdWindow.cc \
		RigidTransformation.cc \
		createCaptureCmds.cc \
		createMenuCmds.cc \
		main.cc
OBJS		= MyCanvasPane.o \
		MyCmdWindow.o \
		RigidTransformation.o \
		createCaptureCmds.o \
		createMenuCmds.o \
		main.o

include $(PROJECT)/lib/p.mk

###
MyCanvasPane.o: MyCanvasPane.h /usr/local/include/TU/v/CanvasPane.h \
	/usr/local/include/TU/v/TUv++.h /usr/local/include/TU/List.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/Minimize.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/range.h /usr/local/include/TU/iterator.h \
	/usr/local/include/TU/tuple.h /usr/local/include/TU/type_traits.h \
	/usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/v/Widget-Xaw.h \
	/usr/local/include/TU/v/CanvasPaneDC.h /usr/local/include/TU/v/XDC.h \
	/usr/local/include/TU/v/DC.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/pair.h /usr/local/include/TU/Manip.h \
	/usr/local/include/TU/Camera++.h /usr/local/include/TU/v/Colormap.h \
	/usr/local/include/TU/v/Menu.h /usr/local/include/TU/v/ShmDC.h
MyCmdWindow.o: MyCmdWindow.h /usr/local/include/TU/v/App.h \
	/usr/local/include/TU/v/TUv++.h /usr/local/include/TU/List.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/Minimize.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/range.h /usr/local/include/TU/iterator.h \
	/usr/local/include/TU/tuple.h /usr/local/include/TU/type_traits.h \
	/usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/v/Widget-Xaw.h \
	/usr/local/include/TU/v/Colormap.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/pair.h /usr/local/include/TU/Manip.h \
	/usr/local/include/TU/Camera++.h /usr/local/include/TU/v/CmdWindow.h \
	/usr/local/include/TU/v/CmdPane.h /usr/local/include/TU/v/Timer.h \
	calibHandEye.h calibHandEyeGUI.h MyCanvasPane.h \
	/usr/local/include/TU/v/CanvasPane.h \
	/usr/local/include/TU/v/CanvasPaneDC.h /usr/local/include/TU/v/XDC.h \
	/usr/local/include/TU/v/DC.h /usr/local/include/TU/v/Menu.h \
	/usr/local/include/TU/v/ShmDC.h \
	/usr/local/include/TU/v/FileSelection.h \
	/usr/local/include/TU/v/ModalDialog.h \
	/usr/local/include/TU/v/Dialog.h /usr/local/include/TU/v/Confirm.h \
	/usr/local/include/TU/v/Notify.h
RigidTransformation.o: RigidTransformation.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/Minimize.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/range.h /usr/local/include/TU/iterator.h \
	/usr/local/include/TU/tuple.h /usr/local/include/TU/type_traits.h \
	/usr/local/include/TU/algorithm.h
createCaptureCmds.o: calibHandEyeGUI.h /usr/local/include/TU/v/TUv++.h \
	/usr/local/include/TU/List.h /usr/local/include/TU/Geometry++.h \
	/usr/local/include/TU/Minimize.h /usr/local/include/TU/Vector++.h \
	/usr/local/include/TU/Array++.h /usr/local/include/TU/range.h \
	/usr/local/include/TU/iterator.h /usr/local/include/TU/tuple.h \
	/usr/local/include/TU/type_traits.h /usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/v/Widget-Xaw.h
createMenuCmds.o: calibHandEyeGUI.h /usr/local/include/TU/v/TUv++.h \
	/usr/local/include/TU/List.h /usr/local/include/TU/Geometry++.h \
	/usr/local/include/TU/Minimize.h /usr/local/include/TU/Vector++.h \
	/usr/local/include/TU/Array++.h /usr/local/include/TU/range.h \
	/usr/local/include/TU/iterator.h /usr/local/include/TU/tuple.h \
	/usr/local/include/TU/type_traits.h /usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/v/Widget-Xaw.h
main.o: MyCmdWindow.h /usr/local/include/TU/v/App.h \
	/usr/local/include/TU/v/TUv++.h /usr/local/include/TU/List.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/Minimize.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/range.h /usr/local/include/TU/iterator.h \
	/usr/local/include/TU/tuple.h /usr/local/include/TU/type_traits.h \
	/usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/v/Widget-Xaw.h \
	/usr/local/include/TU/v/Colormap.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/pair.h /usr/local/include/TU/Manip.h \
	/usr/local/include/TU/Camera++.h /usr/local/include/TU/v/CmdWindow.h \
	/usr/local/include/TU/v/CmdPane.h /usr/local/include/TU/v/Timer.h \
	calibHandEye.h calibHandEyeGUI.h MyCanvasPane.h \
	/usr/local/include/TU/v/CanvasPane.h \
	/usr/local/include/TU/v/CanvasPaneDC.h /usr/local/include/TU/v/XDC.h \
	/usr/local/include/TU/v/DC.h /usr/local/include/TU/v/Menu.h \
	/usr/local/include/TU/v/ShmDC.h
