#
#  $Id: Makefile,v 1.5 2007-05-23 23:47:35 ueshiba Exp $
#
#################################
#  User customizable macros	#
#################################
DEST		= $(BINDIR)
INCDIRS		= -I$(HOME)/include -I/usr/local/include -I$(X11HOME)/include

PROGRAM		= calibCameras

LIBS		= -lTUIeee1394++ -lTUv++ -lTUTools++ \
		  -lXaw3d -lXt -lXext -lXmu -lX11 -lraw1394 \
		  -framework CoreFoundation -framework IOKit
CPPFLAGS	= -DUseXaw -DHAVE_TUToolsPP #-DUseOverlay
CFLAGS		= -O
CCFLAGS		= -O
LDFLAGS		= -L$(LIBDIR) -L$(X11HOME)/lib

LINKER		= $(CCC)

#########################
#  Macros set by mkmf	#
#########################
SUFFIX		= .cc:sC
EXTHDRS		= /Users/ueshiba/include/TU/Array++.h \
		/Users/ueshiba/include/TU/BlockMatrix++.h \
		/Users/ueshiba/include/TU/Geometry++.h \
		/Users/ueshiba/include/TU/Ieee1394++.h \
		/Users/ueshiba/include/TU/Image++.cc \
		/Users/ueshiba/include/TU/Image++.h \
		/Users/ueshiba/include/TU/List++.h \
		/Users/ueshiba/include/TU/Manip.h \
		/Users/ueshiba/include/TU/Minimize++.h \
		/Users/ueshiba/include/TU/Ransac++.h \
		/Users/ueshiba/include/TU/Vector++.h \
		/Users/ueshiba/include/TU/mmInstructions.h \
		/Users/ueshiba/include/TU/types.h \
		/Users/ueshiba/include/TU/utility.h \
		/Users/ueshiba/include/TU/v/App.h \
		/Users/ueshiba/include/TU/v/CanvasPane.h \
		/Users/ueshiba/include/TU/v/CanvasPaneDC.h \
		/Users/ueshiba/include/TU/v/CmdPane.h \
		/Users/ueshiba/include/TU/v/CmdWindow.h \
		/Users/ueshiba/include/TU/v/Colormap.h \
		/Users/ueshiba/include/TU/v/Confirm.h \
		/Users/ueshiba/include/TU/v/DC.h \
		/Users/ueshiba/include/TU/v/Dialog.h \
		/Users/ueshiba/include/TU/v/FileSelection.h \
		/Users/ueshiba/include/TU/v/Menu.h \
		/Users/ueshiba/include/TU/v/ModalDialog.h \
		/Users/ueshiba/include/TU/v/Notify.h \
		/Users/ueshiba/include/TU/v/ShmDC.h \
		/Users/ueshiba/include/TU/v/TUv++.h \
		/Users/ueshiba/include/TU/v/Timer.h \
		/Users/ueshiba/include/TU/v/Widget-Xaw.h \
		/Users/ueshiba/include/TU/v/XDC.h
HDRS		= MarkerDetector.h \
		MyCanvasPane.h \
		MyCanvasPaneTerse.h \
		MyCmdWindow.h \
		MyModalDialog.h \
		ProjectiveMappingWithDistortion.h \
		calibCameras.h \
		debug.h
SRCS		= MarkerDetector.cc \
		MyCanvasPane.cc \
		MyCanvasPaneTerse.cc \
		MyCmdWindow.cc \
		MyModalDialog.cc \
		calibCamerasWithPlanes.cc \
		createCaptureCmds.cc \
		createFeatureCmds.cc \
		createMenuCmds.cc \
		debug.cc \
		main.cc \
		refineCalibration.cc
OBJS		= MarkerDetector.o \
		MyCanvasPane.o \
		MyCanvasPaneTerse.o \
		MyCmdWindow.o \
		MyModalDialog.o \
		calibCamerasWithPlanes.o \
		createCaptureCmds.o \
		createFeatureCmds.o \
		createMenuCmds.o \
		debug.o \
		main.o \
		refineCalibration.o

#########################
#  Macros used by RCS	#
#########################
REV		= $(shell echo $Revision: 1.5 $	|		\
		  sed 's/evision://'		|		\
		  awk -F"."					\
		  '{						\
		      for (count = 1; count < NF; count++)	\
			  printf("%d.", $$count);		\
		      printf("%d", $$count + 1);		\
		  }')

include $(PROJECT)/lib/p.mk
###
MarkerDetector.o: /Users/ueshiba/include/TU/Ransac++.h \
	/Users/ueshiba/include/TU/types.h /Users/ueshiba/include/TU/utility.h \
	MarkerDetector.h /Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h ProjectiveMappingWithDistortion.h \
	debug.h
MyCanvasPane.o: MyCanvasPane.h /Users/ueshiba/include/TU/v/CanvasPane.h \
	/Users/ueshiba/include/TU/v/TUv++.h \
	/Users/ueshiba/include/TU/v/Colormap.h \
	/Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/List++.h \
	/Users/ueshiba/include/TU/v/Widget-Xaw.h \
	/Users/ueshiba/include/TU/v/CanvasPaneDC.h \
	/Users/ueshiba/include/TU/v/XDC.h /Users/ueshiba/include/TU/v/DC.h \
	/Users/ueshiba/include/TU/Manip.h /Users/ueshiba/include/TU/v/Menu.h \
	/Users/ueshiba/include/TU/v/ShmDC.h MarkerDetector.h \
	ProjectiveMappingWithDistortion.h
MyCanvasPaneTerse.o: MyCanvasPaneTerse.h MyCanvasPane.h \
	/Users/ueshiba/include/TU/v/CanvasPane.h \
	/Users/ueshiba/include/TU/v/TUv++.h \
	/Users/ueshiba/include/TU/v/Colormap.h \
	/Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/List++.h \
	/Users/ueshiba/include/TU/v/Widget-Xaw.h \
	/Users/ueshiba/include/TU/v/CanvasPaneDC.h \
	/Users/ueshiba/include/TU/v/XDC.h /Users/ueshiba/include/TU/v/DC.h \
	/Users/ueshiba/include/TU/Manip.h /Users/ueshiba/include/TU/v/Menu.h \
	/Users/ueshiba/include/TU/v/ShmDC.h MarkerDetector.h \
	ProjectiveMappingWithDistortion.h
MyCmdWindow.o: MyCmdWindow.h /Users/ueshiba/include/TU/v/App.h \
	/Users/ueshiba/include/TU/v/TUv++.h \
	/Users/ueshiba/include/TU/v/Colormap.h \
	/Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/List++.h \
	/Users/ueshiba/include/TU/v/Widget-Xaw.h \
	/Users/ueshiba/include/TU/v/CmdWindow.h \
	/Users/ueshiba/include/TU/v/CmdPane.h \
	/Users/ueshiba/include/TU/v/FileSelection.h \
	/Users/ueshiba/include/TU/v/ModalDialog.h \
	/Users/ueshiba/include/TU/v/Dialog.h \
	/Users/ueshiba/include/TU/v/Timer.h MyCanvasPaneTerse.h \
	MyCanvasPane.h /Users/ueshiba/include/TU/v/CanvasPane.h \
	/Users/ueshiba/include/TU/v/CanvasPaneDC.h \
	/Users/ueshiba/include/TU/v/XDC.h /Users/ueshiba/include/TU/v/DC.h \
	/Users/ueshiba/include/TU/Manip.h /Users/ueshiba/include/TU/v/Menu.h \
	/Users/ueshiba/include/TU/v/ShmDC.h MarkerDetector.h \
	ProjectiveMappingWithDistortion.h calibCameras.h \
	/Users/ueshiba/include/TU/Ieee1394++.h \
	/Users/ueshiba/include/TU/v/Confirm.h \
	/Users/ueshiba/include/TU/v/Notify.h
MyModalDialog.o: MyModalDialog.h /Users/ueshiba/include/TU/v/ModalDialog.h \
	/Users/ueshiba/include/TU/v/Dialog.h \
	/Users/ueshiba/include/TU/v/CmdPane.h \
	/Users/ueshiba/include/TU/v/CmdWindow.h \
	/Users/ueshiba/include/TU/v/TUv++.h \
	/Users/ueshiba/include/TU/v/Colormap.h \
	/Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/List++.h \
	/Users/ueshiba/include/TU/v/Widget-Xaw.h
calibCamerasWithPlanes.o: /Users/ueshiba/include/TU/utility.h \
	MarkerDetector.h /Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	ProjectiveMappingWithDistortion.h
createCaptureCmds.o: calibCameras.h /Users/ueshiba/include/TU/v/TUv++.h \
	/Users/ueshiba/include/TU/v/Colormap.h \
	/Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/List++.h \
	/Users/ueshiba/include/TU/v/Widget-Xaw.h \
	/Users/ueshiba/include/TU/Ieee1394++.h
createFeatureCmds.o: calibCameras.h /Users/ueshiba/include/TU/v/TUv++.h \
	/Users/ueshiba/include/TU/v/Colormap.h \
	/Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/List++.h \
	/Users/ueshiba/include/TU/v/Widget-Xaw.h \
	/Users/ueshiba/include/TU/Ieee1394++.h
createMenuCmds.o: calibCameras.h /Users/ueshiba/include/TU/v/TUv++.h \
	/Users/ueshiba/include/TU/v/Colormap.h \
	/Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/List++.h \
	/Users/ueshiba/include/TU/v/Widget-Xaw.h \
	/Users/ueshiba/include/TU/Ieee1394++.h
debug.o: debug.h /Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/Image++.cc \
	/Users/ueshiba/include/TU/mmInstructions.h
main.o: MyCmdWindow.h /Users/ueshiba/include/TU/v/App.h \
	/Users/ueshiba/include/TU/v/TUv++.h \
	/Users/ueshiba/include/TU/v/Colormap.h \
	/Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h \
	/Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/List++.h \
	/Users/ueshiba/include/TU/v/Widget-Xaw.h \
	/Users/ueshiba/include/TU/v/CmdWindow.h \
	/Users/ueshiba/include/TU/v/CmdPane.h \
	/Users/ueshiba/include/TU/v/FileSelection.h \
	/Users/ueshiba/include/TU/v/ModalDialog.h \
	/Users/ueshiba/include/TU/v/Dialog.h \
	/Users/ueshiba/include/TU/v/Timer.h MyCanvasPaneTerse.h \
	MyCanvasPane.h /Users/ueshiba/include/TU/v/CanvasPane.h \
	/Users/ueshiba/include/TU/v/CanvasPaneDC.h \
	/Users/ueshiba/include/TU/v/XDC.h /Users/ueshiba/include/TU/v/DC.h \
	/Users/ueshiba/include/TU/Manip.h /Users/ueshiba/include/TU/v/Menu.h \
	/Users/ueshiba/include/TU/v/ShmDC.h MarkerDetector.h \
	ProjectiveMappingWithDistortion.h calibCameras.h \
	/Users/ueshiba/include/TU/Ieee1394++.h
refineCalibration.o: /Users/ueshiba/include/TU/Minimize++.h \
	/Users/ueshiba/include/TU/Vector++.h \
	/Users/ueshiba/include/TU/Array++.h /Users/ueshiba/include/TU/types.h \
	/Users/ueshiba/include/TU/BlockMatrix++.h MarkerDetector.h \
	/Users/ueshiba/include/TU/Image++.h \
	/Users/ueshiba/include/TU/Geometry++.h \
	/Users/ueshiba/include/TU/utility.h ProjectiveMappingWithDistortion.h
