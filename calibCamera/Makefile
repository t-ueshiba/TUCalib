#
#  $Id: Makefile,v 1.1.1.1 2007-05-23 05:55:43 ueshiba Exp $
#
#################################
#  User customizable macros	#
#################################
DEST		= $(BINDIR)
INCDIRS		= -I../calibCameras -I$(HOME)/include

PROGRAM		= calibCamera
LIBS		= -lTUIeee1394++ -lTUv++ -lTUTools++ \
		  -lXaw3d -lXt -lXext -lXmu -lX11 -lraw1394 \
#		  -framework CoreFoundation -framework IOKit
CPPFLAGS	= -DUseXaw -DHAVE_TUToolsPP #-DDEBUG
CFLAGS		= -g
CCFLAGS		= -g -O
LDFLAGS		= -L$(LIBDIR) -L$(X11HOME)/lib

LINKER		= $(CCC)

#########################
#  Macros set by mkmf	#
#########################
SUFFIX		= .cc:sC
EXTHDRS		= ../calibCameras/MarkerDetector.h \
		../calibCameras/MyCanvasPane.h \
		../calibCameras/ProjectiveMappingWithDistortion.h \
		../calibCameras/calibCameras.h \
		../calibCameras/debug.h \
		/home/ueshiba/include/TU/Array++.h \
		/home/ueshiba/include/TU/Geometry++.h \
		/home/ueshiba/include/TU/Ieee1394++.h \
		/home/ueshiba/include/TU/Image++.cc \
		/home/ueshiba/include/TU/Image++.h \
		/home/ueshiba/include/TU/List++.h \
		/home/ueshiba/include/TU/Manip.h \
		/home/ueshiba/include/TU/Minimize++.h \
		/home/ueshiba/include/TU/Ransac++.h \
		/home/ueshiba/include/TU/Vector++.h \
		/home/ueshiba/include/TU/mmInstructions.h \
		/home/ueshiba/include/TU/types.h \
		/home/ueshiba/include/TU/utility.h \
		/home/ueshiba/include/TU/v/App.h \
		/home/ueshiba/include/TU/v/CanvasPane.h \
		/home/ueshiba/include/TU/v/CanvasPaneDC.h \
		/home/ueshiba/include/TU/v/CmdPane.h \
		/home/ueshiba/include/TU/v/CmdWindow.h \
		/home/ueshiba/include/TU/v/Colormap.h \
		/home/ueshiba/include/TU/v/Confirm.h \
		/home/ueshiba/include/TU/v/DC.h \
		/home/ueshiba/include/TU/v/Dialog.h \
		/home/ueshiba/include/TU/v/FileSelection.h \
		/home/ueshiba/include/TU/v/Menu.h \
		/home/ueshiba/include/TU/v/ModalDialog.h \
		/home/ueshiba/include/TU/v/Notify.h \
		/home/ueshiba/include/TU/v/ShmDC.h \
		/home/ueshiba/include/TU/v/TUv++.h \
		/home/ueshiba/include/TU/v/Timer.h \
		/home/ueshiba/include/TU/v/Widget-Xaw.h \
		/home/ueshiba/include/TU/v/XDC.h
HDRS		= MyCmdWindow.h
SRCS		= MarkerDetector.cc \
		MyCanvasPane.cc \
		MyCmdWindow.cc \
		calibCameraWithPlanes.cc \
		createCaptureCmds.cc \
		createFeatureCmds.cc \
		createMenuCmds.cc \
		debug.cc \
		main.cc
OBJS		= MarkerDetector.o \
		MyCanvasPane.o \
		MyCmdWindow.o \
		calibCameraWithPlanes.o \
		createCaptureCmds.o \
		createFeatureCmds.o \
		createMenuCmds.o \
		debug.o \
		main.o

#########################
#  Macros used by RCS	#
#########################
REV		= $(shell echo $Revision: 1.1.1.1 $	|		\
		  sed 's/evision://'		|		\
		  awk -F"."					\
		  '{						\
		      for (count = 1; count < NF; count++)	\
			  printf("%d.", $$count);		\
		      printf("%d", $$count + 1);		\
		  }')

include $(PROJECT)/lib/p.mk

###
MarkerDetector.o: /home/ueshiba/include/TU/Ransac++.h \
	/home/ueshiba/include/TU/types.h /home/ueshiba/include/TU/utility.h \
	../calibCameras/MarkerDetector.h /home/ueshiba/include/TU/Image++.h \
	/home/ueshiba/include/TU/Geometry++.h \
	/home/ueshiba/include/TU/Minimize++.h \
	/home/ueshiba/include/TU/Vector++.h \
	/home/ueshiba/include/TU/Array++.h \
	../calibCameras/ProjectiveMappingWithDistortion.h \
	../calibCameras/debug.h
MyCanvasPane.o: ../calibCameras/MyCanvasPane.h \
	/home/ueshiba/include/TU/v/CanvasPane.h \
	/home/ueshiba/include/TU/v/TUv++.h \
	/home/ueshiba/include/TU/v/Colormap.h \
	/home/ueshiba/include/TU/Image++.h \
	/home/ueshiba/include/TU/Geometry++.h \
	/home/ueshiba/include/TU/utility.h \
	/home/ueshiba/include/TU/Minimize++.h \
	/home/ueshiba/include/TU/Vector++.h \
	/home/ueshiba/include/TU/Array++.h /home/ueshiba/include/TU/types.h \
	/home/ueshiba/include/TU/List++.h \
	/home/ueshiba/include/TU/v/Widget-Xaw.h \
	/home/ueshiba/include/TU/v/CanvasPaneDC.h \
	/home/ueshiba/include/TU/v/XDC.h /home/ueshiba/include/TU/v/DC.h \
	/home/ueshiba/include/TU/Manip.h /home/ueshiba/include/TU/v/Menu.h \
	/home/ueshiba/include/TU/v/ShmDC.h ../calibCameras/MarkerDetector.h \
	../calibCameras/ProjectiveMappingWithDistortion.h
MyCmdWindow.o: MyCmdWindow.h /home/ueshiba/include/TU/v/App.h \
	/home/ueshiba/include/TU/v/TUv++.h \
	/home/ueshiba/include/TU/v/Colormap.h \
	/home/ueshiba/include/TU/Image++.h \
	/home/ueshiba/include/TU/Geometry++.h \
	/home/ueshiba/include/TU/utility.h \
	/home/ueshiba/include/TU/Minimize++.h \
	/home/ueshiba/include/TU/Vector++.h \
	/home/ueshiba/include/TU/Array++.h /home/ueshiba/include/TU/types.h \
	/home/ueshiba/include/TU/List++.h \
	/home/ueshiba/include/TU/v/Widget-Xaw.h \
	/home/ueshiba/include/TU/v/CmdWindow.h \
	/home/ueshiba/include/TU/v/CmdPane.h \
	/home/ueshiba/include/TU/v/FileSelection.h \
	/home/ueshiba/include/TU/v/ModalDialog.h \
	/home/ueshiba/include/TU/v/Dialog.h \
	/home/ueshiba/include/TU/v/Timer.h ../calibCameras/MyCanvasPane.h \
	/home/ueshiba/include/TU/v/CanvasPane.h \
	/home/ueshiba/include/TU/v/CanvasPaneDC.h \
	/home/ueshiba/include/TU/v/XDC.h /home/ueshiba/include/TU/v/DC.h \
	/home/ueshiba/include/TU/Manip.h /home/ueshiba/include/TU/v/Menu.h \
	/home/ueshiba/include/TU/v/ShmDC.h ../calibCameras/MarkerDetector.h \
	../calibCameras/ProjectiveMappingWithDistortion.h \
	../calibCameras/calibCameras.h /home/ueshiba/include/TU/Ieee1394++.h \
	/home/ueshiba/include/TU/v/Confirm.h \
	/home/ueshiba/include/TU/v/Notify.h
calibCameraWithPlanes.o: /home/ueshiba/include/TU/utility.h \
	/home/ueshiba/include/TU/Minimize++.h \
	/home/ueshiba/include/TU/Vector++.h \
	/home/ueshiba/include/TU/Array++.h /home/ueshiba/include/TU/types.h \
	../calibCameras/MarkerDetector.h /home/ueshiba/include/TU/Image++.h \
	/home/ueshiba/include/TU/Geometry++.h \
	../calibCameras/ProjectiveMappingWithDistortion.h
createCaptureCmds.o: ../calibCameras/calibCameras.h \
	/home/ueshiba/include/TU/v/TUv++.h \
	/home/ueshiba/include/TU/v/Colormap.h \
	/home/ueshiba/include/TU/Image++.h \
	/home/ueshiba/include/TU/Geometry++.h \
	/home/ueshiba/include/TU/utility.h \
	/home/ueshiba/include/TU/Minimize++.h \
	/home/ueshiba/include/TU/Vector++.h \
	/home/ueshiba/include/TU/Array++.h /home/ueshiba/include/TU/types.h \
	/home/ueshiba/include/TU/List++.h \
	/home/ueshiba/include/TU/v/Widget-Xaw.h \
	/home/ueshiba/include/TU/Ieee1394++.h
createFeatureCmds.o: ../calibCameras/calibCameras.h \
	/home/ueshiba/include/TU/v/TUv++.h \
	/home/ueshiba/include/TU/v/Colormap.h \
	/home/ueshiba/include/TU/Image++.h \
	/home/ueshiba/include/TU/Geometry++.h \
	/home/ueshiba/include/TU/utility.h \
	/home/ueshiba/include/TU/Minimize++.h \
	/home/ueshiba/include/TU/Vector++.h \
	/home/ueshiba/include/TU/Array++.h /home/ueshiba/include/TU/types.h \
	/home/ueshiba/include/TU/List++.h \
	/home/ueshiba/include/TU/v/Widget-Xaw.h \
	/home/ueshiba/include/TU/Ieee1394++.h
createMenuCmds.o: ../calibCameras/calibCameras.h \
	/home/ueshiba/include/TU/v/TUv++.h \
	/home/ueshiba/include/TU/v/Colormap.h \
	/home/ueshiba/include/TU/Image++.h \
	/home/ueshiba/include/TU/Geometry++.h \
	/home/ueshiba/include/TU/utility.h \
	/home/ueshiba/include/TU/Minimize++.h \
	/home/ueshiba/include/TU/Vector++.h \
	/home/ueshiba/include/TU/Array++.h /home/ueshiba/include/TU/types.h \
	/home/ueshiba/include/TU/List++.h \
	/home/ueshiba/include/TU/v/Widget-Xaw.h \
	/home/ueshiba/include/TU/Ieee1394++.h
debug.o: ../calibCameras/debug.h /home/ueshiba/include/TU/Image++.h \
	/home/ueshiba/include/TU/Geometry++.h \
	/home/ueshiba/include/TU/utility.h \
	/home/ueshiba/include/TU/Minimize++.h \
	/home/ueshiba/include/TU/Vector++.h \
	/home/ueshiba/include/TU/Array++.h /home/ueshiba/include/TU/types.h \
	/home/ueshiba/include/TU/Image++.cc \
	/home/ueshiba/include/TU/mmInstructions.h
main.o: MyCmdWindow.h /home/ueshiba/include/TU/v/App.h \
	/home/ueshiba/include/TU/v/TUv++.h \
	/home/ueshiba/include/TU/v/Colormap.h \
	/home/ueshiba/include/TU/Image++.h \
	/home/ueshiba/include/TU/Geometry++.h \
	/home/ueshiba/include/TU/utility.h \
	/home/ueshiba/include/TU/Minimize++.h \
	/home/ueshiba/include/TU/Vector++.h \
	/home/ueshiba/include/TU/Array++.h /home/ueshiba/include/TU/types.h \
	/home/ueshiba/include/TU/List++.h \
	/home/ueshiba/include/TU/v/Widget-Xaw.h \
	/home/ueshiba/include/TU/v/CmdWindow.h \
	/home/ueshiba/include/TU/v/CmdPane.h \
	/home/ueshiba/include/TU/v/FileSelection.h \
	/home/ueshiba/include/TU/v/ModalDialog.h \
	/home/ueshiba/include/TU/v/Dialog.h \
	/home/ueshiba/include/TU/v/Timer.h ../calibCameras/MyCanvasPane.h \
	/home/ueshiba/include/TU/v/CanvasPane.h \
	/home/ueshiba/include/TU/v/CanvasPaneDC.h \
	/home/ueshiba/include/TU/v/XDC.h /home/ueshiba/include/TU/v/DC.h \
	/home/ueshiba/include/TU/Manip.h /home/ueshiba/include/TU/v/Menu.h \
	/home/ueshiba/include/TU/v/ShmDC.h ../calibCameras/MarkerDetector.h \
	../calibCameras/ProjectiveMappingWithDistortion.h \
	../calibCameras/calibCameras.h /home/ueshiba/include/TU/Ieee1394++.h
