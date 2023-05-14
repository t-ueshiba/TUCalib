#
#  $Id$
#
#################################
#  User customizable macros	#
#################################
PROGRAM		= planeCalibIIDC
#LIBRARY		= lib$(shell basename $(PWD))

VPATH		=

IDLS		=
MOCHDRS		=

INCDIRS		= -ImarkerDetector -IplaneCalib -I$(PREFIX)/include
CPPFLAGS	= -DNDEBUG
CFLAGS		= -O3
NVCCFLAGS	= -O
ifeq ($(shell arch), armv7l)
  CPPFLAGS     += -DNEON
else ifeq ($(shell arch), aarch64)
  CPPFLAGS     += -DNEON
else
  CPPFLAGS     += -DSSE3
endif
CCFLAGS		= $(CFLAGS)

ifneq ($(findstring darwin,$(OSTYPE)),)
  LIBS		= -lusb-1.0 \
		  -lTUvIIDC -lTUIIDC -lTUv -lTUTools \
		  -lXaw3d -lXt -lXext -lXmu -lX11 -lraw1394
else
  LIBS		= -lTUvIIDC -lTUIIDC -lTUv -lTUTools \
		  -lXaw3d -lXt -lXext -lXmu -lX11 -lraw1394 -lusb-1.0
endif
ifneq ($(findstring darwin,$(OSTYPE)),)
  LIBS	       += -framework IOKit -framework CoreFoundation -framework CoreServices
endif

LINKER		= $(CXX)

BINDIR		= $(PREFIX)/bin
LIBDIR		= $(PREFIX)/lib
INCDIR		= $(PREFIX)/include

#########################
#  Macros set by mkmf	#
#########################
SUFFIX		= .cc:sC .cpp:sC .cu:sC
EXTHDRS		= /usr/local/include/TU/Array++.h \
		/usr/local/include/TU/BlockDiagonalMatrix++.h \
		/usr/local/include/TU/Camera++.h \
		/usr/local/include/TU/DericheConvolver.h \
		/usr/local/include/TU/EdgeDetector.h \
		/usr/local/include/TU/Geometry++.h \
		/usr/local/include/TU/IIDC++.h \
		/usr/local/include/TU/IIDCCameraArray.h \
		/usr/local/include/TU/IIRFilter.h \
		/usr/local/include/TU/Image++.h \
		/usr/local/include/TU/IntegralImage.h \
		/usr/local/include/TU/List.h \
		/usr/local/include/TU/Manip.h \
		/usr/local/include/TU/Minimize.h \
		/usr/local/include/TU/Ransac.h \
		/usr/local/include/TU/SeparableFilter2.h \
		/usr/local/include/TU/Vector++.h \
		/usr/local/include/TU/algorithm.h \
		/usr/local/include/TU/iterator.h \
		/usr/local/include/TU/pair.h \
		/usr/local/include/TU/range.h \
		/usr/local/include/TU/tuple.h \
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
		/usr/local/include/TU/v/TUv++.h \
		/usr/local/include/TU/v/Timer.h \
		/usr/local/include/TU/v/Widget-Xaw.h \
		/usr/local/include/TU/v/XDC.h \
		/usr/local/include/TU/v/vIIDC++.h \
		markerDetector/MarkerDetector.h \
		markerDetector/ProjectivityWithDistortion.h \
		markerDetector/debug.h \
		planeCalib/CameraCalibrator.h
HDRS		= MyCanvasPane.h \
		MyCanvasPaneTerse.h \
		MyCmdWindow.h \
		MyModalDialog.h \
		planeCalibGUI.h
SRCS		= CameraCalibrator.cc \
		MarkerDetector.cc \
		MyCanvasPane.cc \
		MyCanvasPaneTerse.cc \
		MyModalDialog.cc \
		main.cc
OBJS		= CameraCalibrator.o \
		MarkerDetector.o \
		MyCanvasPane.o \
		MyCanvasPaneTerse.o \
		MyModalDialog.o \
		main.o

#include $(PROJECT)/lib/rtc.mk		# IDLHDRS, IDLSRCS, CPPFLAGS, OBJS, LIBS
#include $(PROJECT)/lib/qt.mk		# MOCSRCS, OBJS
#include $(PROJECT)/lib/cnoid.mk	# CPPFLAGS, LIBS, LIBDIR
#include $(PROJECT)/lib/lib.mk		# PUBHDRS TARGHDRS
include $(PROJECT)/lib/common.mk
###
CameraCalibrator.o: planeCalib/CameraCalibrator.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/Minimize.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/range.h /usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/iterator.h /usr/local/include/TU/tuple.h \
	/usr/local/include/TU/BlockDiagonalMatrix++.h
MarkerDetector.o: markerDetector/MarkerDetector.h \
	/usr/local/include/TU/Image++.h /usr/local/include/TU/pair.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/range.h /usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/iterator.h /usr/local/include/TU/tuple.h \
	markerDetector/ProjectivityWithDistortion.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/Minimize.h \
	/usr/local/include/TU/Ransac.h /usr/local/include/TU/IntegralImage.h \
	/usr/local/include/TU/DericheConvolver.h \
	/usr/local/include/TU/IIRFilter.h \
	/usr/local/include/TU/SeparableFilter2.h \
	/usr/local/include/TU/EdgeDetector.h markerDetector/debug.h
MyCanvasPane.o: MyCanvasPane.h /usr/local/include/TU/v/CanvasPane.h \
	/usr/local/include/TU/v/TUv++.h /usr/local/include/TU/List.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/Minimize.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/range.h /usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/iterator.h /usr/local/include/TU/tuple.h \
	/usr/local/include/TU/v/Widget-Xaw.h \
	/usr/local/include/TU/v/CanvasPaneDC.h /usr/local/include/TU/v/XDC.h \
	/usr/local/include/TU/v/DC.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/pair.h /usr/local/include/TU/Manip.h \
	/usr/local/include/TU/v/Colormap.h /usr/local/include/TU/v/Menu.h \
	markerDetector/MarkerDetector.h \
	markerDetector/ProjectivityWithDistortion.h
MyCanvasPaneTerse.o: MyCanvasPaneTerse.h MyCanvasPane.h \
	/usr/local/include/TU/v/CanvasPane.h /usr/local/include/TU/v/TUv++.h \
	/usr/local/include/TU/List.h /usr/local/include/TU/Geometry++.h \
	/usr/local/include/TU/Minimize.h /usr/local/include/TU/Vector++.h \
	/usr/local/include/TU/Array++.h /usr/local/include/TU/range.h \
	/usr/local/include/TU/algorithm.h /usr/local/include/TU/iterator.h \
	/usr/local/include/TU/tuple.h /usr/local/include/TU/v/Widget-Xaw.h \
	/usr/local/include/TU/v/CanvasPaneDC.h /usr/local/include/TU/v/XDC.h \
	/usr/local/include/TU/v/DC.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/pair.h /usr/local/include/TU/Manip.h \
	/usr/local/include/TU/v/Colormap.h /usr/local/include/TU/v/Menu.h \
	markerDetector/MarkerDetector.h \
	markerDetector/ProjectivityWithDistortion.h
MyModalDialog.o: MyModalDialog.h /usr/local/include/TU/v/ModalDialog.h \
	/usr/local/include/TU/v/Dialog.h /usr/local/include/TU/v/CmdPane.h \
	/usr/local/include/TU/v/CmdWindow.h /usr/local/include/TU/v/TUv++.h \
	/usr/local/include/TU/List.h /usr/local/include/TU/Geometry++.h \
	/usr/local/include/TU/Minimize.h /usr/local/include/TU/Vector++.h \
	/usr/local/include/TU/Array++.h /usr/local/include/TU/range.h \
	/usr/local/include/TU/algorithm.h /usr/local/include/TU/iterator.h \
	/usr/local/include/TU/tuple.h /usr/local/include/TU/v/Widget-Xaw.h \
	/usr/local/include/TU/v/Colormap.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/pair.h
main.o: /usr/local/include/TU/v/vIIDC++.h \
	/usr/local/include/TU/v/CmdPane.h /usr/local/include/TU/v/CmdWindow.h \
	/usr/local/include/TU/v/TUv++.h /usr/local/include/TU/List.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/Minimize.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/range.h /usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/iterator.h /usr/local/include/TU/tuple.h \
	/usr/local/include/TU/v/Widget-Xaw.h \
	/usr/local/include/TU/v/Colormap.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/pair.h /usr/local/include/TU/IIDCCameraArray.h \
	/usr/local/include/TU/IIDC++.h MyCmdWindow.h \
	/usr/local/include/TU/v/App.h /usr/local/include/TU/v/Timer.h \
	/usr/local/include/TU/v/FileSelection.h \
	/usr/local/include/TU/v/ModalDialog.h \
	/usr/local/include/TU/v/Dialog.h /usr/local/include/TU/v/Confirm.h \
	/usr/local/include/TU/v/Notify.h /usr/local/include/TU/Camera++.h \
	planeCalib/CameraCalibrator.h \
	/usr/local/include/TU/BlockDiagonalMatrix++.h MyCanvasPaneTerse.h \
	MyCanvasPane.h /usr/local/include/TU/v/CanvasPane.h \
	/usr/local/include/TU/v/CanvasPaneDC.h /usr/local/include/TU/v/XDC.h \
	/usr/local/include/TU/v/DC.h /usr/local/include/TU/Manip.h \
	/usr/local/include/TU/v/Menu.h markerDetector/MarkerDetector.h \
	markerDetector/ProjectivityWithDistortion.h planeCalibGUI.h
