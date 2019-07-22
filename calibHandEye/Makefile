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
EXTHDRS		= /home/ueshiba/calib/planeCalib/markerDetector/MarkerDetector.h \
		/home/ueshiba/calib/planeCalib/markerDetector/ProjectivityWithDistortion.h \
		/home/ueshiba/stereo/corrStereo/Rectify.h \
		/usr/local/include/TU/Array++.h \
		/usr/local/include/TU/Camera++.h \
		/usr/local/include/TU/Geometry++.h \
		/usr/local/include/TU/HRP2++.h \
		/usr/local/include/TU/Image++.h \
		/usr/local/include/TU/List.h \
		/usr/local/include/TU/Manip.h \
		/usr/local/include/TU/Minimize.h \
		/usr/local/include/TU/Vector++.h \
		/usr/local/include/TU/Warp.h \
		/usr/local/include/TU/algorithm.h \
		/usr/local/include/TU/cuda/iterator.h \
		/usr/local/include/TU/cuda/tuple.h \
		/usr/local/include/TU/functional.h \
		/usr/local/include/TU/iterator.h \
		/usr/local/include/TU/pair.h \
		/usr/local/include/TU/range.h \
		/usr/local/include/TU/simd/Array++.h \
		/usr/local/include/TU/simd/allocator.h \
		/usr/local/include/TU/simd/arithmetic.h \
		/usr/local/include/TU/simd/arm/arch.h \
		/usr/local/include/TU/simd/arm/arithmetic.h \
		/usr/local/include/TU/simd/arm/bit_shift.h \
		/usr/local/include/TU/simd/arm/cast.h \
		/usr/local/include/TU/simd/arm/compare.h \
		/usr/local/include/TU/simd/arm/cvt.h \
		/usr/local/include/TU/simd/arm/dup.h \
		/usr/local/include/TU/simd/arm/insert_extract.h \
		/usr/local/include/TU/simd/arm/load_store.h \
		/usr/local/include/TU/simd/arm/logical.h \
		/usr/local/include/TU/simd/arm/lookup.h \
		/usr/local/include/TU/simd/arm/select.h \
		/usr/local/include/TU/simd/arm/shift.h \
		/usr/local/include/TU/simd/arm/type_traits.h \
		/usr/local/include/TU/simd/arm/vec.h \
		/usr/local/include/TU/simd/arm/zero.h \
		/usr/local/include/TU/simd/bit_shift.h \
		/usr/local/include/TU/simd/cast.h \
		/usr/local/include/TU/simd/compare.h \
		/usr/local/include/TU/simd/config.h \
		/usr/local/include/TU/simd/cvt.h \
		/usr/local/include/TU/simd/cvtdown_iterator.h \
		/usr/local/include/TU/simd/cvtup_iterator.h \
		/usr/local/include/TU/simd/dup.h \
		/usr/local/include/TU/simd/insert_extract.h \
		/usr/local/include/TU/simd/load_store.h \
		/usr/local/include/TU/simd/load_store_iterator.h \
		/usr/local/include/TU/simd/logical.h \
		/usr/local/include/TU/simd/lookup.h \
		/usr/local/include/TU/simd/misc.h \
		/usr/local/include/TU/simd/select.h \
		/usr/local/include/TU/simd/shift.h \
		/usr/local/include/TU/simd/shift_iterator.h \
		/usr/local/include/TU/simd/simd.h \
		/usr/local/include/TU/simd/transform.h \
		/usr/local/include/TU/simd/type_traits.h \
		/usr/local/include/TU/simd/vec.h \
		/usr/local/include/TU/simd/x86/arch.h \
		/usr/local/include/TU/simd/x86/arithmetic.h \
		/usr/local/include/TU/simd/x86/bit_shift.h \
		/usr/local/include/TU/simd/x86/cast.h \
		/usr/local/include/TU/simd/x86/compare.h \
		/usr/local/include/TU/simd/x86/cvt.h \
		/usr/local/include/TU/simd/x86/dup.h \
		/usr/local/include/TU/simd/x86/insert_extract.h \
		/usr/local/include/TU/simd/x86/load_store.h \
		/usr/local/include/TU/simd/x86/logical.h \
		/usr/local/include/TU/simd/x86/logical_base.h \
		/usr/local/include/TU/simd/x86/lookup.h \
		/usr/local/include/TU/simd/x86/select.h \
		/usr/local/include/TU/simd/x86/shift.h \
		/usr/local/include/TU/simd/x86/shuffle.h \
		/usr/local/include/TU/simd/x86/svml.h \
		/usr/local/include/TU/simd/x86/type_traits.h \
		/usr/local/include/TU/simd/x86/unpack.h \
		/usr/local/include/TU/simd/x86/vec.h \
		/usr/local/include/TU/simd/x86/zero.h \
		/usr/local/include/TU/simd/zero.h \
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
	/usr/local/include/TU/algorithm.h /usr/local/include/TU/cuda/tuple.h \
	/usr/local/include/TU/cuda/iterator.h \
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
	/usr/local/include/TU/algorithm.h /usr/local/include/TU/cuda/tuple.h \
	/usr/local/include/TU/cuda/iterator.h \
	/usr/local/include/TU/v/Widget-Xaw.h \
	/usr/local/include/TU/v/Colormap.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/pair.h /usr/local/include/TU/Manip.h \
	/usr/local/include/TU/Camera++.h /usr/local/include/TU/v/CmdWindow.h \
	/usr/local/include/TU/v/CmdPane.h /usr/local/include/TU/v/Timer.h \
	/usr/local/include/TU/HRP2++.h calibHandEye.h calibHandEyeGUI.h \
	/home/ueshiba/stereo/corrStereo/Rectify.h \
	/usr/local/include/TU/Warp.h /usr/local/include/TU/simd/Array++.h \
	/usr/local/include/TU/simd/simd.h /usr/local/include/TU/simd/config.h \
	/usr/local/include/TU/simd/vec.h \
	/usr/local/include/TU/simd/type_traits.h \
	/usr/local/include/TU/simd/x86/type_traits.h \
	/usr/local/include/TU/simd/arm/type_traits.h \
	/usr/local/include/TU/simd/x86/vec.h \
	/usr/local/include/TU/simd/x86/arch.h \
	/usr/local/include/TU/simd/arm/vec.h \
	/usr/local/include/TU/simd/arm/arch.h \
	/usr/local/include/TU/simd/allocator.h \
	/usr/local/include/TU/simd/load_store.h \
	/usr/local/include/TU/simd/x86/load_store.h \
	/usr/local/include/TU/simd/arm/load_store.h \
	/usr/local/include/TU/simd/zero.h \
	/usr/local/include/TU/simd/x86/zero.h \
	/usr/local/include/TU/simd/arm/zero.h \
	/usr/local/include/TU/simd/cast.h \
	/usr/local/include/TU/simd/x86/cast.h \
	/usr/local/include/TU/simd/arm/cast.h \
	/usr/local/include/TU/simd/insert_extract.h \
	/usr/local/include/TU/simd/x86/insert_extract.h \
	/usr/local/include/TU/simd/arm/insert_extract.h \
	/usr/local/include/TU/simd/shift.h \
	/usr/local/include/TU/simd/x86/shift.h \
	/usr/local/include/TU/simd/arm/shift.h \
	/usr/local/include/TU/simd/bit_shift.h \
	/usr/local/include/TU/simd/x86/bit_shift.h \
	/usr/local/include/TU/simd/arm/bit_shift.h \
	/usr/local/include/TU/simd/dup.h /usr/local/include/TU/simd/cvt.h \
	/usr/local/include/TU/simd/x86/cvt.h \
	/usr/local/include/TU/simd/x86/unpack.h \
	/usr/local/include/TU/simd/arm/cvt.h \
	/usr/local/include/TU/simd/logical.h \
	/usr/local/include/TU/simd/x86/logical.h \
	/usr/local/include/TU/simd/x86/logical_base.h \
	/usr/local/include/TU/simd/arm/logical.h \
	/usr/local/include/TU/simd/x86/dup.h \
	/usr/local/include/TU/simd/arm/dup.h \
	/usr/local/include/TU/simd/compare.h \
	/usr/local/include/TU/simd/x86/compare.h \
	/usr/local/include/TU/simd/arm/compare.h \
	/usr/local/include/TU/simd/select.h \
	/usr/local/include/TU/simd/x86/select.h \
	/usr/local/include/TU/simd/arm/select.h \
	/usr/local/include/TU/simd/arithmetic.h \
	/usr/local/include/TU/simd/x86/arithmetic.h \
	/usr/local/include/TU/simd/arm/arithmetic.h \
	/usr/local/include/TU/simd/misc.h \
	/usr/local/include/TU/simd/x86/shuffle.h \
	/usr/local/include/TU/simd/x86/svml.h \
	/usr/local/include/TU/simd/transform.h \
	/usr/local/include/TU/functional.h \
	/usr/local/include/TU/simd/lookup.h \
	/usr/local/include/TU/simd/x86/lookup.h \
	/usr/local/include/TU/simd/arm/lookup.h \
	/usr/local/include/TU/simd/load_store_iterator.h \
	/usr/local/include/TU/simd/cvtdown_iterator.h \
	/usr/local/include/TU/simd/cvtup_iterator.h \
	/usr/local/include/TU/simd/shift_iterator.h \
	/home/ueshiba/calib/planeCalib/markerDetector/MarkerDetector.h \
	/home/ueshiba/calib/planeCalib/markerDetector/ProjectivityWithDistortion.h \
	MyCanvasPane.h /usr/local/include/TU/v/CanvasPane.h \
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
	/usr/local/include/TU/algorithm.h /usr/local/include/TU/cuda/tuple.h \
	/usr/local/include/TU/cuda/iterator.h
createCaptureCmds.o: calibHandEyeGUI.h /usr/local/include/TU/v/TUv++.h \
	/usr/local/include/TU/List.h /usr/local/include/TU/Geometry++.h \
	/usr/local/include/TU/Minimize.h /usr/local/include/TU/Vector++.h \
	/usr/local/include/TU/Array++.h /usr/local/include/TU/range.h \
	/usr/local/include/TU/iterator.h /usr/local/include/TU/tuple.h \
	/usr/local/include/TU/type_traits.h /usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/cuda/tuple.h \
	/usr/local/include/TU/cuda/iterator.h \
	/usr/local/include/TU/v/Widget-Xaw.h
createMenuCmds.o: calibHandEyeGUI.h /usr/local/include/TU/v/TUv++.h \
	/usr/local/include/TU/List.h /usr/local/include/TU/Geometry++.h \
	/usr/local/include/TU/Minimize.h /usr/local/include/TU/Vector++.h \
	/usr/local/include/TU/Array++.h /usr/local/include/TU/range.h \
	/usr/local/include/TU/iterator.h /usr/local/include/TU/tuple.h \
	/usr/local/include/TU/type_traits.h /usr/local/include/TU/algorithm.h \
	/usr/local/include/TU/cuda/tuple.h \
	/usr/local/include/TU/cuda/iterator.h \
	/usr/local/include/TU/v/Widget-Xaw.h
main.o: MyCmdWindow.h /usr/local/include/TU/v/App.h \
	/usr/local/include/TU/v/TUv++.h /usr/local/include/TU/List.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/Minimize.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/range.h /usr/local/include/TU/iterator.h \
	/usr/local/include/TU/tuple.h /usr/local/include/TU/type_traits.h \
	/usr/local/include/TU/algorithm.h /usr/local/include/TU/cuda/tuple.h \
	/usr/local/include/TU/cuda/iterator.h \
	/usr/local/include/TU/v/Widget-Xaw.h \
	/usr/local/include/TU/v/Colormap.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/pair.h /usr/local/include/TU/Manip.h \
	/usr/local/include/TU/Camera++.h /usr/local/include/TU/v/CmdWindow.h \
	/usr/local/include/TU/v/CmdPane.h /usr/local/include/TU/v/Timer.h \
	/usr/local/include/TU/HRP2++.h calibHandEye.h calibHandEyeGUI.h \
	/home/ueshiba/stereo/corrStereo/Rectify.h \
	/usr/local/include/TU/Warp.h /usr/local/include/TU/simd/Array++.h \
	/usr/local/include/TU/simd/simd.h /usr/local/include/TU/simd/config.h \
	/usr/local/include/TU/simd/vec.h \
	/usr/local/include/TU/simd/type_traits.h \
	/usr/local/include/TU/simd/x86/type_traits.h \
	/usr/local/include/TU/simd/arm/type_traits.h \
	/usr/local/include/TU/simd/x86/vec.h \
	/usr/local/include/TU/simd/x86/arch.h \
	/usr/local/include/TU/simd/arm/vec.h \
	/usr/local/include/TU/simd/arm/arch.h \
	/usr/local/include/TU/simd/allocator.h \
	/usr/local/include/TU/simd/load_store.h \
	/usr/local/include/TU/simd/x86/load_store.h \
	/usr/local/include/TU/simd/arm/load_store.h \
	/usr/local/include/TU/simd/zero.h \
	/usr/local/include/TU/simd/x86/zero.h \
	/usr/local/include/TU/simd/arm/zero.h \
	/usr/local/include/TU/simd/cast.h \
	/usr/local/include/TU/simd/x86/cast.h \
	/usr/local/include/TU/simd/arm/cast.h \
	/usr/local/include/TU/simd/insert_extract.h \
	/usr/local/include/TU/simd/x86/insert_extract.h \
	/usr/local/include/TU/simd/arm/insert_extract.h \
	/usr/local/include/TU/simd/shift.h \
	/usr/local/include/TU/simd/x86/shift.h \
	/usr/local/include/TU/simd/arm/shift.h \
	/usr/local/include/TU/simd/bit_shift.h \
	/usr/local/include/TU/simd/x86/bit_shift.h \
	/usr/local/include/TU/simd/arm/bit_shift.h \
	/usr/local/include/TU/simd/dup.h /usr/local/include/TU/simd/cvt.h \
	/usr/local/include/TU/simd/x86/cvt.h \
	/usr/local/include/TU/simd/x86/unpack.h \
	/usr/local/include/TU/simd/arm/cvt.h \
	/usr/local/include/TU/simd/logical.h \
	/usr/local/include/TU/simd/x86/logical.h \
	/usr/local/include/TU/simd/x86/logical_base.h \
	/usr/local/include/TU/simd/arm/logical.h \
	/usr/local/include/TU/simd/x86/dup.h \
	/usr/local/include/TU/simd/arm/dup.h \
	/usr/local/include/TU/simd/compare.h \
	/usr/local/include/TU/simd/x86/compare.h \
	/usr/local/include/TU/simd/arm/compare.h \
	/usr/local/include/TU/simd/select.h \
	/usr/local/include/TU/simd/x86/select.h \
	/usr/local/include/TU/simd/arm/select.h \
	/usr/local/include/TU/simd/arithmetic.h \
	/usr/local/include/TU/simd/x86/arithmetic.h \
	/usr/local/include/TU/simd/arm/arithmetic.h \
	/usr/local/include/TU/simd/misc.h \
	/usr/local/include/TU/simd/x86/shuffle.h \
	/usr/local/include/TU/simd/x86/svml.h \
	/usr/local/include/TU/simd/transform.h \
	/usr/local/include/TU/functional.h \
	/usr/local/include/TU/simd/lookup.h \
	/usr/local/include/TU/simd/x86/lookup.h \
	/usr/local/include/TU/simd/arm/lookup.h \
	/usr/local/include/TU/simd/load_store_iterator.h \
	/usr/local/include/TU/simd/cvtdown_iterator.h \
	/usr/local/include/TU/simd/cvtup_iterator.h \
	/usr/local/include/TU/simd/shift_iterator.h \
	/home/ueshiba/calib/planeCalib/markerDetector/MarkerDetector.h \
	/home/ueshiba/calib/planeCalib/markerDetector/ProjectivityWithDistortion.h \
	MyCanvasPane.h /usr/local/include/TU/v/CanvasPane.h \
	/usr/local/include/TU/v/CanvasPaneDC.h /usr/local/include/TU/v/XDC.h \
	/usr/local/include/TU/v/DC.h /usr/local/include/TU/v/Menu.h \
	/usr/local/include/TU/v/ShmDC.h
