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
		../calibCameras/calibCameras.h
HDRS		= MyCmdWindow.h
SRCS		= MyCmdWindow.cc \
		calibCameraWithPlanes.cc \
		main.cc
OBJS		= MyCmdWindow.o \
		calibCameraWithPlanes.o \
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
MyCmdWindow.o: MyCmdWindow.h ../calibCameras/MyCanvasPane.h \
	../calibCameras/MarkerDetector.h \
	../calibCameras/ProjectiveMappingWithDistortion.h \
	../calibCameras/calibCameras.h
calibCameraWithPlanes.o: ../calibCameras/MarkerDetector.h \
	../calibCameras/ProjectiveMappingWithDistortion.h
main.o: MyCmdWindow.h ../calibCameras/MyCanvasPane.h \
	../calibCameras/MarkerDetector.h \
	../calibCameras/ProjectiveMappingWithDistortion.h \
	../calibCameras/calibCameras.h
