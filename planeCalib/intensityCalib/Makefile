#
#  $Id: Makefile,v 1.3 2011-01-14 02:32:10 ueshiba Exp $
#
#################################
#  User customizable macros	#
#################################
DEST		= $(PREFIX)/bin
INCDIRS		= -I$(PREFIX)/include

PROGRAM		= intensityCalib
LIBS		= -lTUThread++ -lTUTools++ -lpthread

CPPFLAGS	=
CFLAGS		= -g
CCFLAGS		= -g
ifeq ($(CCC), icpc)
  CCFLAGS	= -O
endif

LINKER		= $(CCC)

#########################
#  Macros set by mkmf	#
#########################
SUFFIX		= .cc:sC
EXTHDRS		= /usr/local/include/TU/Array++.h \
		/usr/local/include/TU/Camera.h \
		/usr/local/include/TU/CorrectIntensity.h \
		/usr/local/include/TU/Geometry++.h \
		/usr/local/include/TU/Image++.h \
		/usr/local/include/TU/Minimize.h \
		/usr/local/include/TU/Normalize.h \
		/usr/local/include/TU/Ransac.h \
		/usr/local/include/TU/Thread++.h \
		/usr/local/include/TU/Vector++.h \
		/usr/local/include/TU/Warp.h \
		/usr/local/include/TU/mmInstructions.h \
		/usr/local/include/TU/types.h \
		/usr/local/include/TU/utility.h
HDRS		= IntensityCalibrator.h \
		binarize.h \
		rectifyImages.h
SRCS		= main.cc
OBJS		= main.o

#########################
#  Macros used by RCS	#
#########################
REV		= $(shell echo $Revision: 1.3 $	|		\
		  sed 's/evision://'		|		\
		  awk -F"."					\
		  '{						\
		      for (count = 1; count < NF; count++)	\
			  printf("%d.", $$count);		\
		      printf("%d", $$count + 1);		\
		  }')

include $(PROJECT)/lib/p.mk

###
main.o: rectifyImages.h /usr/local/include/TU/Image++.h \
	/usr/local/include/TU/Geometry++.h /usr/local/include/TU/utility.h \
	/usr/local/include/TU/Vector++.h /usr/local/include/TU/Array++.h \
	/usr/local/include/TU/types.h /usr/local/include/TU/Normalize.h \
	/usr/local/include/TU/Minimize.h /usr/local/include/TU/Warp.h \
	/usr/local/include/TU/Camera.h /usr/local/include/TU/mmInstructions.h \
	IntensityCalibrator.h /usr/local/include/TU/Ransac.h binarize.h \
	/usr/local/include/TU/CorrectIntensity.h \
	/usr/local/include/TU/Thread++.h
