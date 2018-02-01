#########################################
#  Target and IDL sources		#
#########################################
LIB		= libomnihrp2.a

IDLSRCS		= HRPcontroller.idl \
		  MomCplugin.idl \
		  OpenHRPCommon.idl \
		  ViewSimulator.idl

#########################################
#  IDL compiler				#
#########################################
IDL		= omniidl
IDLFLAGS	= -bcxx -Wbh=.h -Wbs=.cpp -Wba -Wbd=DynSK.cpp -Wbtf

#########################################
#  C++ preprocessor and compiler	#
#########################################
INCDIRS		= -I.
CPPFLAGS	= -DOMNIORB4 -DTVMET_OPTIMIZE -DNDEBUG -DUSE_CLAPACK_INTERFACE
CCC		= g++
CCFLAGS		= -O3 -Wall -fno-schedule-insns -fno-schedule-insns2 -fno-strict-aliasing

#################################
#  Intermediate files		#
#################################
.SUFFIXES:	.cpp .idl

HDRS		= $(patsubst %.idl,%.h,$(IDLSRCS))
SRCS		= $(patsubst %.idl,%.cpp,$(IDLSRCS)) \
		  $(patsubst %.idl,%DynSK.cpp,$(IDLSRCS))
OBJS		= $(patsubst %.cpp,%.o,$(SRCS))

#################################
#  Making rules			#
#################################
all:		$(LIB)

clean:
		$(RM) $(LIB) $(OBJS) $(SRCS) $(HDRS)

$(LIB):		$(OBJS)
		$(AR) rv $@ $(OBJS)
		ranlib $@

$(OBJS):	$(SRCS) $(HDRS)

$(SRCS) $(HDRS):$(IDLSRCS)

.idl.cpp:
		$(IDL) $(IDLFLAGS) $(INCDIRS) $<

.cpp.o:
		$(CCC) $(CPPFLAGS) $(CCFLAGS) $(INCDIRS) -c $<

