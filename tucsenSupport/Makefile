TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Note, the files in this directory were manually copied from the Andor SDK3 directories 
# for Linux and Windows.  The soft links for Linux were manually created.

INC += TUCamApi.h TUDefine.h

ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS_Linux += $(wildcard ../os/linux-x86_64/*.so*)

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

