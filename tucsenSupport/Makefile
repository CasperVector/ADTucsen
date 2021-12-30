TOP=..
include $(TOP)/configure/CONFIG
#----------------------------------------
#  ADD MACRO DEFINITIONS AFTER THIS LINE
#=============================

# Note, the files in this directory were manually copied from the Andor SDK3 directories 
# for Linux and Windows.  The soft links for Linux were manually created.

INC += TUCamApi.h TUDefine.h

ifeq (linux-x86_64, $(findstring linux-x86_64, $(T_A)))
LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1
LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libTUCam.so.1.0.0
LIB_INSTALLS_Linux += ../os/linux-x86_64/libGCBase_gcc421_v3_0.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libGenApi_gcc421_v3_0.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/liblog4cpp_gcc421_v3_0.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libLog_gcc421_v3_0.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libMathParser_gcc421_v3_0.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libNodeMapData_gcc421_v3_0.so
LIB_INSTALLS_Linux += ../os/linux-x86_64/libXmlParser_gcc421_v3_0.so

endif

#=============================

include $(TOP)/configure/RULES
#----------------------------------------
#  ADD RULES AFTER THIS LINE

