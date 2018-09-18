@ECHO OFF

SET MAKE_TOOL=     *** Place MAKE utility path here ******
SET GNU_COMPILER=  *** Place GCC path here ***************
SET STLINK_UTIL=   *** Place ST-LINK utility path here ***

PATH=%MAKE_TOOL%;%GNU_COMPILER%;%STLINK_UTIL%

cd IDE\ARMGCC
make SHELL=cmd %1