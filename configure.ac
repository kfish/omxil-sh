################################################################################
# openmax-sh: A collection of OpenMAX IL components for SH-Mobile              #
# Copyright (C) 2009 Renesas Technology Corp.                                  #
#                                                                              #
# This library is free software; you can redistribute it and/or                #
# modify it under the terms of the GNU Lesser General Public                   #
# License as published by the Free Software Foundation; either                 #
# version 2.1 of the License, or (at your option) any later version.           #
#                                                                              #
# This library is distributed in the hope that it will be useful,              #
# but WITHOUT ANY WARRANTY; without even the implied warranty of               #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU            #
# Lesser General Public License for more details.                              #
#                                                                              #
# You should have received a copy of the GNU Lesser General Public             #
# License along with this library; if not, write to the Free Software          #
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston MA  02110-1301 USA #
################################################################################

# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.61)
AC_INIT(openmax-sh, 0.6.0, [conrad@metadecks.org])
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([src/vpu/library_entry_point.c])
AC_CONFIG_HEADER([config.h])

# Checks for programs.
AC_PROG_CC
AC_PROG_LIBTOOL

# Checks for system libraries.

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([stdlib.h string.h])

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.
AC_FUNC_MALLOC
AC_CHECK_FUNCS([memset])

# Check for Bellagio
PKG_CHECK_MODULES([OMXIL], [libomxil-bellagio])

_PKG_CONFIG([pluginsdir], [variable=pluginsdir], [libomxil-bellagio])
BELLAGIO_PLUGINSDIR=${pkg_cv_pluginsdir}
AC_SUBST([BELLAGIO_PLUGINSDIR])

## Checks for codec libraries

# Check for HAACD
HAVE_HAACD="no"
HAACD_SUPPORT="no"

ac_enable_haacd=yes
AC_ARG_ENABLE(aac,
     [  --disable-aac           disable building of AAC support ],
     [ ac_enable_haacd=no ], [ ac_enable_haacd=yes] )

if test "x${ac_enable_haacd}" = xyes ; then
  #AC_CHECK_LIB(SACP1, HAACD_Open, HAVE_HAACD="maybe", , [-lm])

  # We cannot use AC_CHECK_LIB because libSACP1 requires the application to
  # implement a function called HAACD_GetData(); AC_CHECK_LIB will fail to link
  AC_MSG_CHECKING([for HAACD_Open in -lSACP1])

  ac_save_LIBS="$LIBS"
  LIBS="$LIBS -lSACP1"

  AC_TRY_COMPILE([#include <HAACD_ADL.h>
                  int HAACD_GetData (unsigned char *wpt, int ndata) { return 0; }],
                 [HAACD_AAC aac; unsigned char aac_stream[1024];
                  HAACD_Open (&aac, aac_stream, sizeof(aac_stream));],
                 [HAVE_HAACD="maybe"
                  AC_MSG_RESULT(yes)],
                 [AC_MSG_RESULT(no)])

  LIBS="$ac_save_LIBS"
                 
  # Ok, check done
  if test "x$HAVE_HAACD" = xmaybe ; then
    AC_CHECK_HEADER(HAACD_ADL.h, HAVE_HAACD="yes", HAVE_HAACD="no")
  fi
  if test "x$HAVE_HAACD" = xyes ; then
    AC_DEFINE(HAVE_HAACD, [1], [Define to 1 if you have HAACD])
    HAACD_LIBS="-lSACP1 -lm"
    AC_SUBST(HAACD_LIBS)
    HAACD_SUPPORT="yes (HAACD)"
  else
    AC_DEFINE(HAVE_HAACD, [0], [Define to 1 if you have HAACD])
  fi
else
  AC_DEFINE(HAVE_HAACD, [0], [Define to 1 if you have HAACD])
  HAACD_SUPPORT="disabled"
fi
AM_CONDITIONAL(HAVE_HAACD, [test "x$HAVE_HAACD" = "xyes"])

# Check for HMP3D
HAVE_HMP3D="no"
HMP3D_SUPPORT="no"

ac_enable_hmp3d=yes
AC_ARG_ENABLE(mp3,
     [  --disable-mp3           disable building of MP3 support ],
     [ ac_enable_hmp3d=no ], [ ac_enable_hmp3d=yes] )

if test "x${ac_enable_hmp3d}" = xyes ; then
  AC_CHECK_LIB(hmp3d, HMP3D_Open, HAVE_HMP3D="maybe", , [-lm])
  if test "x$HAVE_HMP3D" = xmaybe ; then
    AC_CHECK_HEADER(HMP3D_Mp3.h, HAVE_HMP3D="yes", HAVE_HMP3D="no")
  fi
  if test "x$HAVE_HMP3D" = xyes ; then
    AC_DEFINE(HAVE_HMP3D, [1], [Define to 1 if you have HMP3D])
    HMP3D_LIBS="-lhmp3d -lm"
    AC_SUBST(HMP3D_LIBS)
    HMP3D_SUPPORT="yes (HMP3D)"
  else
    AC_DEFINE(HAVE_HMP3D, [0], [Define to 1 if you have HMP3D])
  fi
else
  AC_DEFINE(HAVE_HMP3D, [0], [Define to 1 if you have HMP3D])
  HMP3D_SUPPORT="disabled"
fi
AM_CONDITIONAL(HAVE_HMP3D, [test "x$HAVE_HMP3D" = "xyes"])
  
# Check for shcodecs
HAVE_SHCODECS="no"
VPU_SUPPORT="no"

ac_enable_vpu=yes
AC_ARG_ENABLE(vpu,
     [  --disable-vpu           disable building of VPU support ],
     [ ac_enable_vpu=no ], [ ac_enable_vpu=yes] )

if test "x${ac_enable_vpu}" = xyes ; then
  PKG_CHECK_MODULES([SHCODECS], [shcodecs], HAVE_SHCODECS="yes", HAVE_SHCODECS="no")

  if test "x$HAVE_SHCODECS" = "xyes" ; then
    AC_DEFINE(HAVE_SHCODECS, [1], [Define to 1 if you have SHCODECS])
    AC_SUBST(SHCODECS_LIBS)
    VPU_SUPPORT="yes (SHCODECS)"
  else
    AC_DEFINE(HAVE_SHCODECS, [0], [Define to 1 if you have SHCODECS])
  fi
else
  AC_DEFINE(HAVE_SHCODECS, [0], [Define to 1 if you have SHCODECS])
  VPU_SUPPORT="disabled"
fi
AM_CONDITIONAL(HAVE_SHCODECS, [test "x$HAVE_SHCODECS" = "xyes"])

AC_CONFIG_FILES([
Makefile
src/Makefile
src/aac/Makefile
src/mp3/Makefile
src/vpu/Makefile
src/tools/Makefile
])
AC_OUTPUT

AC_MSG_RESULT([
------------------------------------------------------------------------
  $PACKAGE $VERSION:  Automatic configuration OK.

    AAC support: ................. $HAACD_SUPPORT
    MP3 support: ................. $HMP3D_SUPPORT
    VPU support: ................. $VPU_SUPPORT

------------------------------------------------------------------------
])

