#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.61)
AC_INIT(openmax-sh4, 0.5.0, [conrad@metadecks.org])
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([src/rsovpu4/library_entry_point.c])
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
AC_ARG_ENABLE(haacd,
     [  --disable-haacd         disable building of HAACD support ],
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
    HAACD_SUPPORT="yes"
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
AC_ARG_ENABLE(hmp3d,
     [  --disable-hmp3d         disable building of HMP3D support ],
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
    HMP3D_SUPPORT="yes"
  else
    AC_DEFINE(HAVE_HMP3D, [0], [Define to 1 if you have HMP3D])
  fi
else
  AC_DEFINE(HAVE_HMP3D, [0], [Define to 1 if you have HMP3D])
  HMP3D_SUPPORT="disabled"
fi
AM_CONDITIONAL(HAVE_HMP3D, [test "x$HAVE_HMP3D" = "xyes"])
  
# Check for rsovpu4
HAVE_RSOVPU4="no"
RSOVPU4_SUPPORT="no"

ac_enable_rsovpu4=yes
AC_ARG_ENABLE(rsovpu4,
     [  --disable-rsovpu4       disable building of RSOVPU4 support ],
     [ ac_enable_rsovpu4=no ], [ ac_enable_rsovpu4=yes] )

if test "x${ac_enable_rsovpu4}" = xyes ; then
  PKG_CHECK_MODULES([RSOVPU4], [rsovpu4], HAVE_RSOVPU4="yes", HAVE_RSOVPU4="no")

  if test "x$HAVE_RSOVPU4" = "xyes" ; then
    AC_DEFINE(HAVE_RSOVPU4, [1], [Define to 1 if you have RSOVPU4])
    AC_SUBST(RSOVPU4_LIBS)
    RSOVPU4_SUPPORT="yes"
  else
    AC_DEFINE(HAVE_RSOVPU4, [0], [Define to 1 if you have RSOVPU4])
  fi
else
  AC_DEFINE(HAVE_RSOVPU4, [0], [Define to 1 if you have RSOVPU4])
  RSOVPU4_SUPPORT="disabled"
fi
AM_CONDITIONAL(HAVE_RSOVPU4, [test "x$HAVE_RSOVPU4" = "xyes"])

AC_CONFIG_FILES([Makefile src/Makefile src/haacd/Makefile src/hmp3d/Makefile src/rsovpu4/Makefile])
AC_OUTPUT

AC_MSG_RESULT([
------------------------------------------------------------------------
  $PACKAGE $VERSION:  Automatic configuration OK.

    HAACD support: ............... $HAACD_SUPPORT
    HMP3D support: ............... $HMP3D_SUPPORT
    RSOVPU4 support: ............. $RSOVPU4_SUPPORT

------------------------------------------------------------------------
])

