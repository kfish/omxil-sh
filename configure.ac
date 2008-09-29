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

# Checks for codec libraries

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
    openmax_sh4_pkgdeps="$openmax_sh4_pkgdeps hmp3d"
  else
    AC_DEFINE(HAVE_HMP3D, [0], [Define to 1 if you have HMP3D])
  fi

  if test "x$HAVE_HMP3D" = "xyes" ; then
    HMP3D_LIBS="-lhmp3d -lm"
    AC_SUBST(HMP3D_LIBS)
    HMP3D_SUPPORT="yes"
  fi
else
  AC_DEFINE(HAVE_HMP3D, [0], [Define to 1 if you have HMP3D])
  HMP3D_SUPPORT="disabled"
fi
AM_CONDITIONAL(HAVE_HMP3D, [test "x$HAVE_HMP3D" = "xyes"])
  


PKG_CHECK_MODULES([RSOVPU4], [rsovpu4])

_PKG_CONFIG([pluginsdir], [variable=pluginsdir], [libomxil-bellagio])
BELLAGIO_PLUGINSDIR=${pkg_cv_pluginsdir}
AC_SUBST([BELLAGIO_PLUGINSDIR])

AC_CONFIG_FILES([Makefile src/Makefile src/rsovpu4/Makefile])
AC_OUTPUT

AC_MSG_RESULT([
------------------------------------------------------------------------
  $PACKAGE $VERSION:  Automatic configuration OK.

    HMP3D support: ............... $HMP3D_SUPPORT

------------------------------------------------------------------------
])

