#                                               -*- Autoconf -*-
# Process this file with autoconf to produce a configure script.

AC_PREREQ(2.61)
AC_INIT(omxil-component-mycodec, 0.5.0, [my.email.address@example.com])
AM_INIT_AUTOMAKE
AC_CONFIG_SRCDIR([library_entry_point.c])
AC_CONFIG_HEADER([config.h])

# Checks for programs.
AC_PROG_CC
AC_PROG_LIBTOOL

# Checks for libraries.

# Checks for header files.
AC_HEADER_STDC
AC_CHECK_HEADERS([stdlib.h string.h])

# Checks for typedefs, structures, and compiler characteristics.

# Checks for library functions.
AC_FUNC_MALLOC
AC_CHECK_FUNCS([memset])

# Add a check for your codec library here, eg.
# PKG_CHECK_MODULES([MYCODEC], [mycodec])

PKG_CHECK_MODULES([OMXIL], [libomxil-bellagio])

_PKG_CONFIG([pluginsdir], [variable=pluginsdir], [libomxil-bellagio])
BELLAGIO_PLUGINSDIR=${pkg_cv_pluginsdir}
AC_SUBST([BELLAGIO_PLUGINSDIR])

AC_CONFIG_FILES([Makefile])
AC_OUTPUT
