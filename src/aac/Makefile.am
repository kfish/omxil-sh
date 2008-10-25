CFLAGS= -I$(srcdir) -m4-nofpu -Wa,-dsp

omxshaacdir = $(BELLAGIO_PLUGINSDIR)

if HAVE_HAACD
shaac_la = libomxshaac.la
endif

omxshaac_LTLIBRARIES = $(shaac_la)

libomxshaac_la_SOURCES = omx_shaac_component.c library_entry_point.c omx_shaac_component.h
libomxshaac_la_LIBADD = $(OMXIL_LIBS) $(HAACD_LIBS)
libomxshaac_la_CFLAGS = $(OMXIL_CFLAGS) $(HAACD_CFLAGS)
libomxshaac_la_LDFLAGS = $(OMXIL_LDFLAGS) $(HAACD_LDFLAGS)
