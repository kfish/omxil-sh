CFLAGS= -I$(srcdir) -m4-nofpu -Wa,-dsp

omxhaacddir = $(BELLAGIO_PLUGINSDIR)

if HAVE_HAACD
haacd_la = libomxhaacd.la
endif

omxhaacd_LTLIBRARIES = $(haacd_la)

libomxhaacd_la_SOURCES = omx_haacd_component.c library_entry_point.c omx_haacd_component.h
libomxhaacd_la_LIBADD = $(OMXIL_LIBS) $(HAACD_LIBS)
libomxhaacd_la_CFLAGS = $(OMXIL_CFLAGS) $(HAACD_CFLAGS)
libomxhaacd_la_LDFLAGS = $(OMXIL_LDFLAGS) $(HAACD_LDFLAGS)
