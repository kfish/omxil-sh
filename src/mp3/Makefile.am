CFLAGS= -I$(srcdir) -m4-nofpu -Wa,-dsp

omxshmp3dir = $(BELLAGIO_PLUGINSDIR)

if HAVE_HMP3D
shmp3_la = libomxshmp3.la
endif

omxshmp3_LTLIBRARIES = $(shmp3_la)

libomxshmp3_la_SOURCES = omx_shmp3_component.c library_entry_point.c omx_shmp3_component.h
libomxshmp3_la_LIBADD = $(OMXIL_LIBS) $(HMP3D_LIBS)
libomxshmp3_la_CFLAGS = $(OMXIL_CFLAGS) $(HMP3D_CFLAGS)
libomxshmp3_la_LDFLAGS = $(OMXIL_LDFLAGS) $(HMP3D_LDFLAGS)
