CFLAGS= -I$(srcdir) -m4-nofpu -Wa,-dsp

omxhmp3ddir = $(BELLAGIO_PLUGINSDIR)

if HAVE_HMP3D
hmp3d_la = libomxhmp3d.la
endif

omxhmp3d_LTLIBRARIES = $(hmp3d_la)

libomxhmp3d_la_SOURCES = omx_hmp3d_component.c library_entry_point.c omx_hmp3d_component.h
libomxhmp3d_la_LIBADD = $(OMXIL_LIBS) $(HMP3D_LIBS)
libomxhmp3d_la_CFLAGS = $(OMXIL_CFLAGS) $(HMP3D_CFLAGS)
libomxhmp3d_la_LDFLAGS = $(OMXIL_LDFLAGS) $(HMP3D_LDFLAGS)
