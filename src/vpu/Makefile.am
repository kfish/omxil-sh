AM_CFLAGS = -I$(srcdir)

omxshvpudecdir = $(BELLAGIO_PLUGINSDIR)

if HAVE_SHCODECS
shcodecs_la = libomxshvpudec.la
endif

omxshvpudec_LTLIBRARIES = $(shcodecs_la)

libomxshvpudec_la_SOURCES = omx_shvpudec_component.c library_entry_point.c omx_shvpudec_component.h
libomxshvpudec_la_LIBADD = $(OMXIL_LIBS) $(SHCODECS_LIBS)
libomxshvpudec_la_CFLAGS = $(OMXIL_CFLAGS) $(SHCODECS_CFLAGS)
libomxshvpudec_la_LDFLAGS = $(OMXIL_LDFLAGS) $(SHCODECS_LDFLAGS)
