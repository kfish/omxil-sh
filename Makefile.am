AM_CFLAGS = -I$(srcdir)

omxrsovpu4decdir = $(BELLAGIO_PLUGINSDIR)

omxrsovpu4dec_LTLIBRARIES = libomxrsovpu4dec.la

libomxrsovpu4dec_la_SOURCES = omx_rsovpu4dec_component.c library_entry_point.c omx_rsovpu4dec_component.h
libomxrsovpu4dec_la_LIBADD = $(OMXIL_LIBADD) $(RSOVPU4_LIBS)
libomxrsovpu4dec_la_CFLAGS = $(OMXIL_CFLAGS) $(RSOVPU4_CFLAGS)
libomxrsovpu4dec_la_LDFLAGS = $(OMXIL_LDFLAGS)
