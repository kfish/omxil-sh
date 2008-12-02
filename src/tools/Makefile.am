CFLAGS= -I$(srcdir)

bin_PROGRAMS = omxsh-decode-audio omxsh-decode-video

common_sources = tsemaphore.c

noinst_HEADERS = tsemaphore.h user_debug_levels.h \
                 omxsh-decode-audio.h \
                 omxsh-decode-video.h

omxsh_decode_audio_SOURCES = omxsh-decode-audio.c $(common_sources)
omxsh_decode_audio_CFLAGS = $(OMXIL_CFLAGS)
omxsh_decode_audio_LDADD = $(OMXIL_LIBS) -lpthread

omxsh_decode_video_SOURCES = omxsh-decode-video.c $(common_sources)
omxsh_decode_video_CFLAGS = $(OMXIL_CFLAGS)
omxsh_decode_video_LDADD = $(OMXIL_LIBS) -lpthread
