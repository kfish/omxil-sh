/**
  @file src/components/vorbis/omx_shmp3_component.h

  This component implements a HMP3D decoder. The HMP3D decoder is based on the
  libhmp3d API.

  Copyright (C) 2009 Renesas Technology Corp.

  Adapted from libomxil/src/components/vorbis/omx_vorbis_component.h

  Copyright (C) 2007  STMicroelectronics and Nokia

  This library is free software; you can redistribute it and/or modify it under
  the terms of the GNU Lesser General Public License as published by the Free
  Software Foundation; either version 2.1 of the License, or (at your option)
  any later version.

  This library is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public License for more
  details.

  You should have received a copy of the GNU Lesser General Public License
  along with this library; if not, write to the Free Software Foundation, Inc.,
  51 Franklin St, Fifth Floor, Boston, MA
  02110-1301  USA

  $Date$
  Revision $Rev$
  Author $Author$

*/

#ifndef _OMX_HMP3D_COMPONENT_H_
#define _OMX_HMP3D_COMPONENT_H_

#include <OMX_Types.h>
#include <OMX_Component.h>
#include <OMX_Core.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <omx_base_filter.h>

/* Specific include files for your codec */
#include <HMP3D_Mp3.h>

#define AUDIO_DEC_BASE_NAME "OMX.st.audio_decoder"
#define AUDIO_DEC_MP3_NAME "OMX.st.audio_decoder.mp3.shdsp"
#define AUDIO_DEC_MP3_ROLE "audio_decoder.mp3"

/** MyCodec component private structure.
 */
DERIVEDCLASS(omx_shmp3_component_PrivateType, omx_base_filter_PrivateType)
#define omx_shmp3_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
  /** @param mp3 HMP3D handle */  \
  HMP3D_T_MP3 mp3;  \
  /** @param semaphore for avcodec access syncrhonization */\
  tsem_t* avCodecSyncSem; \
  /** @param pAudioMp3 Reference to OMX_AUDIO_PARAM_MP3TYPE structure */ \
  OMX_AUDIO_PARAM_MP3TYPE pAudioMp3;  \
  /** @param pAudioPcmMode Referece to OMX_AUDIO_PARAM_PCMMODETYPE structure*/  \
  OMX_AUDIO_PARAM_PCMMODETYPE pAudioPcmMode;  \
  /** @param minBufferLength Field that stores the minimun allowed size for ffmpeg decoder */ \
  OMX_U16 minBufferLength; \
  /** @param internalOutputBuffer Field used for first internal output buffer */ \
  OMX_U8* internalOutputBuffer;\
  /** @param initState Field initialization state for the first buffer */ \
  OMX_S32 initState;\
  /** @param audio_coding_type Field that indicate the supported audio format of audio decoder */ \
  OMX_U32 audio_coding_type;
ENDCLASS(omx_shmp3_component_PrivateType)

/* Component private entry points declaration */
OMX_ERRORTYPE omx_shmp3_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName);
OMX_ERRORTYPE omx_shmp3_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_shmp3_component_Init(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_shmp3_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_shmp3_component_MessageHandler(OMX_COMPONENTTYPE*,internalRequestMessageType*);
  
void omx_shmp3_component_BufferMgmtCallback(
  OMX_COMPONENTTYPE *openmaxStandComp,
  OMX_BUFFERHEADERTYPE* inputbuffer,
  OMX_BUFFERHEADERTYPE* outputbuffer);

OMX_ERRORTYPE omx_shmp3_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_shmp3_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);

void omx_shmp3_component_SetInternalParameters(OMX_COMPONENTTYPE *openmaxStandComp);


#endif
