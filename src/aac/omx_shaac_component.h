/**
  @file src/components/vorbis/omx_shaac_component.h

  This component implements a ogg-vorbis decoder. The vorbis decoder is based on libvorbis
  software library.

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

#ifndef _OMX_SHAAC_COMPONENT_H_
#define _OMX_SHAAC_COMPONENT_H_

#include <OMX_Types.h>
#include <OMX_Component.h>
#include <OMX_Core.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <omx_base_filter.h>

/* Specific include files for your codec */
#include <HAACD_ADL.h>

typedef enum
{
//  UNDEFINED_CHANNEL_MODE,
  MONO,
  STEREO,
  LC_STEREO,
  PS_STEREO
}
CHANNEL_MODE;

#define AUDIO_DEC_BASE_NAME "OMX.st.audio_decoder"
#define AUDIO_DEC_AAC_NAME "OMX.st.audio_decoder.aac.shdsp"
#define AUDIO_DEC_AAC_ROLE "audio_decoder.aac"

/** MyCodec component private structure.
 */
DERIVEDCLASS(omx_shaac_component_PrivateType, omx_base_filter_PrivateType)
#define omx_shaac_component_PrivateType_FIELDS omx_base_filter_PrivateType_FIELDS \
  /** @param aac HAACD handle */  \
  HAACD_AAC aac;  \
  /** @param aacdts ADTS(Audio Data Transport Stream) header */  \
  HAACD_AdtsHeader aacadts;  \
  /** @param pce PCD(Program Config Element) header */  \
  HAACD_PCE pce;  \
  /** @param decopt Decode option */  \
  int decopt;  \
  /** @param semaphore for avcodec access syncrhonization */\
  tsem_t* avCodecSyncSem; \
  /** @param pAudioAac Reference to OMX_AUDIO_PARAM_AACPROFILETYPE structure */ \
  OMX_AUDIO_PARAM_AACPROFILETYPE pAudioAac;  \
  /** @param pAudioPcmMode Referece to OMX_AUDIO_PARAM_PCMMODETYPE structure*/  \
  OMX_AUDIO_PARAM_PCMMODETYPE pAudioPcmMode;  \
  /** @param minBufferLength Field that stores the minimun allowed size for ffmpeg decoder */ \
  OMX_U16 minBufferLength; \
  /** @param inputCurrBuffer Field that stores pointer of the current input buffer position */ \
  OMX_U8* inputCurrBuffer;\
  /** @param inputCurrLength Field that stores current input buffer length in bytes */ \
  OMX_U32 inputCurrLength;\
  /** @param internalOutputBuffer Field used for first internal output buffer */ \
  OMX_U8* internalOutputBuffer;\
  /** @param isFirstBuffer Field that the buffer is the first buffer */ \
  OMX_S32 isFirstBuffer;\
  /** @param positionInOutBuf Field that used to calculate starting address of the next output frame to be written */ \
  OMX_S32 positionInOutBuf; \
  /** @param isNewBuffer Field that indicate a new buffer has arrived*/ \
  OMX_S32 isNewBuffer;  \
  /** @param audio_coding_type Field that indicate the supported audio format of audio decoder */ \
  OMX_U32 audio_coding_type;
ENDCLASS(omx_shaac_component_PrivateType)

/* Component private entry points declaration */
OMX_ERRORTYPE omx_shaac_component_Constructor(OMX_COMPONENTTYPE *openmaxStandComp,OMX_STRING cComponentName);
OMX_ERRORTYPE omx_shaac_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_shaac_component_Init(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_shaac_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp);
OMX_ERRORTYPE omx_shaac_component_MessageHandler(OMX_COMPONENTTYPE*,internalRequestMessageType*);
  
void omx_shaac_component_BufferMgmtCallbackHAACD(
  OMX_COMPONENTTYPE *openmaxStandComp,
  OMX_BUFFERHEADERTYPE* inputbuffer,
  OMX_BUFFERHEADERTYPE* outputbuffer);

OMX_ERRORTYPE omx_shaac_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure);

OMX_ERRORTYPE omx_shaac_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure);

void omx_shaac_component_SetInternalParameters(OMX_COMPONENTTYPE *openmaxStandComp);


#endif
