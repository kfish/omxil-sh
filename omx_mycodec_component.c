/**

  This component implements a MyCodec decoder. The MyCodec decoder is based on the
  libmycodec API.

  Based on src/components/vorbis/omx_vorbis_component.c
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

#include <omxcore.h>
#include <omx_base_audio_port.h>
#include <omx_mycodec_component.h>
/** modification to include audio formats */
#include <OMX_Audio.h>

#define MAX_COMPONENT_MYCODEC 4
/** Maximum Number of MyCodec component instances */
static OMX_U32 noMyCodecDecInstance = 0;

/** The Constructor
  * @param cComponentName is the name of the component to be initialized
  */

OMX_ERRORTYPE omx_mycodec_component_Constructor( OMX_COMPONENTTYPE *openmaxStandComp, OMX_STRING cComponentName) {

  OMX_ERRORTYPE err = OMX_ErrorNone;  
  omx_mycodec_component_PrivateType* omx_mycodec_component_Private;
  omx_base_audio_PortType *inPort,*outPort;
  OMX_U32 i;

  if (!openmaxStandComp->pComponentPrivate) {
    DEBUG(DEB_LEV_FUNCTION_NAME, "In %s, allocating component\n", __func__);
    openmaxStandComp->pComponentPrivate = calloc(1, sizeof(omx_mycodec_component_PrivateType));
    if(openmaxStandComp->pComponentPrivate == NULL)  {
      return OMX_ErrorInsufficientResources;
    }
  }  else {
    DEBUG(DEB_LEV_FUNCTION_NAME, "In %s, Error Component %x Already Allocated\n", __func__, (int)openmaxStandComp->pComponentPrivate);
  }

  omx_mycodec_component_Private = openmaxStandComp->pComponentPrivate;
  omx_mycodec_component_Private->ports = NULL;

  /** we could create our own port structures here
    * fixme maybe the base class could use a "port factory" function pointer?  
    */
  err = omx_base_filter_Constructor(openmaxStandComp,cComponentName);

  /** Domain specific section for the ports. */  
  /** first we set the parameter common to both formats
    * common parameters related to input port
    */

  /** Allocate Ports and call port constructor. */  
  if (omx_mycodec_component_Private->sPortTypesParam.nPorts && !omx_mycodec_component_Private->ports) {
    omx_mycodec_component_Private->ports = calloc(omx_mycodec_component_Private->sPortTypesParam.nPorts, sizeof(omx_base_PortType *));
    if (!omx_mycodec_component_Private->ports) {
      return OMX_ErrorInsufficientResources;
    }
    for (i=0; i < omx_mycodec_component_Private->sPortTypesParam.nPorts; i++) {
      omx_mycodec_component_Private->ports[i] = calloc(1, sizeof(omx_base_audio_PortType));
      if (!omx_mycodec_component_Private->ports[i]) {
        return OMX_ErrorInsufficientResources;
      }
    }
  }

  base_audio_port_Constructor(openmaxStandComp, &omx_mycodec_component_Private->ports[0], 0, OMX_TRUE);
  base_audio_port_Constructor(openmaxStandComp, &omx_mycodec_component_Private->ports[1], 1, OMX_FALSE);

  inPort = (omx_base_audio_PortType *) omx_mycodec_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];

  inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE;
  strcpy(inPort->sPortParam.format.audio.cMIMEType, "audio/mycodec");
  inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingFORMAT; // eg.

  inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingFORMAT;
                                                                                                                           
  setHeader(&omx_mycodec_component_Private->pAudioMyCodec,sizeof(OMX_AUDIO_PARAM_FORMATTYPE));
  omx_mycodec_component_Private->pAudioMyCodec.nPortIndex = 0;
  omx_mycodec_component_Private->pAudioMyCodec.nChannels = 2;                                                                                                                          
  omx_mycodec_component_Private->pAudioMyCodec.nBitRate = 28000;
  omx_mycodec_component_Private->pAudioMyCodec.nSampleRate = 44100;
  omx_mycodec_component_Private->pAudioMyCodec.nAudioBandWidth = 0; 
  omx_mycodec_component_Private->pAudioMyCodec.nQuality = 3; 
  
  /**  common parameters related to output port */

  outPort = (omx_base_audio_PortType *) omx_mycodec_component_Private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

  outPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingPCM;
  outPort->sPortParam.nBufferSize = DEFAULT_OUT_BUFFER_SIZE;

  outPort->sAudioParam.eEncoding = OMX_AUDIO_CodingPCM;

  /** settings of output port 
    * output is pcm audo format - so set the pcm mode settings
    */ 
  setHeader(&omx_mycodec_component_Private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
  omx_mycodec_component_Private->pAudioPcmMode.nPortIndex = 1;
  omx_mycodec_component_Private->pAudioPcmMode.nChannels = 2;
  omx_mycodec_component_Private->pAudioPcmMode.eNumData = OMX_NumericalDataSigned;
  omx_mycodec_component_Private->pAudioPcmMode.eEndian = OMX_EndianLittle;
  omx_mycodec_component_Private->pAudioPcmMode.bInterleaved = OMX_TRUE;
  omx_mycodec_component_Private->pAudioPcmMode.nBitPerSample = 16;
  omx_mycodec_component_Private->pAudioPcmMode.nSamplingRate = 44100;
  omx_mycodec_component_Private->pAudioPcmMode.ePCMMode = OMX_AUDIO_PCMModeLinear;
  omx_mycodec_component_Private->pAudioPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
  omx_mycodec_component_Private->pAudioPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;

  /** some more component private structure initialization */
  omx_mycodec_component_Private->BufferMgmtCallback = omx_mycodec_component_BufferMgmtCallbackMyCodec;  
  omx_mycodec_component_Private->messageHandler = omx_mycodec_component_MessageHandler;
  omx_mycodec_component_Private->destructor = omx_mycodec_component_Destructor;
  openmaxStandComp->SetParameter = omx_mycodec_component_SetParameter;
  openmaxStandComp->GetParameter = omx_mycodec_component_GetParameter;

  /** increase the counter of initialized components and check against the maximum limit */
  noMyCodecDecInstance++;

  /** now it's time to know the audio coding type of the component 
    * if audio coding type is set other than mycodec then error returned
    */
  if(!strcmp(cComponentName, AUDIO_DEC_FORMAT_NAME)) {
    omx_mycodec_component_Private->audio_coding_type = OMX_AUDIO_CodingFORMAT;
  }  else if (!strcmp(cComponentName, AUDIO_DEC_BASE_NAME)) {
    omx_mycodec_component_Private->audio_coding_type = OMX_AUDIO_CodingUnused;
  }  else  {
    /** IL client specified an invalid component name */
    return OMX_ErrorInvalidComponentName;
  }

  if(!omx_mycodec_component_Private->avCodecSyncSem) {
    omx_mycodec_component_Private->avCodecSyncSem = calloc(1, sizeof(tsem_t));
    if(omx_mycodec_component_Private->avCodecSyncSem == NULL) {
      return OMX_ErrorInsufficientResources;
    }
    tsem_init(omx_mycodec_component_Private->avCodecSyncSem, 0);
  }
  if(noMyCodecDecInstance > MAX_COMPONENT_FORMATDEC) {
    return OMX_ErrorInsufficientResources;
  }

  return err;
}


/** The destructor
 */
OMX_ERRORTYPE omx_mycodec_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_mycodec_component_PrivateType* omx_mycodec_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_U32 i;

  if(omx_mycodec_component_Private->avCodecSyncSem) {
    tsem_deinit(omx_mycodec_component_Private->avCodecSyncSem);
    free(omx_mycodec_component_Private->avCodecSyncSem);
    omx_mycodec_component_Private->avCodecSyncSem = NULL;
  }

  /* frees port/s */
  if (omx_mycodec_component_Private->ports) {
    for (i=0; i < omx_mycodec_component_Private->sPortTypesParam.nPorts; i++) {
      if(omx_mycodec_component_Private->ports[i])
        omx_mycodec_component_Private->ports[i]->PortDestructor(omx_mycodec_component_Private->ports[i]);
    }
    free(omx_mycodec_component_Private->ports);
    omx_mycodec_component_Private->ports=NULL;
  }

  DEBUG(DEB_LEV_FUNCTION_NAME, "Destructor of mycodec decoder component is called\n");

  omx_base_filter_Destructor(openmaxStandComp);

  noMyCodecDecInstance--;

  return OMX_ErrorNone;
}

/** sets some parameters of the private structure for decoding */

void omx_mycodec_component_SetInternalParameters(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_mycodec_component_PrivateType* omx_mycodec_component_Private;
  omx_base_audio_PortType *pPort;

  omx_mycodec_component_Private = openmaxStandComp->pComponentPrivate;
  
  if(omx_mycodec_component_Private->audio_coding_type == OMX_AUDIO_CodingFORMAT)  {
    strcpy(omx_mycodec_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.audio.cMIMEType, "audio/mycodec");
    omx_mycodec_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingFORMAT;
                                                                                                                             
    setHeader(&omx_mycodec_component_Private->pAudioMyCodec,sizeof(OMX_AUDIO_PARAM_FORMATTYPE));
    omx_mycodec_component_Private->pAudioMyCodec.nPortIndex = 0;
    omx_mycodec_component_Private->pAudioMyCodec.nChannels = 2;                                                                                                                          
    omx_mycodec_component_Private->pAudioMyCodec.nBitRate = 28000;
    omx_mycodec_component_Private->pAudioMyCodec.nSampleRate = 44100;
    omx_mycodec_component_Private->pAudioMyCodec.nAudioBandWidth = 0; 
    omx_mycodec_component_Private->pAudioMyCodec.nQuality = 3; 
    
    pPort = (omx_base_audio_PortType *) omx_mycodec_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
    setHeader(&pPort->sAudioParam, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
    pPort->sAudioParam.nPortIndex = 0;
    pPort->sAudioParam.nIndex = 0;
    pPort->sAudioParam.eEncoding = OMX_AUDIO_CodingFORMAT;
  }
}

/** The Initialization function 
  */
OMX_ERRORTYPE omx_mycodec_component_Init(OMX_COMPONENTTYPE *openmaxStandComp)  {
  omx_mycodec_component_PrivateType* omx_mycodec_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_U32 nBufferSize;

  /** Temporary First Output buffer size*/
  omx_mycodec_component_Private->inputCurrBuffer = NULL;
  omx_mycodec_component_Private->inputCurrLength = 0;
  nBufferSize = omx_mycodec_component_Private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX]->sPortParam.nBufferSize * 2;
  omx_mycodec_component_Private->internalOutputBuffer = malloc(nBufferSize);
  memset(omx_mycodec_component_Private->internalOutputBuffer, 0, nBufferSize);
  omx_mycodec_component_Private->isFirstBuffer = 1;
  omx_mycodec_component_Private->positionInOutBuf = 0;
  omx_mycodec_component_Private->isNewBuffer = 1;
  
  /** initializing mycodec decoder parameters */
  // omx_mycodec_component_Private->codec = mycodec_init(...);
                                                                                                                             
  return err;
};

/** The Deinitialization function 
  */
OMX_ERRORTYPE omx_mycodec_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_mycodec_component_PrivateType* omx_mycodec_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err = OMX_ErrorNone;

  free(omx_mycodec_component_Private->internalOutputBuffer);
  omx_mycodec_component_Private->internalOutputBuffer = NULL;
  
  /** reset the mycodec decoder related parameters */
  // mycodec_delete (omx_mycodec_component_Private->codec);
                                                                                                                             
  return err;
}


/** central buffer management function 
  * @param inputbuffer contains the input ogg file content
  * @param outputbuffer is returned along with its output pcm file content that is produced as a result of this function execution
  */
void omx_mycodec_component_BufferMgmtCallbackMyCodec(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer) {

  omx_mycodec_component_PrivateType* omx_mycodec_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_U8* outputCurrBuffer;
  OMX_U32 outputLength;
  OMX_S32 result;  
  float **pcm;
  OMX_S32 samples;
  OMX_S32 i, j;
  OMX_S32 bout;
  OMX_S32 clipflag=0;
  int val;
  float  *mono;
  static OMX_S32 index=0;
  int eos=0;
  char *mycodec_buffer;
  ogg_int16_t convbuffer[4096];

 
  DEBUG(DEB_LEV_FULL_SEQ, "input buf %x filled len : %d \n", (int)inputbuffer->pBuffer, (int)inputbuffer->nFilledLen);  
  /** Fill up the current input buffer when a new buffer has arrived */
  if(omx_mycodec_component_Private->isNewBuffer) {
    omx_mycodec_component_Private->inputCurrBuffer = inputbuffer->pBuffer;
    omx_mycodec_component_Private->inputCurrLength = inputbuffer->nFilledLen;
    omx_mycodec_component_Private->positionInOutBuf = 0;

    DEBUG(DEB_LEV_SIMPLE_SEQ, "new -- input buf %x filled len : %d \n", (int)inputbuffer->pBuffer, (int)inputbuffer->nFilledLen);  

    /** for each new input buffer --- feed into private decoder structure data */
    // ret = mycodec_decode (inputbuffer->pBuffer, nFilledLen);
    DEBUG(DEB_LEV_FULL_SEQ,"***** bytes read to buffer (of first header): %d \n",(int)inputbuffer->nFilledLen);
  }
  outputCurrBuffer = outputbuffer->pBuffer;
  outputLength = outputbuffer->nAllocLen;
  outputbuffer->nFilledLen = 0;
  outputbuffer->nOffset = 0;
  
  // Process headers and data
#if 0
  if (ret == MYCODEC_HEADER) {
    // handle headers, reinitialize etc.
  } else {
    // Process data: Copy decoded and converted data into outputbuffer->pBuffer, and set outputbuffer->nFIlledLen
  }
#endif

  // Finish
  DEBUG(DEB_LEV_FULL_SEQ, "One output buffer %x len=%d is full returning\n", (int)outputbuffer->pBuffer, (int)outputbuffer->nFilledLen);  

  return;
}

/** setting parameter values
  * @param hComponent is handle of component
  * @param nParamIndex is the indextype of the parameter
  * @param ComponentParameterStructure is the input structure containing parameter setings
  */
OMX_ERRORTYPE omx_mycodec_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)  {
  
  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;
  OMX_AUDIO_PARAM_PCMMODETYPE* pAudioPcmMode;
  OMX_AUDIO_PARAM_FORMATTYPE *pAudioMyCodec; 
  OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
  OMX_U32 portIndex;

  /** Check which structure we are being fed and make control its header */
  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
  omx_mycodec_component_PrivateType* omx_mycodec_component_Private = openmaxStandComp->pComponentPrivate;
  omx_base_audio_PortType *port;
  if (ComponentParameterStructure == NULL) {
    return OMX_ErrorBadParameter;
  }

  DEBUG(DEB_LEV_SIMPLE_SEQ, "   Setting parameter %i\n", nParamIndex);
  switch (nParamIndex) {
  case OMX_IndexParamAudioPortFormat:
    pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
    portIndex = pAudioPortFormat->nPortIndex;
    /** Check Structure Header and verify component state */
    err = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioPortFormat, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
    if(err!=OMX_ErrorNone) { 
      DEBUG(DEB_LEV_ERR, "In %s Parameter Check Error=%x\n",__func__,err); 
      break;
    }
    if (portIndex <= 1) {
      port = (omx_base_audio_PortType *) omx_mycodec_component_Private->ports[portIndex];
      memcpy(&port->sAudioParam,pAudioPortFormat, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
    } else {
      return OMX_ErrorBadPortIndex;
    }
    break;  
      
  case OMX_IndexParamAudioPcm:
    pAudioPcmMode = (OMX_AUDIO_PARAM_PCMMODETYPE*)ComponentParameterStructure;
    portIndex = pAudioPcmMode->nPortIndex;
    /*Check Structure Header and verify component state*/
    err = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
    if(err!=OMX_ErrorNone) { 
      DEBUG(DEB_LEV_ERR, "In %s Parameter Check Error=%x\n",__func__,err); 
      break;
    }
    memcpy(&omx_mycodec_component_Private->pAudioPcmMode, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));          
    break;

  case OMX_IndexParamAudioMyCodec:
    pAudioMyCodec = (OMX_AUDIO_PARAM_FORMATTYPE*)ComponentParameterStructure;
    portIndex = pAudioMyCodec->nPortIndex;
    err = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioMyCodec, sizeof(OMX_AUDIO_PARAM_FORMATTYPE));
    if(err!=OMX_ErrorNone) { 
      DEBUG(DEB_LEV_ERR, "In %s Parameter Check Error=%x\n",__func__,err); 
      break;
    }
    if(pAudioMyCodec->nPortIndex == 0)  {
      memcpy(&omx_mycodec_component_Private->pAudioMyCodec, pAudioMyCodec, sizeof(OMX_AUDIO_PARAM_FORMATTYPE));
    } else  {
      return OMX_ErrorBadPortIndex;
    }
    break;

  case OMX_IndexParamStandardComponentRole:
    pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
    if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_FORMAT_ROLE)) {
      omx_mycodec_component_Private->audio_coding_type = OMX_AUDIO_CodingFORMAT;
    } else {
      return OMX_ErrorBadParameter;
    }
    omx_mycodec_component_SetInternalParameters(openmaxStandComp);
    break;

  default: /*Call the base component function*/
    return omx_base_component_SetParameter(hComponent, nParamIndex, ComponentParameterStructure);
  }
  return err;
}

/** getting parameter values
  * @param hComponent is handle of component
  * @param nParamIndex is the indextype of the parameter
  * @param ComponentParameterStructure is the structure to contain obtained parameter setings
  */
OMX_ERRORTYPE omx_mycodec_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)  {
  
  OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;  
  OMX_AUDIO_PARAM_PCMMODETYPE *pAudioPcmMode;
  OMX_AUDIO_PARAM_FORMATTYPE *pAudioMyCodec; 
  OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
  omx_base_audio_PortType *port;
  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
  omx_mycodec_component_PrivateType* omx_mycodec_component_Private = (omx_mycodec_component_PrivateType*)openmaxStandComp->pComponentPrivate;
  if (ComponentParameterStructure == NULL) {
    return OMX_ErrorBadParameter;
  }
  DEBUG(DEB_LEV_SIMPLE_SEQ, "   Getting parameter %i\n", nParamIndex);
  /* Check which structure we are being fed and fill its header */
  switch(nParamIndex) {
  
  case OMX_IndexParamAudioInit:
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_PORT_PARAM_TYPE))) != OMX_ErrorNone) { 
      break;
    }
    memcpy(ComponentParameterStructure, &omx_mycodec_component_Private->sPortTypesParam, sizeof(OMX_PORT_PARAM_TYPE));
    break;    

  case OMX_IndexParamAudioPortFormat:
    pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE))) != OMX_ErrorNone) { 
      break;
    }
    if (pAudioPortFormat->nPortIndex <= 1) {
      port = (omx_base_audio_PortType *)omx_mycodec_component_Private->ports[pAudioPortFormat->nPortIndex];
      memcpy(pAudioPortFormat, &port->sAudioParam, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
    } else {
      return OMX_ErrorBadPortIndex;
    }
    break;    

  case OMX_IndexParamAudioPcm:
    pAudioPcmMode = (OMX_AUDIO_PARAM_PCMMODETYPE*)ComponentParameterStructure;
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE))) != OMX_ErrorNone) { 
      break;
    }
    if (pAudioPcmMode->nPortIndex > 1) {
      return OMX_ErrorBadPortIndex;
    }
    memcpy(pAudioPcmMode, &omx_mycodec_component_Private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
    break;

  case OMX_IndexParamAudioMyCodec:
    pAudioMyCodec = (OMX_AUDIO_PARAM_FORMATTYPE*)ComponentParameterStructure;
    if(pAudioMyCodec->nPortIndex != 0) {
      return OMX_ErrorBadPortIndex;
    }
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_FORMATTYPE))) != OMX_ErrorNone) { 
      break;
    }
    memcpy(pAudioMyCodec, &omx_mycodec_component_Private->pAudioMyCodec, sizeof(OMX_AUDIO_PARAM_FORMATTYPE));
    break;

  case OMX_IndexParamStandardComponentRole:
    pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) { 
      break;
    }
    if (omx_mycodec_component_Private->audio_coding_type == OMX_AUDIO_CodingFORMAT) {
      strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_FORMAT_ROLE);
    } else {
      strcpy( (char*) pComponentRole->cRole, "\0");;
    }
    break;

  default: /*Call the base component function*/
    return omx_base_component_GetParameter(hComponent, nParamIndex, ComponentParameterStructure);
  }
  return err;
}

/** handles the message generated by the IL client 
  * @param message is the message type
  */
OMX_ERRORTYPE omx_mycodec_component_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp,internalRequestMessageType *message)  {
  omx_mycodec_component_PrivateType* omx_mycodec_component_Private = (omx_mycodec_component_PrivateType*)openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err;

  DEBUG(DEB_LEV_SIMPLE_SEQ, "In %s\n", __func__);

  if (message->messageType == OMX_CommandStateSet){
    if ((message->messageParam == OMX_StateIdle) && (omx_mycodec_component_Private->state == OMX_StateLoaded)) {
      err = omx_mycodec_component_Init(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        DEBUG(DEB_LEV_ERR, "In %s mycodec Decoder Init Failed=%x\n",__func__,err); 
        return err;
      }
    } else if ((message->messageParam == OMX_StateLoaded) && (omx_mycodec_component_Private->state == OMX_StateIdle)) {
      err = omx_mycodec_component_Deinit(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        DEBUG(DEB_LEV_ERR, "In %s MyCodec Decoder Deinit Failed=%x\n",__func__,err); 
        return err;
      }
    }
  }
  // Execute the base message handling
  return omx_base_component_MessageHandler(openmaxStandComp, message);
}
