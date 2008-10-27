/**

  This component implements a HMP3D decoder. The HMP3D decoder is based on the
  libhmp3d API.

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
/** modification to include audio formats */
#include <OMX_Audio.h>

#include "omx_shmp3_component.h"

#define XADR    (HMP3D_T_XRAM *)0xe5007000
#define YADR    (HMP3D_T_YRAM *)0xe5017000

static void dsp_on(void)
{
        __asm__("nopx;nopy;");
}


#define MAX_COMPONENT_HMP3D 4
/** Maximum Number of HMP3D component instances */
static OMX_U32 noHMP3DDecInstance = 0;

/** The Constructor
  * @param cComponentName is the name of the component to be initialized
  */

OMX_ERRORTYPE omx_shmp3_component_Constructor( OMX_COMPONENTTYPE *openmaxStandComp, OMX_STRING cComponentName) {

  OMX_ERRORTYPE err = OMX_ErrorNone;  
  omx_shmp3_component_PrivateType* omx_shmp3_component_Private;
  omx_base_audio_PortType *inPort,*outPort;
  OMX_U32 i;

  if (!openmaxStandComp->pComponentPrivate) {
    DEBUG(DEB_LEV_FUNCTION_NAME, "In %s, allocating component\n", __func__);
    openmaxStandComp->pComponentPrivate = calloc(1, sizeof(omx_shmp3_component_PrivateType));
    if(openmaxStandComp->pComponentPrivate == NULL)  {
      return OMX_ErrorInsufficientResources;
    }
  }  else {
    DEBUG(DEB_LEV_FUNCTION_NAME, "In %s, Error Component %x Already Allocated\n", __func__, (int)openmaxStandComp->pComponentPrivate);
  }

  omx_shmp3_component_Private = openmaxStandComp->pComponentPrivate;
  omx_shmp3_component_Private->ports = NULL;

  /** we could create our own port structures here
    * fixme maybe the base class could use a "port factory" function pointer?  
    */
  err = omx_base_filter_Constructor(openmaxStandComp,cComponentName);

  /** Domain specific section for the ports. */  
  /** first we set the parameter common to both formats
    * parameters related to input port which does not depend upon input audio format
    */
  omx_shmp3_component_Private->sPortTypesParam[OMX_PortDomainAudio].nStartPortNumber = 0;
  omx_shmp3_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts = 2;

  /** Allocate Ports and call port constructor. */  
  if (omx_shmp3_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts && !omx_shmp3_component_Private->ports) {
    omx_shmp3_component_Private->ports = calloc(omx_shmp3_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts, sizeof(omx_base_PortType *));
    if (!omx_shmp3_component_Private->ports) {
      return OMX_ErrorInsufficientResources;
    }
    for (i=0; i < omx_shmp3_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++) {
      omx_shmp3_component_Private->ports[i] = calloc(1, sizeof(omx_base_audio_PortType));
      if (!omx_shmp3_component_Private->ports[i]) {
        return OMX_ErrorInsufficientResources;
      }
    }
  }

  base_audio_port_Constructor(openmaxStandComp, &omx_shmp3_component_Private->ports[0], 0, OMX_TRUE);
  base_audio_port_Constructor(openmaxStandComp, &omx_shmp3_component_Private->ports[1], 1, OMX_FALSE);

  /* parameters related to input port */
  inPort = (omx_base_audio_PortType *) omx_shmp3_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];

  inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE;
  strcpy(inPort->sPortParam.format.audio.cMIMEType, "audio/mpeg");
  inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingMP3;

  inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingMP3;

  setHeader(&omx_shmp3_component_Private->pAudioMp3,sizeof(OMX_AUDIO_PARAM_MP3TYPE));
  omx_shmp3_component_Private->pAudioMp3.nPortIndex = 0;
  omx_shmp3_component_Private->pAudioMp3.nChannels = 2;                                                                                                                          
  omx_shmp3_component_Private->pAudioMp3.nBitRate = 28000;
  omx_shmp3_component_Private->pAudioMp3.nSampleRate = 44100;
  omx_shmp3_component_Private->pAudioMp3.nAudioBandWidth = 0; 
  omx_shmp3_component_Private->pAudioMp3.eChannelMode = OMX_AUDIO_ChannelModeStereo;
  omx_shmp3_component_Private->pAudioMp3.eFormat=OMX_AUDIO_MP3StreamFormatMP1Layer3;
  
  /**  common parameters related to output port */

  outPort = (omx_base_audio_PortType *) omx_shmp3_component_Private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

  outPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingPCM;
  outPort->sPortParam.nBufferSize = DEFAULT_OUT_BUFFER_SIZE;

  outPort->sAudioParam.eEncoding = OMX_AUDIO_CodingPCM;

  /** settings of output port 
    * output is pcm audo format - so set the pcm mode settings
    */ 
  setHeader(&omx_shmp3_component_Private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
  omx_shmp3_component_Private->pAudioPcmMode.nPortIndex = 1;
  omx_shmp3_component_Private->pAudioPcmMode.nChannels = 2;
  omx_shmp3_component_Private->pAudioPcmMode.eNumData = OMX_NumericalDataSigned;
  omx_shmp3_component_Private->pAudioPcmMode.eEndian = OMX_EndianLittle;
  omx_shmp3_component_Private->pAudioPcmMode.bInterleaved = OMX_TRUE;
  omx_shmp3_component_Private->pAudioPcmMode.nBitPerSample = 16;
  omx_shmp3_component_Private->pAudioPcmMode.nSamplingRate = 44100;
  omx_shmp3_component_Private->pAudioPcmMode.ePCMMode = OMX_AUDIO_PCMModeLinear;
  omx_shmp3_component_Private->pAudioPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
  omx_shmp3_component_Private->pAudioPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;

  /** some more component private structure initialization */
  omx_shmp3_component_Private->BufferMgmtCallback = omx_shmp3_component_BufferMgmtCallback;  
  omx_shmp3_component_Private->messageHandler = omx_shmp3_component_MessageHandler;
  omx_shmp3_component_Private->destructor = omx_shmp3_component_Destructor;
  openmaxStandComp->SetParameter = omx_shmp3_component_SetParameter;
  openmaxStandComp->GetParameter = omx_shmp3_component_GetParameter;

  /** increase the counter of initialized components and check against the maximum limit */
  noHMP3DDecInstance++;

  /** now it's time to know the audio coding type of the component 
    * if audio coding type is set other than hmp3d then error returned
    */
  if(!strcmp(cComponentName, AUDIO_DEC_MP3_NAME)) {
    omx_shmp3_component_Private->audio_coding_type = OMX_AUDIO_CodingMP3;
  }  else if (!strcmp(cComponentName, AUDIO_DEC_BASE_NAME)) {
    omx_shmp3_component_Private->audio_coding_type = OMX_AUDIO_CodingUnused;
  }  else  {
    /** IL client specified an invalid component name */
    return OMX_ErrorInvalidComponentName;
  }

  if(!omx_shmp3_component_Private->avCodecSyncSem) {
    omx_shmp3_component_Private->avCodecSyncSem = calloc(1, sizeof(tsem_t));
    if(omx_shmp3_component_Private->avCodecSyncSem == NULL) {
      return OMX_ErrorInsufficientResources;
    }
    tsem_init(omx_shmp3_component_Private->avCodecSyncSem, 0);
  }
  if(noHMP3DDecInstance > MAX_COMPONENT_HMP3D) {
    return OMX_ErrorInsufficientResources;
  }

  return err;
}


/** The destructor
 */
OMX_ERRORTYPE omx_shmp3_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_shmp3_component_PrivateType* omx_shmp3_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_U32 i;

  if(omx_shmp3_component_Private->avCodecSyncSem) {
    tsem_deinit(omx_shmp3_component_Private->avCodecSyncSem);
    free(omx_shmp3_component_Private->avCodecSyncSem);
    omx_shmp3_component_Private->avCodecSyncSem = NULL;
  }

  /* frees port/s */
  if (omx_shmp3_component_Private->ports) {
    for (i=0; i < omx_shmp3_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++) {
      if(omx_shmp3_component_Private->ports[i])
        omx_shmp3_component_Private->ports[i]->PortDestructor(omx_shmp3_component_Private->ports[i]);
    }
    free(omx_shmp3_component_Private->ports);
    omx_shmp3_component_Private->ports=NULL;
  }

  DEBUG(DEB_LEV_FUNCTION_NAME, "Destructor of hmp3d decoder component is called\n");

  omx_base_filter_Destructor(openmaxStandComp);

  noHMP3DDecInstance--;

  return OMX_ErrorNone;
}

/** sets some parameters of the private structure for decoding */

void omx_shmp3_component_SetInternalParameters(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_shmp3_component_PrivateType* omx_shmp3_component_Private;
  omx_base_audio_PortType *pPort;

  omx_shmp3_component_Private = openmaxStandComp->pComponentPrivate;
  pPort = (omx_base_audio_PortType *) omx_shmp3_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
  
  strcpy(pPort->sPortParam.format.audio.cMIMEType, "audio/mpeg");
  pPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingMP3;

  setHeader(&omx_shmp3_component_Private->pAudioMp3,sizeof(OMX_AUDIO_PARAM_MP3TYPE));
  omx_shmp3_component_Private->pAudioMp3.nPortIndex = 0;
  omx_shmp3_component_Private->pAudioMp3.nChannels = 2;                                                                                                                          
  omx_shmp3_component_Private->pAudioMp3.nBitRate = 28000;
  omx_shmp3_component_Private->pAudioMp3.nSampleRate = 44100;
  omx_shmp3_component_Private->pAudioMp3.nAudioBandWidth = 0; 
  omx_shmp3_component_Private->pAudioMp3.eChannelMode = OMX_AUDIO_ChannelModeStereo;
  omx_shmp3_component_Private->pAudioMp3.eFormat=OMX_AUDIO_MP3StreamFormatMP1Layer3;
    
  setHeader(&pPort->sAudioParam, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
  pPort->sAudioParam.nPortIndex = 0;
  pPort->sAudioParam.nIndex = 0;
  pPort->sAudioParam.eEncoding = OMX_AUDIO_CodingMP3;
}

/** The Initialization function 
  */
OMX_ERRORTYPE omx_shmp3_component_Init(OMX_COMPONENTTYPE *openmaxStandComp)  {
  omx_shmp3_component_PrivateType* omx_shmp3_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_U32 nBufferSize;

  void *xrammap=XADR, *yrammap=YADR;

  DEBUG(DEB_LEV_FUNCTION_NAME, "In %s, initializing component\n", __func__);

  /** Temporary First Output buffer size*/
  nBufferSize = omx_shmp3_component_Private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX]->sPortParam.nBufferSize * 2;
  omx_shmp3_component_Private->internalOutputBuffer = malloc(nBufferSize);
  memset(omx_shmp3_component_Private->internalOutputBuffer, 0, nBufferSize);
  
  /** initializing hmp3d decoder parameters */
  DEBUG(DEB_LEV_FULL_SEQ, "Initializing hmp3d decoder parameters ...\n");
  dsp_on ();
  HMP3D_Open (&omx_shmp3_component_Private->mp3, xrammap, yrammap);

  return err;
};

/** The Deinitialization function 
  */
OMX_ERRORTYPE omx_shmp3_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_shmp3_component_PrivateType* omx_shmp3_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err = OMX_ErrorNone;

  free(omx_shmp3_component_Private->internalOutputBuffer);
  omx_shmp3_component_Private->internalOutputBuffer = NULL;
  
  /** reset the hmp3d decoder related parameters */
  HMP3D_Close (&omx_shmp3_component_Private->mp3);

  return err;
}


/** central buffer management function 
  * @param inputbuffer contains the input mp3 file content
  * @param outputbuffer is returned along with its output pcm file content that is produced as a result of this function execution
  */
void omx_shmp3_component_BufferMgmtCallback(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer) {

  omx_shmp3_component_PrivateType* omx_shmp3_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_S32 i, j;
  int ret;
  long len, outlen;
 
  DEBUG(DEB_LEV_FULL_SEQ, "input buf %x filled length : %d \n", (int)inputbuffer->pBuffer, (int)inputbuffer->nFilledLen);  

  outputbuffer->nFilledLen = 0;
  outputbuffer->nOffset = 0;

  len = inputbuffer->nFilledLen;
  outlen = outputbuffer->nAllocLen;
  DEBUG(DEB_LEV_FULL_SEQ,"Output buffer has length %ld, about to call HMP3D_Decode\n", outlen);
  
  // Process data: Copy decoded and converted data into outputbuffer->pBuffer, and set outputbuffer->nFIlledLen
  ret = HMP3D_Decode (&omx_shmp3_component_Private->mp3,
                      inputbuffer->pBuffer, &len,
                      (short *)outputbuffer->pBuffer, &outlen);
                      //(short *)outputbuffer->pBuffer, &outputbuffer->nFilledLen);
  DEBUG(DEB_LEV_FULL_SEQ,"Decoded outlen %ld, ret %d\n", outlen, ret);
  inputbuffer->nFilledLen = 0;
  outputbuffer->nFilledLen = outlen * 2;

  // Finish
  DEBUG(DEB_LEV_FULL_SEQ, "One output buffer %x len=%d is full returning\n", (int)outputbuffer->pBuffer, (int)outputbuffer->nFilledLen);  

  /* return output buffer */
}

/** setting parameter values
  * @param hComponent is handle of component
  * @param nParamIndex is the indextype of the parameter
  * @param ComponentParameterStructure is the input structure containing parameter setings
  */
OMX_ERRORTYPE omx_shmp3_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)  {
  
  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;
  OMX_AUDIO_PARAM_PCMMODETYPE* pAudioPcmMode;
  OMX_AUDIO_PARAM_MP3TYPE *pAudioMp3; 
  OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
  OMX_U32 portIndex;

  /** Check which structure we are being fed and make control its header */
  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
  omx_shmp3_component_PrivateType* omx_shmp3_component_Private = openmaxStandComp->pComponentPrivate;
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
      port = (omx_base_audio_PortType *) omx_shmp3_component_Private->ports[portIndex];
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
    memcpy(&omx_shmp3_component_Private->pAudioPcmMode, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));          
    break;

  case OMX_IndexParamAudioMp3:
    pAudioMp3 = (OMX_AUDIO_PARAM_MP3TYPE*)ComponentParameterStructure;
    portIndex = pAudioMp3->nPortIndex;
    err = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
    if(err!=OMX_ErrorNone) { 
      DEBUG(DEB_LEV_ERR, "In %s Parameter Check Error=%x\n",__func__,err); 
      break;
    }
    if(pAudioMp3->nPortIndex == 0)  {
      memcpy(&omx_shmp3_component_Private->pAudioMp3, pAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
    } else  {
      return OMX_ErrorBadPortIndex;
    }
    break;

  case OMX_IndexParamStandardComponentRole:
    pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
    if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_MP3_ROLE)) {
      omx_shmp3_component_Private->audio_coding_type = OMX_AUDIO_CodingMP3;
    } else {
      return OMX_ErrorBadParameter;
    }
    omx_shmp3_component_SetInternalParameters(openmaxStandComp);
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
OMX_ERRORTYPE omx_shmp3_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)  {
  
  OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;  
  OMX_AUDIO_PARAM_PCMMODETYPE *pAudioPcmMode;
  OMX_AUDIO_PARAM_MP3TYPE *pAudioMp3; 
  OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
  omx_base_audio_PortType *port;
  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
  omx_shmp3_component_PrivateType* omx_shmp3_component_Private = (omx_shmp3_component_PrivateType*)openmaxStandComp->pComponentPrivate;
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
    memcpy(ComponentParameterStructure, &omx_shmp3_component_Private->sPortTypesParam, sizeof(OMX_PORT_PARAM_TYPE));
    break;    

  case OMX_IndexParamAudioPortFormat:
    pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE))) != OMX_ErrorNone) { 
      break;
    }
    if (pAudioPortFormat->nPortIndex <= 1) {
      port = (omx_base_audio_PortType *)omx_shmp3_component_Private->ports[pAudioPortFormat->nPortIndex];
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
    memcpy(pAudioPcmMode, &omx_shmp3_component_Private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
    break;

  case OMX_IndexParamAudioMp3:
    pAudioMp3 = (OMX_AUDIO_PARAM_MP3TYPE*)ComponentParameterStructure;
    if(pAudioMp3->nPortIndex != 0) {
      return OMX_ErrorBadPortIndex;
    }
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_MP3TYPE))) != OMX_ErrorNone) { 
      break;
    }
    memcpy(pAudioMp3, &omx_shmp3_component_Private->pAudioMp3, sizeof(OMX_AUDIO_PARAM_MP3TYPE));
    break;

  case OMX_IndexParamStandardComponentRole:
    pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) { 
      break;
    }
    if (omx_shmp3_component_Private->audio_coding_type == OMX_AUDIO_CodingMP3) {
      strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_MP3_ROLE);
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
OMX_ERRORTYPE omx_shmp3_component_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp,internalRequestMessageType *message)  {
  omx_shmp3_component_PrivateType* omx_shmp3_component_Private = (omx_shmp3_component_PrivateType*)openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err;

  DEBUG(DEB_LEV_SIMPLE_SEQ, "In %s\n", __func__);

  if (message->messageType == OMX_CommandStateSet){
    if ((message->messageParam == OMX_StateIdle) && (omx_shmp3_component_Private->state == OMX_StateLoaded)) {
      DEBUG(DEB_LEV_SIMPLE_SEQ, "About to shmp3_component_Init()...\n");
      err = omx_shmp3_component_Init(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        DEBUG(DEB_LEV_ERR, "In %s hmp3d Decoder Init Failed=%x\n",__func__,err); 
        return err;
      }
    } else if ((message->messageParam == OMX_StateLoaded) && (omx_shmp3_component_Private->state == OMX_StateIdle)) {
      err = omx_shmp3_component_Deinit(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        DEBUG(DEB_LEV_ERR, "In %s HMP3D Decoder Deinit Failed=%x\n",__func__,err); 
        return err;
      }
    }
  }
  DEBUG(DEB_LEV_SIMPLE_SEQ, "Done, about to call base message handler()...\n");
  // Execute the base message handling
  return omx_base_component_MessageHandler(openmaxStandComp, message);
}
