/**

  This component implements a HAACD decoder. The HAACD decoder is based on the
  libhaacd API.

  Copyright (C) 2009 Renesas Technology Corp.

  Adapted from libomxil/src/components/vorbis/omx_vorbis_component.c
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

#include "omx_shaac_component.h"

#define XRAM_P0 (short *)0xe5007000     // X memory page0 start address
#define YRAM_P0 (short *)0xe5017000     // X memory page0 start address

#define ADIF    1                       // bitstream format identifier - ADIF
#define ADTS    2                       // bitstream format identifier - ADTS
#define Raw     3                       // bitstream format identifier - RawDataStream
#define LC_PROF 1                       // Fixed LC profile

#define SUPPORT_SR      12
long a_sampl_rate[SUPPORT_SR] = {96000, 88200, 64000, 48000, 44100, 32000,              // AAC
                                 24000, 22050, 16000, 12000, 11025, 8000 };             // AAC & aacPlus

static void dsp_on(void)
{
        __asm__("nopx;nopy;");
}


/**
 * Maximum Number of HAACD component instances. As libSACP1 does not provide any
 * callback info arguments for HAACD_GetData(), we cannot differentiate between
 * different component instances. Hence we are limited to only 1 HAACD component.
 */
#define MAX_COMPONENT_HAACD 1
static OMX_U32 noHAACDDecInstance = 0;

/** The Constructor
  * @param cComponentName is the name of the component to be initialized
  */

OMX_ERRORTYPE omx_shaac_component_Constructor( OMX_COMPONENTTYPE *openmaxStandComp, OMX_STRING cComponentName) {

  OMX_ERRORTYPE err = OMX_ErrorNone;  
  omx_shaac_component_PrivateType* omx_shaac_component_Private;
  omx_base_audio_PortType *inPort,*outPort;
  OMX_U32 i;

  if (!openmaxStandComp->pComponentPrivate) {
    DEBUG(DEB_LEV_FUNCTION_NAME, "In %s, allocating component\n", __func__);
    openmaxStandComp->pComponentPrivate = calloc(1, sizeof(omx_shaac_component_PrivateType));
    if(openmaxStandComp->pComponentPrivate == NULL)  {
      return OMX_ErrorInsufficientResources;
    }
  }  else {
    DEBUG(DEB_LEV_FUNCTION_NAME, "In %s, Error Component %x Already Allocated\n", __func__, (int)openmaxStandComp->pComponentPrivate);
  }

  omx_shaac_component_Private = openmaxStandComp->pComponentPrivate;
  omx_shaac_component_Private->ports = NULL;

  /** we could create our own port structures here
    * fixme maybe the base class could use a "port factory" function pointer?  
    */
  err = omx_base_filter_Constructor(openmaxStandComp,cComponentName);

  /** Domain specific section for the ports. */  
  /** first we set the parameter common to both formats
    * parameters related to input port which does not depend upon input audio format
    */
  omx_shaac_component_Private->sPortTypesParam[OMX_PortDomainAudio].nStartPortNumber = 0;
  omx_shaac_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts = 2;

  /** Allocate Ports and call port constructor. */  
  if (omx_shaac_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts && !omx_shaac_component_Private->ports) {
    omx_shaac_component_Private->ports = calloc(omx_shaac_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts, sizeof(omx_base_PortType *));
    if (!omx_shaac_component_Private->ports) {
      return OMX_ErrorInsufficientResources;
    }
    for (i=0; i < omx_shaac_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++) {
      omx_shaac_component_Private->ports[i] = calloc(1, sizeof(omx_base_audio_PortType));
      if (!omx_shaac_component_Private->ports[i]) {
        return OMX_ErrorInsufficientResources;
      }
    }
  }

  base_audio_port_Constructor(openmaxStandComp, &omx_shaac_component_Private->ports[0], 0, OMX_TRUE);
  base_audio_port_Constructor(openmaxStandComp, &omx_shaac_component_Private->ports[1], 1, OMX_FALSE);

  /* parameters related to input port */
  inPort = (omx_base_audio_PortType *) omx_shaac_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];

  inPort->sPortParam.nBufferSize = DEFAULT_IN_BUFFER_SIZE;
  strcpy(inPort->sPortParam.format.audio.cMIMEType, "audio/mpeg");
  inPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingAAC;

  inPort->sAudioParam.eEncoding = OMX_AUDIO_CodingAAC;

  setHeader(&omx_shaac_component_Private->pAudioAac,sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
  omx_shaac_component_Private->pAudioAac.nPortIndex = 0;
  omx_shaac_component_Private->pAudioAac.nChannels = 2;                                                                                                                          
  omx_shaac_component_Private->pAudioAac.nBitRate = 28000;
  omx_shaac_component_Private->pAudioAac.nSampleRate = 44100;
  omx_shaac_component_Private->pAudioAac.nAudioBandWidth = 0; //encoder decides the needed bandwidth
  omx_shaac_component_Private->pAudioAac.eChannelMode = OMX_AUDIO_ChannelModeStereo;
  omx_shaac_component_Private->pAudioAac.nFrameLength = 0; //encoder decides the framelength
  
  /**  common parameters related to output port */

  outPort = (omx_base_audio_PortType *) omx_shaac_component_Private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX];

  outPort->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingPCM;
  outPort->sPortParam.nBufferSize = DEFAULT_OUT_BUFFER_SIZE;

  outPort->sAudioParam.eEncoding = OMX_AUDIO_CodingPCM;

  /** settings of output port 
    * output is pcm audo format - so set the pcm mode settings
    */ 
  setHeader(&omx_shaac_component_Private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
  omx_shaac_component_Private->pAudioPcmMode.nPortIndex = 1;
  omx_shaac_component_Private->pAudioPcmMode.nChannels = 2;
  omx_shaac_component_Private->pAudioPcmMode.eNumData = OMX_NumericalDataSigned;
  omx_shaac_component_Private->pAudioPcmMode.eEndian = OMX_EndianLittle;
  omx_shaac_component_Private->pAudioPcmMode.bInterleaved = OMX_TRUE;
  omx_shaac_component_Private->pAudioPcmMode.nBitPerSample = 16;
  omx_shaac_component_Private->pAudioPcmMode.nSamplingRate = 44100;
  omx_shaac_component_Private->pAudioPcmMode.ePCMMode = OMX_AUDIO_PCMModeLinear;
  omx_shaac_component_Private->pAudioPcmMode.eChannelMapping[0] = OMX_AUDIO_ChannelLF;
  omx_shaac_component_Private->pAudioPcmMode.eChannelMapping[1] = OMX_AUDIO_ChannelRF;

  /** some more component private structure initialization */
  omx_shaac_component_Private->BufferMgmtCallback = omx_shaac_component_BufferMgmtCallbackHAACD;  
  omx_shaac_component_Private->messageHandler = omx_shaac_component_MessageHandler;
  omx_shaac_component_Private->destructor = omx_shaac_component_Destructor;
  openmaxStandComp->SetParameter = omx_shaac_component_SetParameter;
  openmaxStandComp->GetParameter = omx_shaac_component_GetParameter;

  /** increase the counter of initialized components and check against the maximum limit */
  noHAACDDecInstance++;

  /** now it's time to know the audio coding type of the component 
    * if audio coding type is set other than haacd then error returned
    */
  if(!strcmp(cComponentName, AUDIO_DEC_AAC_NAME)) {
    omx_shaac_component_Private->audio_coding_type = OMX_AUDIO_CodingAAC;
  }  else if (!strcmp(cComponentName, AUDIO_DEC_BASE_NAME)) {
    omx_shaac_component_Private->audio_coding_type = OMX_AUDIO_CodingUnused;
  }  else  {
    /** IL client specified an invalid component name */
    return OMX_ErrorInvalidComponentName;
  }

  if(!omx_shaac_component_Private->avCodecSyncSem) {
    omx_shaac_component_Private->avCodecSyncSem = calloc(1, sizeof(tsem_t));
    if(omx_shaac_component_Private->avCodecSyncSem == NULL) {
      return OMX_ErrorInsufficientResources;
    }
    tsem_init(omx_shaac_component_Private->avCodecSyncSem, 0);
  }
  if(noHAACDDecInstance > MAX_COMPONENT_HAACD) {
    return OMX_ErrorInsufficientResources;
  }

  return err;
}


/** The destructor
 */
OMX_ERRORTYPE omx_shaac_component_Destructor(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_shaac_component_PrivateType* omx_shaac_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_U32 i;

  if(omx_shaac_component_Private->avCodecSyncSem) {
    tsem_deinit(omx_shaac_component_Private->avCodecSyncSem);
    free(omx_shaac_component_Private->avCodecSyncSem);
    omx_shaac_component_Private->avCodecSyncSem = NULL;
  }

  /* frees port/s */
  if (omx_shaac_component_Private->ports) {
    for (i=0; i < omx_shaac_component_Private->sPortTypesParam[OMX_PortDomainAudio].nPorts; i++) {
      if(omx_shaac_component_Private->ports[i])
        omx_shaac_component_Private->ports[i]->PortDestructor(omx_shaac_component_Private->ports[i]);
    }
    free(omx_shaac_component_Private->ports);
    omx_shaac_component_Private->ports=NULL;
  }

  DEBUG(DEB_LEV_FUNCTION_NAME, "Destructor of haacd decoder component is called\n");

  omx_base_filter_Destructor(openmaxStandComp);

  noHAACDDecInstance--;

  return OMX_ErrorNone;
}

/** sets some parameters of the private structure for decoding */

void omx_shaac_component_SetInternalParameters(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_shaac_component_PrivateType* omx_shaac_component_Private;
  omx_base_audio_PortType *pPort;

  omx_shaac_component_Private = openmaxStandComp->pComponentPrivate;
  
  if(omx_shaac_component_Private->audio_coding_type == OMX_AUDIO_CodingAAC)  {
    strcpy(omx_shaac_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.audio.cMIMEType, "audio/aac");
    omx_shaac_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX]->sPortParam.format.audio.eEncoding = OMX_AUDIO_CodingAAC;
                                                                                                                             
    setHeader(&omx_shaac_component_Private->pAudioAac,sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
    omx_shaac_component_Private->pAudioAac.nPortIndex = 0;
    omx_shaac_component_Private->pAudioAac.nChannels = 2;                                                                                                                          
    omx_shaac_component_Private->pAudioAac.nBitRate = 28000;
    omx_shaac_component_Private->pAudioAac.nSampleRate = 44100;
    omx_shaac_component_Private->pAudioAac.nAudioBandWidth = 0; // encoder decides the needed bandwidth
    omx_shaac_component_Private->pAudioAac.eChannelMode = OMX_AUDIO_ChannelModeStereo;
    omx_shaac_component_Private->pAudioAac.nFrameLength = 0; // encoder decides the framelength
    
    pPort = (omx_base_audio_PortType *) omx_shaac_component_Private->ports[OMX_BASE_FILTER_INPUTPORT_INDEX];
    setHeader(&pPort->sAudioParam, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE));
    pPort->sAudioParam.nPortIndex = 0;
    pPort->sAudioParam.nIndex = 0;
    pPort->sAudioParam.eEncoding = OMX_AUDIO_CodingAAC;
  }
}

/** The Initialization function 
  */
OMX_ERRORTYPE omx_shaac_component_Init(OMX_COMPONENTTYPE *openmaxStandComp)  {
  omx_shaac_component_PrivateType* omx_shaac_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_U32 nBufferSize;

  /** Temporary First Output buffer size*/
  nBufferSize = omx_shaac_component_Private->ports[OMX_BASE_FILTER_OUTPUTPORT_INDEX]->sPortParam.nBufferSize * 2;
  omx_shaac_component_Private->internalOutputBuffer = malloc(nBufferSize);
  memset(omx_shaac_component_Private->internalOutputBuffer, 0, nBufferSize);
  omx_shaac_component_Private->initState = 0;

  return err;
};

/** The Deinitialization function 
  */
OMX_ERRORTYPE omx_shaac_component_Deinit(OMX_COMPONENTTYPE *openmaxStandComp) {
  omx_shaac_component_PrivateType* omx_shaac_component_Private = openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err = OMX_ErrorNone;

  free(omx_shaac_component_Private->internalOutputBuffer);
  omx_shaac_component_Private->internalOutputBuffer = NULL;
  
  return err;
}

static int omx_shaac_component_DeferredInit(OMX_COMPONENTTYPE *openmaxStandComp,
                                            unsigned char * data, int len)
{
  omx_shaac_component_PrivateType* omx_shaac_component_Private = openmaxStandComp->pComponentPrivate;
  int ret = 0;

  /** initializing haacd decoder parameters */
  dsp_on ();

  HAACD_AllocXYRAM (XRAM_P0, 80, YRAM_P0, 80);

  HAACD_Open (&omx_shaac_component_Private->aac, data, len);

  HAACD_SetPCEArea(&omx_shaac_component_Private->aac, &omx_shaac_component_Private->pce);
  ret = HAACD_SetDecOpt(&omx_shaac_component_Private->aac, omx_shaac_component_Private->decopt);

  return 1;
}

static unsigned char * input;
static int remaining;

/* A function of this name is required by libSACP1. It fills a buffer of
 * input data and returns the number of bytes filled.
 */
int HAACD_GetData( unsigned char *wpt, int ndata )
{
  int copied=0;

  if (remaining > 0) {
    copied = remaining < ndata ? remaining : ndata ;
    memcpy (wpt, input, copied);

    input += copied;
    remaining -= copied;
  }

  return copied;
}

/** central buffer management function 
  * @param inputbuffer contains the input ogg file content
  * @param outputbuffer is returned along with its output pcm file content that is produced as a result of this function execution
  */
void omx_shaac_component_BufferMgmtCallbackHAACD(OMX_COMPONENTTYPE *openmaxStandComp, OMX_BUFFERHEADERTYPE* inputbuffer, OMX_BUFFERHEADERTYPE* outputbuffer) {

  omx_shaac_component_PrivateType* omx_shaac_component_Private = openmaxStandComp->pComponentPrivate;
  char *haacd_buffer;
  int ret;
  int pcnt = 0;
 
  DEBUG(DEB_LEV_FULL_SEQ, "input buf %x filled len : %d \n", (int)inputbuffer->pBuffer, (int)inputbuffer->nFilledLen);  

  outputbuffer->nFilledLen = 0;
  outputbuffer->nOffset = 0;

  if (omx_shaac_component_Private->initState == 0) {
    omx_shaac_component_DeferredInit (openmaxStandComp, inputbuffer->pBuffer, inputbuffer->nFilledLen);
    omx_shaac_component_Private->initState = 1;
  }

  /* Set up input, remaining static variables used by HAACD_GetData() above */
  input = inputbuffer->pBuffer;
  remaining = inputbuffer->nFilledLen;

  /* Process data: Copy decoded and converted data into outputbuffer->pBuffer, and set outputbuffer->nFIlledLen */
  ret = HAACD_DecodeInit (&omx_shaac_component_Private->aac,
                          inputbuffer->pBuffer, inputbuffer->nFilledLen);

  ret = HAACD_GetAdtsHeader (&omx_shaac_component_Private->aac,
                             &omx_shaac_component_Private->aacadts);

  ret = HAACD_Decode (&omx_shaac_component_Private->aac,
                      (short *)outputbuffer->pBuffer, &pcnt);

  if (ret == HAACD_ERR_DATA_EMPTY) {
    outputbuffer->nFilledLen = 0;
  } else if (ret >= 0) {
    outputbuffer->nFilledLen = pcnt*2;
  }

  if (omx_shaac_component_Private->initState == 1) {
    if (omx_shaac_component_Private->pAudioAac.nChannels != omx_shaac_component_Private->aac.ChannelNumber ||
    omx_shaac_component_Private->pAudioAac.nSampleRate != a_sampl_rate[omx_shaac_component_Private->aac.sampling_frequency_index]) {
      omx_shaac_component_Private->pAudioAac.nChannels = omx_shaac_component_Private->aac.ChannelNumber;
      omx_shaac_component_Private->pAudioAac.nSampleRate = a_sampl_rate[omx_shaac_component_Private->aac.sampling_frequency_index];

      /*Send Port Settings changed call back*/
      DEBUG(DEB_LEV_FULL_SEQ, "---->Sending Port Settings Change Event\n");

      (*(omx_shaac_component_Private->callbacks->EventHandler))
        (openmaxStandComp,
        omx_shaac_component_Private->callbackData,
        OMX_EventPortSettingsChanged, /* The command was completed */
        0,
        1, /* This is the output port index */
        NULL);

      /* Update initialization state (Done) */
    }

    omx_shaac_component_Private->initState = 2;
  }

  inputbuffer->nFilledLen = 0;

  // Finish
  DEBUG(DEB_LEV_FULL_SEQ, "One output buffer %x len=%d is full returning\n", (int)outputbuffer->pBuffer, (int)outputbuffer->nFilledLen);  

  return;
}

/** setting parameter values
  * @param hComponent is handle of component
  * @param nParamIndex is the indextype of the parameter
  * @param ComponentParameterStructure is the input structure containing parameter setings
  */
OMX_ERRORTYPE omx_shaac_component_SetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_IN  OMX_PTR ComponentParameterStructure)  {
  
  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;
  OMX_AUDIO_PARAM_PCMMODETYPE* pAudioPcmMode;
  OMX_AUDIO_PARAM_AACPROFILETYPE *pAudioAac; 
  OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
  OMX_U32 portIndex;

  /** Check which structure we are being fed and make control its header */
  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
  omx_shaac_component_PrivateType* omx_shaac_component_Private = openmaxStandComp->pComponentPrivate;
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
      port = (omx_base_audio_PortType *) omx_shaac_component_Private->ports[portIndex];
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
    memcpy(&omx_shaac_component_Private->pAudioPcmMode, pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));          
    break;

  case OMX_IndexParamAudioAac:
    pAudioAac = (OMX_AUDIO_PARAM_AACPROFILETYPE*)ComponentParameterStructure;
    portIndex = pAudioAac->nPortIndex;
    err = omx_base_component_ParameterSanityCheck(hComponent, portIndex, pAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
    if(err!=OMX_ErrorNone) { 
      DEBUG(DEB_LEV_ERR, "In %s Parameter Check Error=%x\n",__func__,err); 
      break;
    }
    if(pAudioAac->nPortIndex == 0)  {
      memcpy(&omx_shaac_component_Private->pAudioAac, pAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
    } else  {
      return OMX_ErrorBadPortIndex;
    }
    break;

  case OMX_IndexParamStandardComponentRole:
    pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
    if (!strcmp( (char*) pComponentRole->cRole, AUDIO_DEC_AAC_ROLE)) {
      omx_shaac_component_Private->audio_coding_type = OMX_AUDIO_CodingAAC;
    } else {
      return OMX_ErrorBadParameter;
    }
    omx_shaac_component_SetInternalParameters(openmaxStandComp);
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
OMX_ERRORTYPE omx_shaac_component_GetParameter(
  OMX_IN  OMX_HANDLETYPE hComponent,
  OMX_IN  OMX_INDEXTYPE nParamIndex,
  OMX_INOUT OMX_PTR ComponentParameterStructure)  {
  
  OMX_AUDIO_PARAM_PORTFORMATTYPE *pAudioPortFormat;  
  OMX_AUDIO_PARAM_PCMMODETYPE *pAudioPcmMode;
  OMX_AUDIO_PARAM_AACPROFILETYPE *pAudioAac; 
  OMX_PARAM_COMPONENTROLETYPE * pComponentRole;
  omx_base_audio_PortType *port;
  OMX_ERRORTYPE err = OMX_ErrorNone;
  OMX_COMPONENTTYPE *openmaxStandComp = (OMX_COMPONENTTYPE *)hComponent;
  omx_shaac_component_PrivateType* omx_shaac_component_Private = (omx_shaac_component_PrivateType*)openmaxStandComp->pComponentPrivate;
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
    memcpy(ComponentParameterStructure, &omx_shaac_component_Private->sPortTypesParam, sizeof(OMX_PORT_PARAM_TYPE));
    break;    

  case OMX_IndexParamAudioPortFormat:
    pAudioPortFormat = (OMX_AUDIO_PARAM_PORTFORMATTYPE*)ComponentParameterStructure;
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_PORTFORMATTYPE))) != OMX_ErrorNone) { 
      break;
    }
    if (pAudioPortFormat->nPortIndex <= 1) {
      port = (omx_base_audio_PortType *)omx_shaac_component_Private->ports[pAudioPortFormat->nPortIndex];
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
    memcpy(pAudioPcmMode, &omx_shaac_component_Private->pAudioPcmMode, sizeof(OMX_AUDIO_PARAM_PCMMODETYPE));
    break;

  case OMX_IndexParamAudioAac:
    pAudioAac = (OMX_AUDIO_PARAM_AACPROFILETYPE*)ComponentParameterStructure;
    if(pAudioAac->nPortIndex != 0) {
      return OMX_ErrorBadPortIndex;
    }
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE))) != OMX_ErrorNone) { 
      break;
    }
    memcpy(pAudioAac, &omx_shaac_component_Private->pAudioAac, sizeof(OMX_AUDIO_PARAM_AACPROFILETYPE));
    break;

  case OMX_IndexParamStandardComponentRole:
    pComponentRole = (OMX_PARAM_COMPONENTROLETYPE*)ComponentParameterStructure;
    if ((err = checkHeader(ComponentParameterStructure, sizeof(OMX_PARAM_COMPONENTROLETYPE))) != OMX_ErrorNone) { 
      break;
    }
    if (omx_shaac_component_Private->audio_coding_type == OMX_AUDIO_CodingAAC) {
      strcpy( (char*) pComponentRole->cRole, AUDIO_DEC_AAC_ROLE);
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
OMX_ERRORTYPE omx_shaac_component_MessageHandler(OMX_COMPONENTTYPE* openmaxStandComp,internalRequestMessageType *message)  {
  omx_shaac_component_PrivateType* omx_shaac_component_Private = (omx_shaac_component_PrivateType*)openmaxStandComp->pComponentPrivate;
  OMX_ERRORTYPE err;

  DEBUG(DEB_LEV_SIMPLE_SEQ, "In %s\n", __func__);

  if (message->messageType == OMX_CommandStateSet){
    if ((message->messageParam == OMX_StateIdle) && (omx_shaac_component_Private->state == OMX_StateLoaded)) {
      err = omx_shaac_component_Init(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        DEBUG(DEB_LEV_ERR, "In %s haacd Decoder Init Failed=%x\n",__func__,err); 
        return err;
      }
    } else if ((message->messageParam == OMX_StateLoaded) && (omx_shaac_component_Private->state == OMX_StateIdle)) {
      err = omx_shaac_component_Deinit(openmaxStandComp);
      if(err!=OMX_ErrorNone) { 
        DEBUG(DEB_LEV_ERR, "In %s HAACD Decoder Deinit Failed=%x\n",__func__,err); 
        return err;
      }
    }
  }
  // Execute the base message handling
  return omx_base_component_MessageHandler(openmaxStandComp, message);
}
