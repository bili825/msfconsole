/******************************************************************************
 *
 * 版权信息：Copyright (c) 2017, 杭州海康汽车技术有限公司，保留所有版权
 *
 * 文件名称：HAT_AD_algProcess.c

 * 摘    要：
 *
 * 当前版本：
 * 作    者：
 * 日    期：
 * 备    注：
 *
 * 历史记录: <版本> <时间> <作者> <修改描述>
   a)

 *******************************************************************************/

#include "hikcType.h"
#include "interface.h"
#include "HAT_AD_alarm.h"
#include "HAT_AD_algProcess.h"
#include "HAT_AD_Algorithm.h"
#include "hikCType.h"
#include "HAT_AD_ipc.h"
#include "debug.h"
#include "mss.h"
#include "common.h"
#include "util_mem.h"
#include "register.h"
#include "vca_base.h"
#include "task.h"
#include "HAT_AD_algStatus.h"
#include "string.h"
#include "NvtIpcInt.h"

#ifdef VCA_DBA
#include "HAT_AD_dbaInterface.h"
#endif
#ifdef VCA_BSD
#include "HAT_AD_bsdInterface.h"
#endif
#ifdef VCA_FCW
#include "HAT_AD_fcwInterface.h"
#endif

/* 打印接口定义 */
#define ALG_PROCESS_PRT		  DBG_MSG
#define ALG_PROCESS_WARNING	  DBG_WRN
#define ALG_PROCESS_DEBUG	  DBG_WRN
#define ALG_PROCESS_ERR		  DBG_ERR

typedef struct _HAT_AD_ALG_CTRL_
{
	UINT32 algVideoCnt;
	UINT32 algVideoChn[MAX_YUV_FRAME];
	HAT_AD_SMART_MODE algMode;
}HAT_AD_ALG_CTRL;

/**********************************定义全局变量**********************************/
extern HAT_AD_ALG_SHARE_MEM *pDspShareMem;
static HAT_AD_ALG_CTRL gAlgCtrl = {0};

void* alloc_buffer(VCA_MEM_BUF *pMemBuf, unsigned int nSize, unsigned int alignment)
{
    void        *buf_res = NULL;        // 分配的内存结果
    int         free_sz;                // 可用内存大小

    CHECK_ERROR( pMemBuf == NULL, NULL);

    if( ((unsigned int)pMemBuf->cur_pos % alignment) == 0 )
    {
        buf_res = (void*)pMemBuf->cur_pos;
    }
    else
    {
        /* 缓存中空余内存起始位置 */
        buf_res = (void*)(((unsigned int)pMemBuf->cur_pos + (alignment-1)) & (~(alignment-1)));
    }

    /* 计算缓存中的空余空间大小 */
    free_sz = (unsigned int)pMemBuf->end - (unsigned int)buf_res;

    /*空间不够，返回空指针*/
    if( free_sz < nSize )
    {
		DBG_ERR("free_sz = %d, nSize = %d\n", free_sz, nSize);
        buf_res = NULL;
    }
    else
    {
        /* 清空分配内存 */
        //memset(buf_res, 0, nSize);

        /* 更新空余指针位置 */
        pMemBuf->cur_pos = (void*)((unsigned int)buf_res + nSize);
    }

    //DBG_MSG("alloc buf addr=%p size=%d\n", buf_res, nSize);
    return buf_res;
}

void * alloc_mem_tab(VCA_MEM_TAB_V2 *mem_tab, void *vca_buf)
{
	int i;

	for (i = 0; i < VCA_MEM_TAB_NUM; i++)
	{
		if (mem_tab[i].size > 0)
		{
			mem_tab[i].base = alloc_buffer((VCA_MEM_BUF *)vca_buf, mem_tab[i].size, mem_tab[i].alignment);
			CHECK_ERROR(mem_tab[i].base == NULL, NULL);
		}
		else
		{
			mem_tab[i].base = NULL;
		}
		DBG_MSG("tab %d memsize: %f M\r\n", i, mem_tab[i].size / 1024.0 / 1024.0);
	}

	return 0;
}

INT32 HAT_AD_ipcInit(void)
{
	INT32 iRet = 0;
	unsigned long long start, end;
	ALG_IPC_QUE_CTRL *pAlgIpcCtrl = NULL;
	int i = DSP_CORE;

	pAlgIpcCtrl = &(pDspShareMem->algShareInfo[i].algInitParam.algResultIpc);

	end = start = mss_get_clock();
	while(!(pAlgIpcCtrl->pIpcMemPhy))
	{
		if ((((end - start)/600) % 1000000) == 0)
		{
			/*ALG_PROCESS_PRT("1.[algResultIpc] phy addr 0x%x, memsize %d, depth %d\r\n",
				pAlgIpcCtrl->pIpcMemPhy, pAlgIpcCtrl->memSize, pAlgIpcCtrl->ipcDepth);*/
		}
		
		end = mss_get_clock();
	}
	
	/* 初始化队列*/
	iRet = HAT_AD_IpcChannelCreate((void *)(pAlgIpcCtrl->pIpcMemPhy), pAlgIpcCtrl->memSize,
							 pAlgIpcCtrl->ipcDepth, NULL, (IpcPortT *)&(pAlgIpcCtrl->pIpcHandle));
	if(iRet != 0)
	{
		ALG_PROCESS_ERR("create ipc handle error.%d\r\n",iRet);
		return -1;
	}
	return 0;
}

INT32 HAT_AD_algoInit(HAT_AD_INIT_PARAM *alg_init_mem)
{
	int iRet = OK;

	if(NULL == alg_init_mem)
	{
		ALG_PROCESS_ERR("HAT_AD_BSD_init error.[0x%x]\r\n", iRet);
		return -3;
	}

	ALG_PROCESS_DEBUG("algTypeMask:0x%x\n", alg_init_mem->algParam.algTypeMask);
	/*初始化DBA*/
#ifdef VCA_DBA
	if(alg_init_mem->algParam.algTypeMask & (1 << SMART_MODE_DBA))
	{
		iRet = HAT_AD_dba_init(alg_init_mem);
		if(iRet != 0)
		{
			ALG_PROCESS_ERR("HAT_AD_dba_init error.[0x%x]\r\n", iRet);
			return -3;
		}
	}
#endif

	/*初始化BSD*/
#ifdef VCA_BSD
	if(alg_init_mem->algParam.algTypeMask & (1 << SMART_MODE_BSD_R))
	{
		iRet = HAT_AD_BSD_init(alg_init_mem);
		if(iRet != 0)
		{
			ALG_PROCESS_ERR("HAT_AD_BSD_init error.[0x%x]\r\n", iRet);
			return -3;
		}
	}
#endif
	/*初始化FCW*/
#ifdef VCA_FCW
	if(alg_init_mem->algParam.algTypeMask & (1 << SMART_MODE_FCW))
		{
			iRet = HAT_AD_fcw_init(&alg_init_mem->algParam.algInfo[SMART_MODE_FCW]);
			if(iRet != OK)
			{
				ALG_PROCESS_ERR("HAT_AD_fcw_init error.[0x%x]\r\n", iRet);
				return -3;
			}
		}
#endif

	/* 设计参考结构体HAT_AD_INIT_PARAM */
    return iRet;
	
}



/*******************************************************************************

   Function:		HAT_AD_fusionInit

   Description:     初始化算法处理模块

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
INT32 HAT_AD_fusionInit(HAT_AD_INIT_PARAM *alg_init_mem)
{
	int iRet = 0;

	iRet = HAT_AD_ipcInit();
	if(iRet != 0)
	{
		ALG_PROCESS_ERR("HAT_AD_ipcInit error.[0x%x]\r\n", iRet);
		return iRet;
	}

	iRet = HAT_AD_algoInit(alg_init_mem);
	if(iRet != 0)
	{
		ALG_PROCESS_ERR("HAT_AD_algoInit error.[0x%x]\r\n", iRet);
		return iRet;
	}

	return iRet;
}
/*******************************************************************************

   Function:		HAT_AD_algorithmSleepMs

   Description:     延时函数

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
void HAT_AD_algorithmSleepMs(int sleepMs)
{
	unsigned long long start, end;
	
	do{
		end = start = mss_get_clock();
		while(1)
		{
			end = mss_get_clock();

			if (((end - start)/600) >= 1000)
			{
				break;
			}
		}
	}while(sleepMs--);
}

/*******************************************************************************

   Function:		HAT_AD_algorithmSetChn

   Description:     算法处理执行函数

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
INT32 HAT_AD_algorithmSetChn(UINT32 mask)
{
	UINT32 i = 0;

	memset(&gAlgCtrl, 0, sizeof(HAT_AD_ALG_CTRL));

	/*ALG_PROCESS_PRT("recive ipc mask 0x%x.\r\n",mask);*/

	if(0 == mask)
	{
		return ERROR;
	}

	for(i=0; i<MAX_YUV_FRAME; i++)
	{
		if((mask>>i) & 1)
		{
			gAlgCtrl.algVideoCnt ++;
			gAlgCtrl.algVideoChn[i] = i;
		}
	}

	/*ALG_PROCESS_PRT("alg video frame cnt %d.\r\n",gAlgCtrl.algVideoCnt);*/
	
	return OK;
	
}

/*******************************************************************************

   Function:		HAT_AD_algorithmGetChn

   Description:     算法处理FRAME通道号

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
INT32 HAT_AD_algorithmGetChn(UINT32 idx)
{	
	return gAlgCtrl.algVideoChn[idx];
}
/*******************************************************************************

   Function:		HAT_AD_algorithmGetNum

   Description:     获取算法FRAME通道数

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
INT32 HAT_AD_algorithmGetNum(void)
{	
	return gAlgCtrl.algVideoCnt;
}

/*******************************************************************************

   Function:		HAT_AD_algorithmSetMode

   Description:     设置算法处理模式

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
UINT32 HAT_AD_algorithmSetMode(HAT_AD_SMART_MODE mode)
{
	gAlgCtrl.algMode = mode;
	return OK;
}

/*******************************************************************************

   Function:		HAT_AD_algorithmGetMode

   Description:     获取设置算法处理模式

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
UINT32 HAT_AD_algorithmGetMode(void)
{
	return (UINT32)gAlgCtrl.algMode;
}


/*******************************************************************************

   Function:		HAT_AD_algorithmProc

   Description:     算法处理执行函数

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
INT32 HAT_AD_algorithmProc(UINT32 idx,
		                   FUSION_SHARE_DATA *alg_input,
                           HAT_AD_ALG_RESULT *alg_output,
                           HAT_AD_ALG_STATUS *alg_run_status)
{
	INT32 ret = 0;
	UINT32 imgChn = 0;
	HAT_AD_ALG_STATUS *pAlgRunStatus = NULL;

	pAlgRunStatus = &pDspShareMem->algShareInfo[DSP_CORE].algRunStatus;

	if(NULL == alg_input || NULL == alg_output || NULL == alg_run_status)
	{
		ALG_PROCESS_ERR("input param error.\r\n");
		return ERROR;
	}

	/* 获取视频帧通道 */
	imgChn = HAT_AD_algorithmGetChn(idx);

	if(NULL == alg_input->videoFrame[imgChn].pVideoDesc)
	{
        ALG_PROCESS_ERR("data addr 0x%x, video mask 0x%x.\r\n",alg_input,alg_input->videoMask);
		ALG_PROCESS_ERR("chan %d, idx %d, input param[pVideoDesc] error.\r\n",imgChn,idx);
		return ERROR;
	}

	/* 设置当前视频帧的算法模式 */
	HAT_AD_algorithmSetMode(alg_input->videoFrame[imgChn].smartMode);

	/*ALG_PROCESS_PRT("imgChn %d, smartMode:0x%x.\r\n", imgChn, alg_input->videoFrame[imgChn].smartMode);*/

	switch(alg_input->videoFrame[imgChn].smartMode)
	{
		case SMART_MODE_DBA:
#ifdef VCA_DBA
			ret = HAT_AD_ALG_dba_process(alg_input, alg_output, alg_run_status, imgChn);
			if(ret != OK)
			{
				ALG_PROCESS_ERR("HAT_AD_ALG_dba_process ERR,[0x%x]", ret);
			}
#endif
			break;
		case SMART_MODE_BSD_R:
#ifdef VCA_BSD
			ret = HAT_AD_ALG_bsd_process(alg_input, alg_output, alg_run_status, imgChn);
			if(ret != OK)
			{
				ALG_PROCESS_ERR("HAT_AD_ALG_dba_process ERR,[0x%x]", ret);
			}
#endif
			break;
		case SMART_MODE_MOD:
			/*MOD*/
			break;
		case SMART_MODE_FCW:
			/*FCW*/
#ifdef VCA_FCW
			ret = HAT_AD_ALG_fcw_process(alg_input, alg_output, alg_run_status, imgChn);
			if(ret != OK)
			{
				ALG_PROCESS_ERR("HAT_AD_ALG_fcw_process ERR,[0x%x]\n", ret);
			}
#endif	
			break;
		default:
			ALG_PROCESS_ERR("error idx[%d] smart(vca) type %d.\r\n",imgChn,alg_input->videoFrame[imgChn].smartMode);
			break;
	}
	
	HAT_AD_delay_statistic(&pAlgRunStatus->frameTimePot.preDelay, alg_input->videoFrame[imgChn].pVideoDesc->video_header.tagProc);
	alg_input->videoFrame[imgChn].pVideoDesc->video_header.tagProc = mss_get_clock();
    return OK;
}

/*******************************************************************************

   Function:		HAT_AD_algorithmPrintStatus

   Description:     打印当前算法处理模块的状态信息

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
INT32 HAT_AD_algorithmPrintStatus(void)
{
	/*DBA*/

	/*MOD*/

	/*FCW*/

	/*BSD*/

    return 0;
}

void qman_open_complement(void)
{
	UINT32 uireg;

	// 0x728
	uireg = (XM4_REG_ADD0_ATT1_DABSZ << 9) | (XM4_REG_ADD0_ATT1_DADOL << 13) | (XM4_REG_ADD0_ATT1_DAUOL << 18);
	reg_write_int(XM4_REG_ADD0_ATT1, uireg);

	// 0x730
	reg_write_int(XM4_REG_ADD1_START, XM4_REG_ADD1_START_VAL);

	// 0x624: set BSZ to 0x0
	uireg = reg_read_int(XM4_REG_MSS_DDTC);
	uireg = (uireg & 0xe1000000) >> 25;
	if(uireg==0xf)
	{
		ALG_PROCESS_PRT("Warning! BSZ in MSS_DDTC is 0xF!\n\r");
	}
}

/*******************************************************************************

   Function:		HAT_AD_tskAlgProcess

   Description:

   Input:

   Output:

   Return:
    0:			Successful
    ohters:		Failed

*******************************************************************************/
void HAT_AD_tskAlgProcess(void)
{
	INT32 iRet = 0;
	UINT32 i = 0;
	UINT32 wIdx = 0;
	UINT32 frameCnt = 0;
	UINT32 dspIdx = DSP_CORE;
	UINT32 tickTime = 0;
	UINT32 totalTime = 0;
	volatile UINT32 start = 0;
	volatile UINT32 end = 0;
	IpcMsgT ipcMsg = {0}; 
	IpcPortT pReadFusionPort = NULL;
	ALG_IPC_QUE_CTRL *pAlgIpcCtrl  = NULL;
	ALG_DATA_DESC  *pAlgDataQue = NULL;
	ALG_DATA_DESC  *pAlgResultQue = NULL;
	FUSION_SHARE_DATA *pFusionData = NULL;
	HAT_AD_ALG_RESULT *pAlgCevaResult = NULL;
	HAT_AD_ALG_STATUS *pAlgRunStatus = NULL;
	HAT_AD_TASK_STATUS *pAlgTaskStatus = NULL;
	FUSION_SHARE_DATA *pAlgOutput = NULL;
	UINT32 imgChn = 0;
	
	pAlgIpcCtrl  = &(pDspShareMem->algShareInfo[dspIdx].algInitParam.algFusionIpc);
		
	ALG_PROCESS_PRT("[algFusionIpc] addr 0x%x handle 0x%x memsize %d depth %d\r\n",\
			pAlgIpcCtrl->pIpcMemPhy,pAlgIpcCtrl->pIpcHandle,pAlgIpcCtrl->memSize,pAlgIpcCtrl->ipcDepth);
	
	iRet = HAT_AD_IpcInitReadPort(pAlgIpcCtrl->pIpcMemPhy, &pReadFusionPort);
	if(OK != iRet)
	{
		ALG_PROCESS_ERR("ipc init read port error.\r\n");
		return;
	}
	
	ALG_PROCESS_PRT("[algFusionIpc] addr 0x%x pReadFusionPort 0x%x.\r\n",pAlgIpcCtrl->pIpcMemPhy,pReadFusionPort);

	pAlgRunStatus = (HAT_AD_ALG_STATUS *)HAT_AD_getAlgStatusHandle(dspIdx);
	pAlgTaskStatus = &(pAlgRunStatus->taskStatus);

	qman_open(4);
	qman_open_complement();

    while(1)
    {
		/**************************读取融合数据*************************/
		start = mss_get_clock();
		memset(&ipcMsg, 0, sizeof(IpcMsgT));
		iRet = HAT_AD_IpcRecv(pReadFusionPort, &ipcMsg);
		if(OK != iRet)
		{
			/*ALG_PROCESS_ERR("ipc receive fusion data error.\r\n");*/
			HAT_AD_setAlgReceiFail(pAlgRunStatus);
			HAT_AD_setAlgTaskDebug(0, pAlgTaskStatus);
			continue;
		}
		
		HAT_AD_setAlgReceiSucc(pAlgRunStatus);

		pAlgDataQue = (ALG_DATA_DESC*)(ipcMsg.data);
		if(NULL == pAlgDataQue)
		{
			ALG_PROCESS_ERR("error, pAlgDataQue is null.\r\n");
			HAT_AD_setAlgTaskDebug(1, pAlgTaskStatus);
			continue;
		}

		/*ALG_PROCESS_ERR("pAlgDataQue 0x%x, pFusionData 0x%x.\r\n",pAlgDataQue,pAlgDataQue->pBufAddr);*/

		/* 判断队列状态,解析状态失败不会通知UITRON */
		if(pAlgDataQue->bufStatus == HAT_AD_DATA_READY)
		{
			pAlgDataQue->bufStatus = HAT_AD_DATA_INUSE;
		}
		else
		{
			ALG_PROCESS_ERR("warning, AlgDataQue buf status[%d] is not ready.\r\n",pAlgDataQue->bufStatus);
			ALG_PROCESS_ERR("warning, addr: pAlgDataQue 0x%x, pFusionData 0x%x.\r\n",pAlgDataQue,pAlgDataQue->pBufAddr);
			HAT_AD_setAlgTaskDebug(2, pAlgTaskStatus);
			continue;
		}
		/****************************************************************/
		
		pFusionData = (FUSION_SHARE_DATA *)pAlgDataQue->pBufAddr;

		/* 解析融合数据帧,解析状态失败不会通知UITRON */
		iRet = HAT_AD_algorithmSetChn(pFusionData->videoMask);
		if(OK == iRet)
		{
			frameCnt = HAT_AD_algorithmGetNum();
		}
		
		HAT_AD_setAlgProcFrame(pAlgRunStatus);
		HAT_AD_setAlgTaskDebug(3, pAlgTaskStatus);
		
	    for(i=0; i<frameCnt; i++)
		{
			/* 判断当前队列的数据是否已经被使用完 */
			do
			{
				pAlgResultQue = &(pDspShareMem->algShareInfo[dspIdx].algResultDesc[wIdx]);
				pAlgCevaResult = (HAT_AD_ALG_RESULT *)(pAlgResultQue->pBufAddr);
				HAT_AD_setAlgTaskDebug(4, pAlgTaskStatus);
			}
			while (pAlgResultQue->bufStatus == HAT_AD_DATA_READY || pAlgResultQue->bufStatus == HAT_AD_DATA_INUSE);	
			
			imgChn = HAT_AD_algorithmGetChn(i);
			pFusionData->videoFrame[imgChn].pVideoDesc->video_header.tagProc = mss_get_clock();

			/***************************调用算法*****************************/
            HAT_AD_setAlgTaskDebug(5, pAlgTaskStatus);
			iRet = HAT_AD_algorithmProc(i, pFusionData, pAlgCevaResult, pAlgRunStatus);
			if(OK != iRet)
			{
				ALG_PROCESS_ERR("HAT_AD_algorithmProc error.\r\n");
			}
            HAT_AD_setAlgTaskDebug(6, pAlgTaskStatus);
			/****************************************************************/


			/*************************发送算法结果数据**********************/
			pAlgResultQue->bufStatus     = HAT_AD_DATA_READY;
			pAlgResultQue->bufSize       = sizeof(HAT_AD_ALG_RESULT);
			if(pAlgCevaResult)
			{                
				pAlgCevaResult->pAlgDataDesc = pAlgDataQue;
			}
			
			/* 算法类型 */
			memset(&ipcMsg, 0, sizeof(IpcMsgT));
			ipcMsg.type    = HAT_AD_algorithmGetMode();
			ipcMsg.datalen = sizeof(ALG_DATA_DESC);
			ipcMsg.data    = pAlgResultQue;
            ipc_flushCache((UINT32)pAlgResultQue, sizeof(ALG_DATA_DESC));

			
			/* 发送算法结果数据 */
			do
			{
				pAlgIpcCtrl = &(pDspShareMem->algShareInfo[dspIdx].algInitParam.algResultIpc);
				HAT_AD_setAlgTaskDebug(7, pAlgTaskStatus);
			}
			while (HAT_AD_IpcSend((IpcPortT)(pAlgIpcCtrl->pIpcHandle), &ipcMsg) != OK);

			HAT_AD_delay_statistic(&pAlgRunStatus->frameTimePot.cevaIpcDelay,gTimePost);

			HAT_AD_setAlgSendSucc(pAlgRunStatus);
			HAT_AD_setAlgTaskDebug(8, pAlgTaskStatus);

			//printf("ipc send algorithm result data ok.i:%d\r\n", i);
			/****************************************************************/
			
			GO_FORWARD(wIdx, HAT_AD_DATA_MAX_NUM);
	   }
		
	   end = mss_get_clock();

	   tickTime = (end - start) / 600000;
	   HAT_AD_setAlgTaskTime(tickTime, pAlgTaskStatus);
	   HAT_AD_setAlgTaskCurStatus(__LINE__, tickTime, pAlgTaskStatus);
	   
	   HAT_AD_setAlgTaskCnt(pAlgTaskStatus);
	   totalTime = totalTime + tickTime;
	   HAT_AD_setAlgTaskAvgTime(totalTime, pAlgTaskStatus);
	   
	   HAT_AD_setAlgTaskDebug(9, pAlgTaskStatus);
	}

	qman_close();
	vTaskDelete( NULL );
}

