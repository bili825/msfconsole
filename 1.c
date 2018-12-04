/**
 * @file:   debug_ctl.c
 * @note:   2010-2020, ���ݺ����������ּ����ɷ����޹�˾
 * @brief   ʵ�ֵ��Դ�ӡ�ӿ�,�ʹ�������getDebug,setDebug,debugLog
 * @author: zhoufeng16
 * @date    2018/7/16
 * @note:
 * @note \n History:
 *  1.��    ��: 2018/7/16
 *    ��    ��: zhoufeng16
 *    �޸���ʷ: �����ļ�
 */

/*----------------------------------------------*/
/*                 ����ͷ�ļ�                   */
/*----------------------------------------------*/
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include "sys/sys_type.h"
#include "sys/sys_posix.h"
#include "sys/sys_mem.h"
#include "sys/sys_time.h"
#include "sys/error.h"
#include "sys/utils.h"
#include "dblog/debug_ctl.h"
#include "dblog/log_ctl.h"

/*----------------------------------------------*/
/*                 �����Ͷ���                   */
/*----------------------------------------------*/

/* ��ӡ����Ĭ�ϲ��� */
#define DEFAULT_DEBUG_LEVEL RT_ERROR    /* /< �豸�����ÿ��ģ���Ĭ�ϴ�ӡ�ȼ�,'���д���' */

/* ��ӡ��Ϣ��������С��صĺ� */
#define MAX_DBGMSG_LEN	256 /* /< ÿ����ӡ��Ϣ����󳤶� */

/* ��ӡ��Ϣ�ĸ�����Ϣ������صĺ� */
#define ISFILENAME_SET_BIT	0x04 /* /< ��2λ���ó�1,��Ҫ��ʾ�ļ��� */
#define ISFUNCNAME_SET_BIT	0x02 /* /< ��1λ���ó�1,��Ҫ��ʾ������ */
#define ISLINE_SET_BIT		0x01 /* /< ��0λ���ó�1,��Ҫ��ʾ�к� */

#define ALL_MOD_NAME "ALL"      /* /< ��������ģ��ʱ,�����ģ���� */

#define DB_LOG_VISION "V1.00"
#define CUSTOM_MODLUS_SUM 24
/* #define RECORD_DEBUG(level, arg...)  dev_debug(level, MOD_SYSFUN_RECORD, ##arg) */
#define ALL_MOD_NUMS (PREINSTALL_ALL_MOD_NUMS+CUSTOM_MODLUS_SUM)
#define UNUSE		0
#define USED		-2
/*----------------------------------------------*/
/*                �ṹ�嶨��                    */
/*----------------------------------------------*/


/*----------------------------------------------*/
/*                 ��������                     */
/*----------------------------------------------*/
static inline BOOL isprint_bylevel(UINT32 debug_level, UINT32 mod_curdebug_level);
static inline MOD_ATTRIBUTE_INFO *get_allmod_attr_info(void);
static inline DEBUG_LEVEL_ATTRIBUTE *get_all_level_attr_info(void);

/*----------------------------------------------*/
/*                 ȫ�ֱ���                     */
/*----------------------------------------------*/
FUNC_PT_LIST g_debuglogcom[MAX_COM_FUNC] =
{
	(GenFuncPt)debug_log_add_storage         ,	"debug_log_add_storage"     ,
	(GenFuncPt)debug_log_del_storage         ,	"debug_log_del_storage"     ,
	(GenFuncPt)debug_test_init               ,	"debug_test_init"           ,
	(GenFuncPt)get_db_log_vision             ,	"get_db_log_vision"         ,
	(GenFuncPt)get_debug_para_print          ,	"get_debug_para_print"      ,
	(GenFuncPt)init_debug_log_modlue         ,	"init_debug_log_modlue"     ,
	(GenFuncPt)print_debug                   ,	"print_debug"               ,
	(GenFuncPt)set_debug_para                ,	"set_debug_para"            ,
	(GenFuncPt)set_level_cont                ,	"set_level_cont"            ,
	(GenFuncPt)uninit_debug_log_modlue       ,	"uninit_debug_log_modlue"   ,
};



/* ���е��Եȼ��������� */
static DEBUG_LEVEL_ATTRIBUTE g_all_debuglevel_attr[ALL_DBG_LEVEL_NUMS] =
{
    {NON,            "NON",          "NON",      DEBUG_COLOR_WHITE},
    {SYS_ERROR, 	 "SYS_ERROR",    "ERROR",    DEBUG_COLOR_RED},
    {RT_ERROR,       "RT_ERROR",     "ERROR",    DEBUG_COLOR_RED},
    {KEY_WARN,       "KEY_WARN",     "STATE",    DEBUG_COLOR_GREEN},
    {NORMAL_WARN,    "NORMAL_WARN",  "WARNING",  DEBUG_COLOR_YELLOW},
    {DEBUG_NOTICE,   "DEBUG_NOTICE", "DEBUG",    DEBUG_COLOR_WHITE},
    {DEBUG_INFO,     "DEBUG_INFO",   "DEBUG",    DEBUG_COLOR_WHITE},
    {DEBUG_ALL_DAV,	 "DEBUG_ALL_DAV","DEBUG",    DEBUG_COLOR_WHITE},
};

/* ����ÿ��ģ��������� */
static MOD_ATTRIBUTE_INFO g_allmod_attr_info[ALL_MOD_NUMS] =
{
    /* ϵͳ������ */
    {MOD_SYSFUN_SYSINIT,     "SYSINIT",     DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_UPGRADE,     "UPGRADE",     DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_POWERON,     "POWERON",     DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_RECORD,      "RECORD",      DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_STORE,       "STORE",       DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_MP4_PACK,    "MP4_PACK",    DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_PLAYBACK,    "PLAYBACK",    KEY_WARN,        DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_SYSLOG,      "SYSLOG",      DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_ALARM_EXP,   "ALARM_EXP",   DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_CAPA,        "CAPA",        DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_SM,          "SM",          KEY_WARN,        DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_CMD,         "CMD",         DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_RPC,         "RPC",         DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_UTIL,        "UTIL",        DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_OPERA,       "OPERA",       DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_DEV_INFO,    "DEV_INFO",    DEBUG_NOTICE,    DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_SYSFUN_FILE,        "FILE_NOTIFY", DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},

    /* �������� */
    {MOD_CAN_INTERFACE,      "CAN",         DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_WIFI_INTERFACE,     "WIFI",        DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_VEH_INTERFACE,      "VEH",         DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_DSP_INTERFACE,      "DSP",         DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_UART_INTERFACE,     "UART",        DEBUG_NOTICE,    DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_PHONE_INTERFACE,    "PHONE",       DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},

    /* Ӳ���� */
    {MOD_HW_MCU,            "MCU",          DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_BUTTON,         "BUTTON",       KEY_WARN,        DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_LED,            "LED",          DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_SENSOR,         "SENSOR",       DEBUG_NOTICE,    DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_GPS,            "GPS",          KEY_WARN,        DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_SD,             "SD",           DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},

    /* T 1���� */
    {MOD_T1_TEST,           "T1_TEST",      DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_SYSFUN_PS_PACK,    "PS_TEST",      DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},

    /* ����ϵͳ�� */
    {MOD_OTHER,             "OTHER",        DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_OTHER_CLASS_ID,  NON},

};

/* ʹ��Ĭ�ϵ����� */
DB_LOG_CONFIG g_debugLogConfig =
{
    CTL_COM_LOG, TRUE, TRUE
};

/*----------------------------------------------*/
/*                 ��������                     */
/*----------------------------------------------*/

/**
 * @function:   isprint_bylevel
 * @brief:      �ж�����ȼ��Ĵ�ӡ��Ϣ�Ƿ�Ҫ���
 * @param[in]   UINT32 debug_level
 * @param[in]   UINT32 mod_curdebug_level
 * @param[out]: �Ƿ�Ҫ���
 * @return      static inline BOOL
 */
static inline BOOL isprint_bylevel(UINT32 debug_level, UINT32 mod_curdebug_level)
{
    BOOL b_print_out = FALSE; /* �Ƿ�Ҫ���,Ĭ�ϲ�Ҫ��� */

    /* ��������ӡ��Ϣ�ĵȼ�ֵС�ڵ������õ�ֵ���Ҳ�����NON �����,��Ϊ�ȼ�ֵԽС���ȼ�Խ�� */
    if ((debug_level <= mod_curdebug_level) && (debug_level != NON))
    {
        b_print_out = TRUE;
    }

    return b_print_out;

}

/******************************����ӿ�*****************************************/

/**
 * @function:   print_debug_modlue_str
 * @brief:      ��֯��ӡ���뻺���У�ʹ��ģ������������
 * @param[in]:  UINT32 debug_level     ���Եȼ� 
 * @param[in]:  char *ModlueName      ����ģ���ַ�������
 * @param[in]:  const char *filename  �����ļ�
 * @param[in]:  UINT32 line           �������ļ��е��к�
 * @param[in]:  const char *funname   ������
 * @param[in]:  const char *format    �ɱ��������
 * @param[in]:  ...                   �ɱ���� 
 * @param[out]: None
 * @return:     void
 */
void print_debug_modlue_str(UINT32 debug_level, char *ModlueName, const char *filename, UINT32 line, const char *funname, const char *format, ...)
{
    int i = 0;
    va_list ap; /* �ɱ�����ṹ�� */
    char sz_degmsg[MAX_DBGMSG_LEN]; /* ��ӡ��Ϣ���� */
    for(i=0; i<ALL_MOD_NUMS ;i++)
    {
        if (strncmp(g_allmod_attr_info[i].mod_name,ModlueName,MAX_MOD_NAME) == 0)
        {
            break;
        }
    }
    if (i == ALL_MOD_NUMS)
    {
        /*���ģ�����Ʋ����ڣ����Լ�Other���д�ӡ*/
        sys_printf("[dblog]error print_debug no modlue %s\n",ModlueName);
        i = MOD_OTHER;
    }
    va_start(ap, format);
    vsnprintf(sz_degmsg,sizeof(sz_degmsg)-1,format,ap);
    va_end(ap);
    print_debug(debug_level,i,filename,line,funname,sz_degmsg);

}

/** @fn	void print_debug(UINT32 debug_level, UINT32 debug_mod_idx, const INT8 *filename, UINT32 line, const INT8 *funname, const INT8 *format, ...)
 *  @brief	��֯��ӡ��ϢȻ����뻺������
 *  @param[in]  debug_level ���Եȼ�
 *  @param[in]  debug_mod_idx ����ģ������
 *  @param[in]  filename �����ļ�
 *  @param[in]  line �������ļ��е��к�
 *  @param[in]  funname ������
 *  @param[in]  format �ɱ��������
 *  @param[in]  ... �ɱ����
 *  @param[out] ��ӡ��Ϣ
 *  @return	  ��
 */
void print_debug(UINT32 debug_level, UINT32 debug_mod_idx, const char *filename, UINT32 line, const char *funname, const char *format, ...)
{
    UINT32 cur_debug_mod_idx = 0; /* ������ӡ��Ϣ��ģ������ */
    UINT32 cur_debug_content = 0; /* ĳ��ģ�����õĸ��ӿ�����Ϣ */
    UINT32 cur_debug_level = 0; /* ĳ��ģ�����õĵ��Եȼ� */
    UINT32 cur_debug_log_level = 0; /* ĳ��ģ�����õ���־�ȼ� */

    char sz_degmsg[MAX_DBGMSG_LEN]; /* ��ӡ��Ϣ���� */
    char *plevelname = NULL; /* ���Եȼ����� */
    char *pmod_name = NULL; /* ָ��ģ���ģ���� */

    SYS_DATE_TIME_T sysTime;

    va_list ap; /* �ɱ�����ṹ�� */
    INT32 msglen = 0; /* sz_degmsg�������ַ����ĳ��� */
    char sz_msg_tmp[MAX_DBGMSG_LEN]; /* ������ʱ����ַ��� */
    INT32 tmp_msg_len = 0; /* ��ʱ�ַ����������ַ����ĳ��� */
    MOD_ATTRIBUTE_INFO *pallmod_attr_info = NULL; /* ����ָ��ģ��������Ϣ�ṹ������ */
    DEBUG_LEVEL_ATTRIBUTE *pall_level_attr_info = NULL; /* ����ָ����Եȼ��������� */
    int ret = 0;

    if ((NULL == filename) || (NULL == funname) || (NULL == format))
    {
        return;
    }

    memset(sz_degmsg, 0, MAX_DBGMSG_LEN);
    memset(sz_msg_tmp, 0, MAX_DBGMSG_LEN);

    /* ��ȡ����ģ����������Ϣ�ṹ�������׵�ַ */
    pallmod_attr_info = get_allmod_attr_info();
    if (NULL == pallmod_attr_info)
    {
        sys_printf("[dblog]error g_allmod_attr_info have no space,filename %s,line %u\n", __FILENAME__, __LINE__);
        return;
    }

    /* ��ȡ���Եȼ�����������׵�ַ */
    pall_level_attr_info = get_all_level_attr_info();
    if (NULL == pall_level_attr_info)
    {
        sys_printf("[dblog]error g_all_debuglevel_attr have no space,filename %s,line %u\n", __FILENAME__, __LINE__);
        return;
    }

    /* ��debug_level������Χ,��ȼ���ΪĬ��ֵ,�����������Ǹ�ֵ */
    if (debug_level >= ALL_DBG_LEVEL_NUMS)
    {
        debug_level = DEFAULT_DEBUG_LEVEL;
    }

    /* ��ȡ������ӡ����ģ��ĵ��Կ�����Ϣ,���Եȼ��Լ�������Ϣ */
    if (debug_mod_idx >= ALL_MOD_NUMS)
    {
        /* ��ģ������������Χ,Ĭ��ģ��������MOD_OTHER,����Ĭ����Ϣ��0 */
        cur_debug_mod_idx = MOD_OTHER;
        cur_debug_content = 0;
        cur_debug_level = DEFAULT_DEBUG_LEVEL;
        cur_debug_log_level = DEFAULT_DEBUG_LEVEL;
    }
    else
    {
        /* debug_mod_idx��pallmod_attr_info[debug_mod_idx].mod_idx��һ���� */
        cur_debug_mod_idx = debug_mod_idx;
        cur_debug_content = pallmod_attr_info[debug_mod_idx].content;
        cur_debug_level = pallmod_attr_info[debug_mod_idx].level;
        cur_debug_log_level = pallmod_attr_info[debug_mod_idx].log_level;
    }

    /* ��Ҫ���������Ϣ, ����Ҫ����ϴ�ӡ��Ϣ */

    /* ��ȡ������ӡ��Ϣ�ĵ��Եȼ����� */
    plevelname = pall_level_attr_info[debug_level].level_name_show;

    /* ��ȡģ������ */
    pmod_name = pallmod_attr_info[cur_debug_mod_idx].mod_name;

    /* ��ȡϵͳʱ�䣬Ȼ���ʼ�� */
    ret = sys_datetime_get(&sysTime);

    /* ������ӡ��Ϣ�ĸ�ʽ[time][modname][level][file][line][function][contents] */

    /* Ĭ��û�и�����Ϣ,��д[time][modname][level].�����ǽ���ӡ��Ϣ����ʽ������sz_degmsg��,�����Ƚ�ÿ��
     * ��ӡ��Ϣ��ĳЩ��snprintf����ʱ����sz_msg_tmp��,��sz_msg_tmp���ַ����ĳ��� + sz_degmsg��ԭ���ַ���
     * �ĳ��� < ÿ����ӡ��Ϣ����󳤶�,��׷�ӵ�����sz_degmsg�� */



    /* �����߳�ID��ʱ�䡢��ӡ�ȼ����Լ��������� */
    tmp_msg_len = snprintf(sz_degmsg, sizeof(sz_degmsg), "[%08x][%02d %02d:%02d:%02d][%s][%s]",
                           (int)sys_pthread_self(), sysTime.uMonth, sysTime.uHour, sysTime.uMinute, sysTime.uSec,
                           pmod_name, plevelname);

    if ((tmp_msg_len > 0) && ((msglen + tmp_msg_len) < MAX_DBGMSG_LEN))
    {
        /* ��Ҫ׷�ӵĳ��ȼ���ԭ�г��Ȳ�Խ��,��׷�ӡ�����׷��'�ļ���'��'�к�'��'������'ʱ���ƴ��� */
        strcat(&sz_degmsg[msglen], sz_msg_tmp);
        msglen += tmp_msg_len;
    }

    /* ��Ҫ�����ļ��� */
    if ((cur_debug_content & ISFILENAME_SET_BIT) > 0)
    {
        tmp_msg_len = snprintf(sz_msg_tmp, MAX_DBGMSG_LEN, "[%s]", filename);
        if ((tmp_msg_len > 0) && ((msglen + tmp_msg_len) < MAX_DBGMSG_LEN))
        {
            strcat(&sz_degmsg[msglen], sz_msg_tmp);
            msglen += tmp_msg_len;
        }
    }

    /* ��Ҫ�����к� */
    if ((cur_debug_content & ISLINE_SET_BIT) > 0)
    {
        tmp_msg_len = snprintf(sz_msg_tmp, MAX_DBGMSG_LEN, "[%u]", line);
        if ((tmp_msg_len > 0) && ((msglen + tmp_msg_len) < MAX_DBGMSG_LEN))
        {
            strcat(&sz_degmsg[msglen], sz_msg_tmp);
            msglen += tmp_msg_len;
        }
    }

    /* ��Ҫ���������� */
    if ((cur_debug_content & ISFUNCNAME_SET_BIT) > 0)
    {
        tmp_msg_len = snprintf(sz_msg_tmp, MAX_DBGMSG_LEN, "[%s]", funname);
        if ((tmp_msg_len > 0) && ((msglen + tmp_msg_len) < MAX_DBGMSG_LEN))
        {
            strcat(&sz_degmsg[msglen], sz_msg_tmp);
            msglen += tmp_msg_len;
        }
    }

    /* ������Ӵ�ӡ���� */
    va_start(ap, format);
    ret = vsnprintf(&sz_degmsg[msglen], MAX_DBGMSG_LEN - msglen, format, ap);
    va_end(ap);
    msglen += ret;

    sz_degmsg[MAX_DBGMSG_LEN - 1] = '\0';

    
    if (g_debugLogConfig.disAddr & CTL_COM)
    {
        if (isprint_bylevel(debug_level, cur_debug_log_level))
        {
            log_save_file(sz_degmsg);
        }
    }

    if (g_debugLogConfig.disAddr & CTL_LOG)
    {
        if (g_debugLogConfig.delNewLineEnable)
        {

        }

        /* ���Ҫ������ɫ */
        if (g_debugLogConfig.colorEnable)
        {
            tmp_msg_len = snprintf(sz_msg_tmp, MAX_DBGMSG_LEN, "%s%s%s", g_all_debuglevel_attr[debug_level].leve_color, sz_degmsg, DEBUG_COLOR_CLOSE);
            if (tmp_msg_len < MAX_DBGMSG_LEN)
            {
                strncpy(sz_degmsg, sz_msg_tmp, sizeof(sz_degmsg) - 1);
            }
        }

        /* ����ӡ��ϢҪ��������� */
        if (isprint_bylevel(debug_level, cur_debug_level))
        {
            /* �ж������λ�� */
            sys_printf("%s", sz_degmsg);
        }
    }
    return;
}


/**
 * @function:   init_debug_log_modlue
 * @brief:      ��ʼ����ӡ��־��ģ��
 * @param[in]:  UINT32 disAddr  
 * @param[in]:  char *logPath   
 * @param[out]: None
 * @return:     int
 */
int init_debug_log_modlue(UINT32 disAddr, char *logPath)
{
    int ret = 0;
    g_debugLogConfig.disAddr = disAddr;
    ret = log_module_init(logPath);
    return ret;
}

/**
 * @function:   uninit_debug_log_modlue
 * @brief:      δ��ʼ����ӡ��־��ģ��
 * @param[in]:  UINT32 disAddr  
 * @param[in]:  char *logPath   
 * @param[out]: None
 * @return:     void
 */
void uninit_debug_log_modlue(void)
{
    log_module_uninit();
}

/** @fn	void set_debug_para(UINT32 debug_level, UINT32 mod_idx_marked, UINT32 debug_content)
 *  @brief	���õ�ǰģ��ĵȼ�,������Ϣ.debug_mod_idx�����ǵ���ģ������ֵ,ĳһ��ģ������ֵ,����ģ������ֵ.
 *			���ȸ���debug_mod_idx��ֵȷ��Ҫ���õ�ģ����g_allmod_attr_info��ʼ��źͽ�����ţ�Ȼ���
 *			��ֹ��������ڵ�ģ�����õ��Եȼ��򸽼���Ϣ
 *  @param[in]  debug_level Ҫ���õĵ��Եȼ�.
 *  @param[in]  mod_idx_marked Ҫ���õĵ���ģ������,�ܷ������ǵ���ģ��,ģ����,����������ģ������
 *  @param[in]  debug_content Ҫ���õĵ��Ը�����Ϣ
 *  @return	  ��
 */
void set_debug_para(UINT32 debug_level, char *mod_name, UINT32 content)
{
    INT32 mod_idx = -1;
    UINT32 i = 0;
    MOD_ATTRIBUTE_INFO *pallmod_attr_info = NULL; /* ����ָ��ģ��������Ϣ�ṹ������ */

    /* ��ȡ����ģ������������׵�ַ */
    pallmod_attr_info = get_allmod_attr_info();
    if (NULL == pallmod_attr_info)
    {
        sys_printf("[dblog]g_allmod_attr_info is NULL ,filename %s,line %u\n", __FILENAME__, __LINE__);
        return;
    }

    /* ��������ģ�����������ڲ���,���������ģ�����ǲ���ĳ��ģ���� */
    for (i = 0; i < ALL_MOD_NUMS; i++)
    {
        if (strncmp(mod_name, pallmod_attr_info[i].mod_name, MAX_MOD_NAME) == 0)
        {
            /* ���ǵ���ģ����,mod_idx_marked�ĸ�16λ��0,��16λ�����ģ����ģ�������ڵ��±� */
            mod_idx = i;
            break;
        }
    }

    /*�޶�Ӧģ��id�� �˳�*/
    if (-1 == mod_idx)
    {
        return;
    }

    /* ���ô�star_id~end_id��ÿ��ģ��ĵȼ��͸�����Ϣ */
    /* ���ȼ�ֵ��0xffffffff ��ʾ������ */
    if (debug_level != 0xffffffff)
    {
        /* ���Եȼ������Ϸ���Ĭ�ϳ�DEFAULT_DEBUG_LEVEL */
        if (debug_level >= ALL_DBG_LEVEL_NUMS)
        {
            pallmod_attr_info[mod_idx].level = DEFAULT_DEBUG_LEVEL;
        }
        else
        {
            pallmod_attr_info[mod_idx].level = debug_level;
        }
    }

    pallmod_attr_info[mod_idx].content = content;
    return;
}

/** @fn	void get_debug_para_print(void)
 *  @brief	��ӡ��ǰ����ģ��ĵ��Բ���
 *  @param[out] ��ӡ���Եȼ���ģ����,������Ϣ
 *  @return	 ��
 */
void get_debug_para_print(void)
{
    UINT32 mod_id = 0; /* ģ�����������ģ����� */
    UINT32 content_val = 0; /* ������Ϣ���� */
    MOD_ATTRIBUTE_INFO *pallmod_attr_info = NULL; /* ����ָ��ģ��������Ϣ�ṹ������ */
    char snpr_tmp[128] = {0};

    /* ��ȡ����ģ��������Ϣ�ṹ��������׵�ַ */
    pallmod_attr_info = get_allmod_attr_info();
    if (NULL == pallmod_attr_info)
    {
        sys_printf("[dblog]mod para error\n");
        return;
    }

    /* ��ӡ�б� */
    sys_printf("%-16s\t%-10s\t%-10s\t%s(FILENAME,FUNCTION,LINE)\n", "modulename", "level", "log_level", "display");

    for (mod_id = 0; mod_id < ALL_MOD_NUMS; mod_id++)
    {
        if (mod_id >= PREINSTALL_ALL_MOD_NUMS &&  pallmod_attr_info[mod_id].mod_idx == UNUSE)
        {
           continue;
        }
        /* ��ӡģ���� */
        snprintf(snpr_tmp, sizeof(snpr_tmp) - 1, "%-16s\t", pallmod_attr_info[mod_id].mod_name);

        /* ��ӡ�ȼ� */
        snprintf(snpr_tmp + strlen(snpr_tmp), sizeof(snpr_tmp) - strlen(snpr_tmp) - 1, "%-10u\t", pallmod_attr_info[mod_id].level);

        /* ��ӡ��־�ȼ� */
        snprintf(snpr_tmp + strlen(snpr_tmp), sizeof(snpr_tmp) - strlen(snpr_tmp) - 1, "%-10u\t", pallmod_attr_info[mod_id].log_level);

        /* ��ӡ������Ϣ */
        content_val = pallmod_attr_info[mod_id].content;
        snprintf(snpr_tmp + strlen(snpr_tmp), sizeof(snpr_tmp) - strlen(snpr_tmp) - 1,
                 "%1u%1u%1u\n", ((content_val & ISFILENAME_SET_BIT) > 0), ((content_val & ISFUNCNAME_SET_BIT) > 0),
                 ((content_val & ISLINE_SET_BIT) > 0));
        sys_printf(snpr_tmp);
    }

    return;
}

/** @fn	inline MOD_ATTRIBUTE_INFO *get_allmod_attr_info(void)
 *  @brief	��ȡ����ģ��������Ϣ�ṹ������ָ��
 *  @param[in]  g_allmod_attr_info ����ģ��������Ϣ
 *  @param[out] ��
 *  @return	  g_allmod_attr_info ģ�����Խṹ������ָ��
 */
static inline MOD_ATTRIBUTE_INFO *get_allmod_attr_info(void)
{
    return g_allmod_attr_info;
}

/** @fn	inline DEBUG_LEVEL_ATTRIBUTE *get_all_level_attr_info(void)
 *  @brief	��ȡ���е��Եȼ����Խṹ������ָ��
 *  @param[in]  g_all_debuglevel_attr ���е��Եȼ�������Ϣ
 *  @param[out] ��
 *  @return	  g_all_debuglevel_attr ���е��Եȼ����Խṹ������ָ��
 */
static inline DEBUG_LEVEL_ATTRIBUTE *get_all_level_attr_info(void)
{
    return g_all_debuglevel_attr;
}

/**
 * @function:   set_level
 * @brief:      ���ô�ӡ��־�ȼ��Լ���ʾ
 * @param[in]:  char *mod_name ����ģ�������
 * @param[in]:  char level  ���õĵȼ�
 * @param[in]:  char type  ѡ�����õ�����
 * @param[in]:  UINT32 content  ���ø��ӵ���Ϣ
 * @param[out]: None
 * @return:     void
 */
void set_level_cont(char *mod_name, char level, char type, UINT32 content)
{
    INT32 mod_idx = -1;
    UINT32 i = 0;
    MOD_ATTRIBUTE_INFO *pallmod_attr_info = NULL; /* ����ָ��ģ��������Ϣ�ṹ������ */

    /* ��ȡ����ģ������������׵�ַ */
    pallmod_attr_info = get_allmod_attr_info();
    if (NULL == pallmod_attr_info)
    {
        sys_printf("[dblog]g_allmod_attr_info is NULL ,filename %s,line %u\n", __FILENAME__, __LINE__);
        return;
    }

    /* ��������ģ�����������ڲ���,���������ģ�����ǲ���ĳ��ģ���� */
    for (i = 0; i < ALL_MOD_NUMS; i++)
    {
        if (strncmp(mod_name, pallmod_attr_info[i].mod_name, MAX_MOD_NAME) == 0)
        {
            /* ���ǵ���ģ����,mod_idx_marked�ĸ�16λ��0,��16λ�����ģ����ģ�������ڵ��±� */
            mod_idx = i;
            break;
        }
    }

    if (strncmp(mod_name, ALL_MOD_NAME, MAX_MOD_NAME) == 0)
    {
        mod_idx = ALL_MOD_NUMS;
    }

    /*�޶�Ӧģ��id�� �˳�*/
    if (-1 == mod_idx)
    {
        return;
    }

    /* ���ô�star_id~end_id��ÿ��ģ��ĵȼ��͸�����Ϣ */
    /* ���ȼ�ֵ��0xffffffff ��ʾ������ */
    if (level != 0xffffffff && mod_idx != ALL_MOD_NUMS)
    {
        /* ���Եȼ������Ϸ���Ĭ�ϳ�DEFAULT_DEBUG_LEVEL */
        if (level >= ALL_DBG_LEVEL_NUMS)
        {
            if (type & CTL_COM)
            {
                pallmod_attr_info[mod_idx].level = DEFAULT_DEBUG_LEVEL;
            }

            if (type & CTL_LOG)
            {
                pallmod_attr_info[mod_idx].log_level = DEFAULT_DEBUG_LEVEL;
            }
        }
        else
        {
            if (type & CTL_COM)
            {
                pallmod_attr_info[mod_idx].level = level;
            }

            if (type & CTL_LOG)
            {
                pallmod_attr_info[mod_idx].log_level = level;
            }
        }

        pallmod_attr_info[mod_idx].content = content;
    }
    else if (level != 0xffffffff && mod_idx == ALL_MOD_NUMS)
    {
        for (i = 0; i < ALL_MOD_NUMS; i++)
        {
            if (level >= ALL_DBG_LEVEL_NUMS)
            {
                if (type & CTL_COM)
                {
                    pallmod_attr_info[i].level = DEFAULT_DEBUG_LEVEL;
                }

                if (type & CTL_LOG)
                {
                    pallmod_attr_info[i].log_level = DEFAULT_DEBUG_LEVEL;
                }
            }
            else
            {
                if (type & CTL_COM)
                {
                    pallmod_attr_info[i].level = level;
                }

                if (type & CTL_LOG)
                {
                    pallmod_attr_info[i].log_level = level;
                }
            }

            pallmod_attr_info[i].content = content;
        }
    }
}


/**
 * @fn log_add_storage
 * @brief �����־ģ��洢����
 * @param NULL
 * @return	0: �ɹ� <0 :ʧ��
 * @note 2017/04/11 qiuwei ����
 */
int debug_log_add_storage()
{
    return log_add_storage();
}

/**
 * @fn log_add_storage
 * @brief ɾ����־ģ��洢����
 * @param NULL
 * @return	0: �ɹ� <0 :ʧ��
 * @note 2017/04/11 qiuwei ����
 */
int debug_log_del_storage()
{
    return log_del_storage();
}


/**
 * @function:   get_db_log_vision
 * @brief:      ��ȡ����İ汾��Ϣ
 * @param[in]:  void
 * @param[out]: ��ӡ����汾��
 * @return:     void
 */
void get_db_log_vision(void)
{
    sys_printf("[dblog]DB_LOG_VISION:%s", DB_LOG_VISION);
}

/**
 * @function:   regeDebugModlue
 * @brief:      ģ��ע��
 * @param[in]:  char *ModlueName  
 * @param[in]:  int printLevel    
 * @param[in]:  int logLevel      
 * @param[in]:  int content       
 * @param[out]: None
 * @return:     int
 */
int regeDebugModlue(char *ModlueName, int printLevel,int logLevel,int content)
{
    int i=0;
    /*����Ƿ��Ѿ����ڸ�ģ����*/
    for(i = 0; i<ALL_MOD_NUMS; i++)
    {
        if (strncmp(g_allmod_attr_info[i].mod_name,ModlueName,MAX_MOD_NAME) == 0)
        {
            return FALSE;
        }
        if (g_allmod_attr_info[i].mod_idx == UNUSE)
        {
            break;
        }
    }

    /*Ѱ�ҿ�λ����ע��*/
    for(i = PREINSTALL_ALL_MOD_NUMS; i<ALL_MOD_NUMS; i++)
    {   
        if (g_allmod_attr_info[i].mod_idx == UNUSE)
        {
          g_allmod_attr_info[i].mod_idx = USED;
          snprintf(g_allmod_attr_info[i].mod_name, MAX_MOD_NAME - 1,"%s",ModlueName);
          g_allmod_attr_info[i].content = content;
          g_allmod_attr_info[i].log_level = logLevel;
          g_allmod_attr_info[i].level = printLevel;
          g_allmod_attr_info[i].belong_modclass = MOD_CUSTOM_CLASS_ID;
          return TRUE;
        }
    }
    return FALSE;
}


/**
 * @function:   get_debug_log_com_list
 * @brief:      ��ȡ��ӡ��������ĺ����б�
 
 * @param[out]: None
 * @return:     void *
 */
void *get_debug_log_com_list(void)
{
    return g_debuglogcom;
}

/**
 * @function:   debug_test
 * @brief:      ���Դ���
 * @param[in]   void
 * @return      void
 */
void debug_test(int num)
{
    while (1)
    {
        PDebug("sys_printf test %d \n", num);
        sys_task_delay(1000);
        PWarn("sys_printf test\n");
        sys_task_delay(1000);
        PError(">>>>>>>>>>>>>>>>>>wangjie4 : debug test !!!!>>>>>>>>>>>>>>>>>>\n");
        sys_task_delay(1000);
        PTraceStr("TEST %d %s",123,"test");
    }
}

/**
 * @function:   debug_test_init
 * @brief:      �����߳�
 * @param[in]   void
 * @return      int
 */
int debug_test_init(void)
{
    static int ret = 1;
    regeDebugModlue("TESTMODE", 7, 1, 7);
    if (ret == 1)
    {
        ret = sys_pthread_create(NULL, "db_log_test", TASK_PRIORITY_2, SIZE_32KB, 0, (FUNCPTR)debug_test, 1, 1);
    }
    else
    {
        PWarn("db_log_test alread run\n");
    }

    return ret;
}

