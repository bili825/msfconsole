/**
 * @file:   debug_ctl.c
 * @note:   2010-2020, 杭州海康威视数字技术股份有限公司
 * @brief   实现调试打印接口,和串口命令getDebug,setDebug,debugLog
 * @author: zhoufeng16
 * @date    2018/7/16
 * @note:
 * @note \n History:
 *  1.日    期: 2018/7/16
 *    作    者: zhoufeng16
 *    修改历史: 创建文件
 */

/*----------------------------------------------*/
/*                 包含头文件                   */
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
/*                 宏类型定义                   */
/*----------------------------------------------*/

/* 打印调试默认参数 */
#define DEFAULT_DEBUG_LEVEL RT_ERROR    /* /< 设备程序的每个模块的默认打印等级,'运行错误' */

/* 打印信息缓存区大小相关的宏 */
#define MAX_DBGMSG_LEN	256 /* /< 每条打印信息的最大长度 */

/* 打印信息的附加信息控制相关的宏 */
#define ISFILENAME_SET_BIT	0x04 /* /< 第2位被置成1,需要显示文件名 */
#define ISFUNCNAME_SET_BIT	0x02 /* /< 第1位被置成1,需要显示函数名 */
#define ISLINE_SET_BIT		0x01 /* /< 第0位被置成1,需要显示行号 */

#define ALL_MOD_NAME "ALL"      /* /< 设置所有模块时,输入的模块名 */

#define DB_LOG_VISION "V1.00"
#define CUSTOM_MODLUS_SUM 24
/* #define RECORD_DEBUG(level, arg...)  dev_debug(level, MOD_SYSFUN_RECORD, ##arg) */
#define ALL_MOD_NUMS (PREINSTALL_ALL_MOD_NUMS+CUSTOM_MODLUS_SUM)
#define UNUSE		0
#define USED		-2
/*----------------------------------------------*/
/*                结构体定义                    */
/*----------------------------------------------*/


/*----------------------------------------------*/
/*                 函数声明                     */
/*----------------------------------------------*/
static inline BOOL isprint_bylevel(UINT32 debug_level, UINT32 mod_curdebug_level);
static inline MOD_ATTRIBUTE_INFO *get_allmod_attr_info(void);
static inline DEBUG_LEVEL_ATTRIBUTE *get_all_level_attr_info(void);

/*----------------------------------------------*/
/*                 全局变量                     */
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



/* 所有调试等级属性数组 */
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

/* 对于每个模块进行配置 */
static MOD_ATTRIBUTE_INFO g_allmod_attr_info[ALL_MOD_NUMS] =
{
    /* 系统功能类 */
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

    /* 车机交互 */
    {MOD_CAN_INTERFACE,      "CAN",         DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_WIFI_INTERFACE,     "WIFI",        DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_VEH_INTERFACE,      "VEH",         DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_DSP_INTERFACE,      "DSP",         DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_UART_INTERFACE,     "UART",        DEBUG_NOTICE,    DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},
    {MOD_PHONE_INTERFACE,    "PHONE",       DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_SYSFUN_CLASS_ID, KEY_WARN},

    /* 硬件类 */
    {MOD_HW_MCU,            "MCU",          DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_BUTTON,         "BUTTON",       KEY_WARN,        DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_LED,            "LED",          DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_SENSOR,         "SENSOR",       DEBUG_NOTICE,    DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_GPS,            "GPS",          KEY_WARN,        DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_HW_SD,             "SD",           DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},

    /* T 1测试 */
    {MOD_T1_TEST,           "T1_TEST",      DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},
    {MOD_SYSFUN_PS_PACK,    "PS_TEST",      DEBUG_ALL_DAV,   DEBUG_CODE_INFO_FUNC_LINE, MOD_HW_CLASS_ID,     KEY_WARN},

    /* 其他系统类 */
    {MOD_OTHER,             "OTHER",        DEBUG_INFO,      DEBUG_CODE_INFO_FUNC_LINE, MOD_OTHER_CLASS_ID,  NON},

};

/* 使用默认的配置 */
DB_LOG_CONFIG g_debugLogConfig =
{
    CTL_COM_LOG, TRUE, TRUE
};

/*----------------------------------------------*/
/*                 函数定义                     */
/*----------------------------------------------*/

/**
 * @function:   isprint_bylevel
 * @brief:      判断这个等级的打印信息是否要输出
 * @param[in]   UINT32 debug_level
 * @param[in]   UINT32 mod_curdebug_level
 * @param[out]: 是否要输出
 * @return      static inline BOOL
 */
static inline BOOL isprint_bylevel(UINT32 debug_level, UINT32 mod_curdebug_level)
{
    BOOL b_print_out = FALSE; /* 是否要输出,默认不要输出 */

    /* 若这条打印信息的等级值小于等于设置的值，且不等于NON 则输出,因为等级值越小，等级越高 */
    if ((debug_level <= mod_curdebug_level) && (debug_level != NON))
    {
        b_print_out = TRUE;
    }

    return b_print_out;

}

/******************************对外接口*****************************************/

/**
 * @function:   print_debug_modlue_str
 * @brief:      组织打印放入缓存中，使用模块名称做索引
 * @param[in]:  UINT32 debug_level     调试等级 
 * @param[in]:  char *ModlueName      调试模块字符串名称
 * @param[in]:  const char *filename  调用文件
 * @param[in]:  UINT32 line           调试在文件中的行号
 * @param[in]:  const char *funname   函数名
 * @param[in]:  const char *format    可变参数个数
 * @param[in]:  ...                   可变参数 
 * @param[out]: None
 * @return:     void
 */
void print_debug_modlue_str(UINT32 debug_level, char *ModlueName, const char *filename, UINT32 line, const char *funname, const char *format, ...)
{
    int i = 0;
    va_list ap; /* 可变参数结构体 */
    char sz_degmsg[MAX_DBGMSG_LEN]; /* 打印信息数组 */
    for(i=0; i<ALL_MOD_NUMS ;i++)
    {
        if (strncmp(g_allmod_attr_info[i].mod_name,ModlueName,MAX_MOD_NAME) == 0)
        {
            break;
        }
    }
    if (i == ALL_MOD_NUMS)
    {
        /*如果模块名称不存在，则以及Other进行打印*/
        sys_printf("[dblog]error print_debug no modlue %s\n",ModlueName);
        i = MOD_OTHER;
    }
    va_start(ap, format);
    vsnprintf(sz_degmsg,sizeof(sz_degmsg)-1,format,ap);
    va_end(ap);
    print_debug(debug_level,i,filename,line,funname,sz_degmsg);

}

/** @fn	void print_debug(UINT32 debug_level, UINT32 debug_mod_idx, const INT8 *filename, UINT32 line, const INT8 *funname, const INT8 *format, ...)
 *  @brief	组织打印信息然后放入缓存区内
 *  @param[in]  debug_level 调试等级
 *  @param[in]  debug_mod_idx 调试模块索引
 *  @param[in]  filename 调用文件
 *  @param[in]  line 调试在文件中的行号
 *  @param[in]  funname 函数名
 *  @param[in]  format 可变参数个数
 *  @param[in]  ... 可变参数
 *  @param[out] 打印信息
 *  @return	  无
 */
void print_debug(UINT32 debug_level, UINT32 debug_mod_idx, const char *filename, UINT32 line, const char *funname, const char *format, ...)
{
    UINT32 cur_debug_mod_idx = 0; /* 本条打印信息的模块索引 */
    UINT32 cur_debug_content = 0; /* 某个模块设置的附加控制信息 */
    UINT32 cur_debug_level = 0; /* 某个模块设置的调试等级 */
    UINT32 cur_debug_log_level = 0; /* 某个模块设置的日志等级 */

    char sz_degmsg[MAX_DBGMSG_LEN]; /* 打印信息数组 */
    char *plevelname = NULL; /* 调试等级名称 */
    char *pmod_name = NULL; /* 指向本模块的模块名 */

    SYS_DATE_TIME_T sysTime;

    va_list ap; /* 可变参数结构体 */
    INT32 msglen = 0; /* sz_degmsg数组内字符串的长度 */
    char sz_msg_tmp[MAX_DBGMSG_LEN]; /* 用于临时存放字符串 */
    INT32 tmp_msg_len = 0; /* 临时字符串数组内字符串的长度 */
    MOD_ATTRIBUTE_INFO *pallmod_attr_info = NULL; /* 用于指向模块属性信息结构体数组 */
    DEBUG_LEVEL_ATTRIBUTE *pall_level_attr_info = NULL; /* 用于指向调试等级属性数组 */
    int ret = 0;

    if ((NULL == filename) || (NULL == funname) || (NULL == format))
    {
        return;
    }

    memset(sz_degmsg, 0, MAX_DBGMSG_LEN);
    memset(sz_msg_tmp, 0, MAX_DBGMSG_LEN);

    /* 获取所有模块类属性信息结构体数组首地址 */
    pallmod_attr_info = get_allmod_attr_info();
    if (NULL == pallmod_attr_info)
    {
        sys_printf("[dblog]error g_allmod_attr_info have no space,filename %s,line %u\n", __FILENAME__, __LINE__);
        return;
    }

    /* 获取调试等级属性数组的首地址 */
    pall_level_attr_info = get_all_level_attr_info();
    if (NULL == pall_level_attr_info)
    {
        sys_printf("[dblog]error g_all_debuglevel_attr have no space,filename %s,line %u\n", __FILENAME__, __LINE__);
        return;
    }

    /* 若debug_level超出范围,则等级设为默认值,若不超，则还是该值 */
    if (debug_level >= ALL_DBG_LEVEL_NUMS)
    {
        debug_level = DEFAULT_DEBUG_LEVEL;
    }

    /* 获取这条打印所属模块的调试控制信息,调试等级以及附加信息 */
    if (debug_mod_idx >= ALL_MOD_NUMS)
    {
        /* 若模块索引超出范围,默认模块索引是MOD_OTHER,附加默认信息是0 */
        cur_debug_mod_idx = MOD_OTHER;
        cur_debug_content = 0;
        cur_debug_level = DEFAULT_DEBUG_LEVEL;
        cur_debug_log_level = DEFAULT_DEBUG_LEVEL;
    }
    else
    {
        /* debug_mod_idx和pallmod_attr_info[debug_mod_idx].mod_idx是一样的 */
        cur_debug_mod_idx = debug_mod_idx;
        cur_debug_content = pallmod_attr_info[debug_mod_idx].content;
        cur_debug_level = pallmod_attr_info[debug_mod_idx].level;
        cur_debug_log_level = pallmod_attr_info[debug_mod_idx].log_level;
    }

    /* 需要输出调试信息, 根据要求组合打印信息 */

    /* 获取这条打印信息的调试等级名称 */
    plevelname = pall_level_attr_info[debug_level].level_name_show;

    /* 获取模块名称 */
    pmod_name = pallmod_attr_info[cur_debug_mod_idx].mod_name;

    /* 读取系统时间，然后初始化 */
    ret = sys_datetime_get(&sysTime);

    /* 整个打印信息的格式[time][modname][level][file][line][function][contents] */

    /* 默认没有附加信息,先写[time][modname][level].以下是将打印信息按格式拷贝到sz_degmsg中,都是先将每条
     * 打印信息的某些项snprintf到临时数组sz_msg_tmp中,若sz_msg_tmp中字符串的长度 + sz_degmsg中原有字符串
     * 的长度 < 每条打印信息的最大长度,则追加到数组sz_degmsg中 */



    /* 包含线程ID、时间、打印等级，以及代码行数 */
    tmp_msg_len = snprintf(sz_degmsg, sizeof(sz_degmsg), "[%08x][%02d %02d:%02d:%02d][%s][%s]",
                           (int)sys_pthread_self(), sysTime.uMonth, sysTime.uHour, sysTime.uMinute, sysTime.uSec,
                           pmod_name, plevelname);

    if ((tmp_msg_len > 0) && ((msglen + tmp_msg_len) < MAX_DBGMSG_LEN))
    {
        /* 若要追加的长度加上原有长度不越界,则追加。以下追加'文件名'，'行号'，'函数名'时类似处理 */
        strcat(&sz_degmsg[msglen], sz_msg_tmp);
        msglen += tmp_msg_len;
    }

    /* 若要包含文件名 */
    if ((cur_debug_content & ISFILENAME_SET_BIT) > 0)
    {
        tmp_msg_len = snprintf(sz_msg_tmp, MAX_DBGMSG_LEN, "[%s]", filename);
        if ((tmp_msg_len > 0) && ((msglen + tmp_msg_len) < MAX_DBGMSG_LEN))
        {
            strcat(&sz_degmsg[msglen], sz_msg_tmp);
            msglen += tmp_msg_len;
        }
    }

    /* 若要包含行号 */
    if ((cur_debug_content & ISLINE_SET_BIT) > 0)
    {
        tmp_msg_len = snprintf(sz_msg_tmp, MAX_DBGMSG_LEN, "[%u]", line);
        if ((tmp_msg_len > 0) && ((msglen + tmp_msg_len) < MAX_DBGMSG_LEN))
        {
            strcat(&sz_degmsg[msglen], sz_msg_tmp);
            msglen += tmp_msg_len;
        }
    }

    /* 若要包括函数名 */
    if ((cur_debug_content & ISFUNCNAME_SET_BIT) > 0)
    {
        tmp_msg_len = snprintf(sz_msg_tmp, MAX_DBGMSG_LEN, "[%s]", funname);
        if ((tmp_msg_len > 0) && ((msglen + tmp_msg_len) < MAX_DBGMSG_LEN))
        {
            strcat(&sz_degmsg[msglen], sz_msg_tmp);
            msglen += tmp_msg_len;
        }
    }

    /* 接着添加打印内容 */
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

        /* 如果要包含颜色 */
        if (g_debugLogConfig.colorEnable)
        {
            tmp_msg_len = snprintf(sz_msg_tmp, MAX_DBGMSG_LEN, "%s%s%s", g_all_debuglevel_attr[debug_level].leve_color, sz_degmsg, DEBUG_COLOR_CLOSE);
            if (tmp_msg_len < MAX_DBGMSG_LEN)
            {
                strncpy(sz_degmsg, sz_msg_tmp, sizeof(sz_degmsg) - 1);
            }
        }

        /* 若打印信息要输出到串口 */
        if (isprint_bylevel(debug_level, cur_debug_level))
        {
            /* 判断输出的位置 */
            sys_printf("%s", sz_degmsg);
        }
    }
    return;
}


/**
 * @function:   init_debug_log_modlue
 * @brief:      初始化打印日志的模块
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
 * @brief:      未初始化打印日志的模块
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
 *  @brief	设置当前模块的等级,附加信息.debug_mod_idx可能是单个模块索引值,某一类模块索引值,所有模块索引值.
 *			首先根据debug_mod_idx的值确定要设置的模块在g_allmod_attr_info起始序号和结束序号，然后对
 *			起止序号区间内的模块设置调试等级或附加信息
 *  @param[in]  debug_level 要设置的调试等级.
 *  @param[in]  mod_idx_marked 要设置的调试模块索引,能否区分是单个模块,模块类,或者是所有模块索引
 *  @param[in]  debug_content 要设置的调试附加信息
 *  @return	  无
 */
void set_debug_para(UINT32 debug_level, char *mod_name, UINT32 content)
{
    INT32 mod_idx = -1;
    UINT32 i = 0;
    MOD_ATTRIBUTE_INFO *pallmod_attr_info = NULL; /* 用于指向模块属性信息结构体数组 */

    /* 获取所有模块属性数组的首地址 */
    pallmod_attr_info = get_allmod_attr_info();
    if (NULL == pallmod_attr_info)
    {
        sys_printf("[dblog]g_allmod_attr_info is NULL ,filename %s,line %u\n", __FILENAME__, __LINE__);
        return;
    }

    /* 先在所有模块属性数组内查找,查找输入的模块名是不是某个模块名 */
    for (i = 0; i < ALL_MOD_NUMS; i++)
    {
        if (strncmp(mod_name, pallmod_attr_info[i].mod_name, MAX_MOD_NAME) == 0)
        {
            /* 若是单个模块名,mod_idx_marked的高16位是0,低16位是这个模块在模块数组内的下标 */
            mod_idx = i;
            break;
        }
    }

    /*无对应模块id， 退出*/
    if (-1 == mod_idx)
    {
        return;
    }

    /* 设置从star_id~end_id的每个模块的等级和附加信息 */
    /* 若等级值是0xffffffff 表示不设置 */
    if (debug_level != 0xffffffff)
    {
        /* 调试等级若不合法，默认成DEFAULT_DEBUG_LEVEL */
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
 *  @brief	打印当前所有模块的调试参数
 *  @param[out] 打印调试等级，模块名,附加信息
 *  @return	 无
 */
void get_debug_para_print(void)
{
    UINT32 mod_id = 0; /* 模块属性数组的模块序号 */
    UINT32 content_val = 0; /* 附加信息控制 */
    MOD_ATTRIBUTE_INFO *pallmod_attr_info = NULL; /* 用于指向模块属性信息结构体数组 */
    char snpr_tmp[128] = {0};

    /* 获取所有模块属性信息结构体数组的首地址 */
    pallmod_attr_info = get_allmod_attr_info();
    if (NULL == pallmod_attr_info)
    {
        sys_printf("[dblog]mod para error\n");
        return;
    }

    /* 打印列表 */
    sys_printf("%-16s\t%-10s\t%-10s\t%s(FILENAME,FUNCTION,LINE)\n", "modulename", "level", "log_level", "display");

    for (mod_id = 0; mod_id < ALL_MOD_NUMS; mod_id++)
    {
        if (mod_id >= PREINSTALL_ALL_MOD_NUMS &&  pallmod_attr_info[mod_id].mod_idx == UNUSE)
        {
           continue;
        }
        /* 打印模块名 */
        snprintf(snpr_tmp, sizeof(snpr_tmp) - 1, "%-16s\t", pallmod_attr_info[mod_id].mod_name);

        /* 打印等级 */
        snprintf(snpr_tmp + strlen(snpr_tmp), sizeof(snpr_tmp) - strlen(snpr_tmp) - 1, "%-10u\t", pallmod_attr_info[mod_id].level);

        /* 打印日志等级 */
        snprintf(snpr_tmp + strlen(snpr_tmp), sizeof(snpr_tmp) - strlen(snpr_tmp) - 1, "%-10u\t", pallmod_attr_info[mod_id].log_level);

        /* 打印附加信息 */
        content_val = pallmod_attr_info[mod_id].content;
        snprintf(snpr_tmp + strlen(snpr_tmp), sizeof(snpr_tmp) - strlen(snpr_tmp) - 1,
                 "%1u%1u%1u\n", ((content_val & ISFILENAME_SET_BIT) > 0), ((content_val & ISFUNCNAME_SET_BIT) > 0),
                 ((content_val & ISLINE_SET_BIT) > 0));
        sys_printf(snpr_tmp);
    }

    return;
}

/** @fn	inline MOD_ATTRIBUTE_INFO *get_allmod_attr_info(void)
 *  @brief	获取所有模块属性信息结构体数组指针
 *  @param[in]  g_allmod_attr_info 所有模块属性信息
 *  @param[out] 无
 *  @return	  g_allmod_attr_info 模块属性结构体数组指针
 */
static inline MOD_ATTRIBUTE_INFO *get_allmod_attr_info(void)
{
    return g_allmod_attr_info;
}

/** @fn	inline DEBUG_LEVEL_ATTRIBUTE *get_all_level_attr_info(void)
 *  @brief	获取所有调试等级属性结构体数组指针
 *  @param[in]  g_all_debuglevel_attr 所有调试等级属性信息
 *  @param[out] 无
 *  @return	  g_all_debuglevel_attr 所有调试等级属性结构体数组指针
 */
static inline DEBUG_LEVEL_ATTRIBUTE *get_all_level_attr_info(void)
{
    return g_all_debuglevel_attr;
}

/**
 * @function:   set_level
 * @brief:      设置打印日志等级以及显示
 * @param[in]:  char *mod_name 设置模块的名称
 * @param[in]:  char level  设置的等级
 * @param[in]:  char type  选择设置的类型
 * @param[in]:  UINT32 content  设置附加的信息
 * @param[out]: None
 * @return:     void
 */
void set_level_cont(char *mod_name, char level, char type, UINT32 content)
{
    INT32 mod_idx = -1;
    UINT32 i = 0;
    MOD_ATTRIBUTE_INFO *pallmod_attr_info = NULL; /* 用于指向模块属性信息结构体数组 */

    /* 获取所有模块属性数组的首地址 */
    pallmod_attr_info = get_allmod_attr_info();
    if (NULL == pallmod_attr_info)
    {
        sys_printf("[dblog]g_allmod_attr_info is NULL ,filename %s,line %u\n", __FILENAME__, __LINE__);
        return;
    }

    /* 先在所有模块属性数组内查找,查找输入的模块名是不是某个模块名 */
    for (i = 0; i < ALL_MOD_NUMS; i++)
    {
        if (strncmp(mod_name, pallmod_attr_info[i].mod_name, MAX_MOD_NAME) == 0)
        {
            /* 若是单个模块名,mod_idx_marked的高16位是0,低16位是这个模块在模块数组内的下标 */
            mod_idx = i;
            break;
        }
    }

    if (strncmp(mod_name, ALL_MOD_NAME, MAX_MOD_NAME) == 0)
    {
        mod_idx = ALL_MOD_NUMS;
    }

    /*无对应模块id， 退出*/
    if (-1 == mod_idx)
    {
        return;
    }

    /* 设置从star_id~end_id的每个模块的等级和附加信息 */
    /* 若等级值是0xffffffff 表示不设置 */
    if (level != 0xffffffff && mod_idx != ALL_MOD_NUMS)
    {
        /* 调试等级若不合法，默认成DEFAULT_DEBUG_LEVEL */
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
 * @brief 添加日志模块存储功能
 * @param NULL
 * @return	0: 成功 <0 :失败
 * @note 2017/04/11 qiuwei 创建
 */
int debug_log_add_storage()
{
    return log_add_storage();
}

/**
 * @fn log_add_storage
 * @brief 删除日志模块存储功能
 * @param NULL
 * @return	0: 成功 <0 :失败
 * @note 2017/04/11 qiuwei 创建
 */
int debug_log_del_storage()
{
    return log_del_storage();
}


/**
 * @function:   get_db_log_vision
 * @brief:      获取组件的版本信息
 * @param[in]:  void
 * @param[out]: 打印输出版本号
 * @return:     void
 */
void get_db_log_vision(void)
{
    sys_printf("[dblog]DB_LOG_VISION:%s", DB_LOG_VISION);
}

/**
 * @function:   regeDebugModlue
 * @brief:      模块注册
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
    /*检查是否已经存在改模块了*/
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

    /*寻找空位进行注册*/
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
 * @brief:      获取打印调试组件的函数列表
 
 * @param[out]: None
 * @return:     void *
 */
void *get_debug_log_com_list(void)
{
    return g_debuglogcom;
}

/**
 * @function:   debug_test
 * @brief:      测试代码
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
 * @brief:      测试线程
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

