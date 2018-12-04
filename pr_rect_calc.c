/******************************************************************************
*
* 版权信息：版权所有 (c) 2010~2018, 杭州海康威视软件公司, 保留所有权利
*
* 文件名称：call_pr.c
* 文件标识：CALL_PR_C
* 摘    要：车牌识别预处理及调用牌识库函数实现文件
*
* 当前版本：1.0
* 作    者：罗义平
* 日    期：2014年10月11号
* 备    注：
*
******************************************************************************/

#include <string.h>
#include <math.h>
#include <stdio.h>
#include "mpr_common.h"
#include "call_pr.h"
#include "mpr_queue.h"
#include "Hik_debug.h"
#include "pr_rect_calc.h"

/**************************************************************************************************
* 功 能：计算当前帧率
* 参 数：*
*         mpr_pr_hdl              - I    模块句柄
*         cur_frm_inf             - I    当前帧帧信息
*         frame_rate              - O    帧率
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
HRESULT VTS_MPR_get_frame_rate(MPR_PR_HANDLE    *hdl,
                               VCA_FRAME_HEADER *cur_frm_inf,
                               unsigned int     *frame_rate)
{
    unsigned int frm_dis, time_dis;

    CHECK_ERROR((NULL == hdl || NULL == cur_frm_inf), HIK_MPR_PS_PTR_NULL); // 指针为空

    if (hdl->frm_rate == 0)
    {
        hdl->frm_rate = cur_frm_inf->frame_rate;
        if (hdl->frm_rate == 0)
        {
            hdl->frm_rate = 12;
        }
        hdl->frm_num_base  = cur_frm_inf->frame_num;
        hdl->frm_time_base = cur_frm_inf->time_stamp;
    }

    if (cur_frm_inf->frame_num - hdl->frm_num_base >= 50
        && cur_frm_inf->time_stamp - hdl->frm_time_base > 0)
    {
        frm_dis            = cur_frm_inf->frame_num - hdl->frm_num_base;
        time_dis           = cur_frm_inf->time_stamp - hdl->frm_time_base;
        hdl->frm_rate      = 1000 * frm_dis / time_dis;
        hdl->frm_num_base  = cur_frm_inf->frame_num;
        hdl->frm_time_base = cur_frm_inf->time_stamp;
    }
    *frame_rate = hdl->frm_rate;
	PRTF("hdl->frm_rate  =%d \n",hdl->frm_rate);
    return HIK_MPR_S_OK;
}

/**************************************************************************************************
* 功 能：计算牌识区域个数和对应的牌识参数，直接利用动态区域作为牌识区域
* 参 数：*
*         dynamic_pr_rect        - I    当前车道动态牌识区域
*         lane                   - I    车道号
*         obj_pr_cur             - O    牌识区域参数
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
HRESULT VTS_MPR_get_pr_rect(VCA_RECT_S            *rect_s,
                            int                    lane,
                            MPR_OBJ_PR_RECT_ARRAY *obj_pr_cur)
{
    MPR_OBJ_PR_RECT *obj_rect;

    CHECK_ERROR((NULL == rect_s) || (NULL == obj_pr_cur), HIK_MPR_FLW_PTR_NULL);

    // 开关打开时才调用预测牌识区域模块
    if ((rect_s->width >= 40) && (rect_s->height >= 10) && (obj_pr_cur->pr_num < ITS_MAX_VEH_NUM - 1))
    {
        obj_rect = &obj_pr_cur->obj_rect[obj_pr_cur->pr_num];
        obj_rect->direct         = 2;        // 方向未知
        obj_rect->lane           = lane;
        obj_rect->rect_type      = 4;
        obj_rect->pr_rect.left   = rect_s->x;
        obj_rect->pr_rect.right  = rect_s->x + rect_s->width;
        obj_rect->pr_rect.top    = rect_s->y;
        obj_rect->pr_rect.bottom = rect_s->y + rect_s->height;
        obj_rect->use_init_rect  = 0;
        obj_pr_cur->pr_num++;


    }

    return HIK_MPR_S_OK;
}

/****************************************************************************************************************
* 函  数：MPR_get_pre_one_rect_trig
* 功  能：利用历史挑选车牌位置和速度，预测当前帧的牌识的定位区域。
* 参  数：
*         trig_plate              -I  历史车牌数据
*         pr_up_line              -I  牌识区域上边界
*         time_dis，frame_dis     -I  当前帧与历史车牌时间戳间隔、帧间隔
*         img_w，img_h            -I  图像宽高
*         obj_pr_cur              -O  预测的牌识区域
* 返回值：错误状态码
*****************************************************************************************************************/
HRESULT VTS_MPR_get_pre_rect(MPR_PS_PLATE          *trig_plate,
							 VCA_BOX_S			   *init_pr_rect,
                             int                    time_dis,
                             int                    frame_dis,
                             int                    img_w,
                             int                    img_h,
                             MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur)
{
    int         thred_ex = 100;                                                     // 扩展阈值
    int         pixel_vx = 0;
    int         pixel_vy = 0;
    struct rect rect_temp;                                                          // 预测的牌识区域

    MPR_OBJ_PR_RECT *obj_rect = NULL;                                               // 牌识区域目标框
    struct rect      rect_plate, rect_pre;

    CHECK_ERROR((NULL == trig_plate || NULL == pr_rect_cur), HIK_MPR_FLW_PTR_NULL); // 指针为空

    pixel_vx = trig_plate->latest_res_for_cur_plt.ps_info.pixel_vx;
    pixel_vy = trig_plate->latest_res_for_cur_plt.ps_info.pixel_vy;

    memcpy(&rect_plate, &trig_plate->latest_res_for_cur_plt.pr_info.segment_rect, sizeof(struct rect));

    if (pr_rect_cur->pr_num >= ITS_MAX_VEH_NUM)
    {
        return HIK_MPR_S_OK;
    }

    // 水平方向位置预测
    if (pixel_vx >= 0)
    {
        rect_temp.lx = rect_plate.lx + (pixel_vx * time_dis) / 100 - thred_ex;
        rect_temp.rx = rect_plate.rx + (pixel_vx * time_dis) / 100 + thred_ex;
    }
    else
    {
        rect_temp.lx = rect_plate.lx + (pixel_vx * time_dis) / 100 - thred_ex;
        rect_temp.rx = rect_plate.rx + (pixel_vx * time_dis) / 100 + thred_ex;
    }

    // 垂直方向位置预测
    if (pixel_vy >= 0)
    {
        rect_temp.ty = rect_plate.ty + (pixel_vy * time_dis) / 100 - thred_ex;
        rect_temp.by = rect_plate.by + (pixel_vy * time_dis) / 100 + thred_ex;
    }
    else
    {
        rect_temp.ty = rect_plate.ty + (pixel_vy * time_dis) / 100 - thred_ex;
        rect_temp.by = rect_plate.by + (pixel_vy * time_dis) / 100 + thred_ex;
    }

    rect_pre.lx = rect_temp.lx + thred_ex;
    rect_pre.rx = rect_temp.rx - thred_ex;
    rect_pre.ty = rect_temp.ty + thred_ex;
    rect_pre.by = rect_temp.by - thred_ex;

    // 预测位置边界值校验
	if (rect_temp.lx < init_pr_rect->left)
    {
		rect_temp.lx = init_pr_rect->left;
    }
	if (rect_temp.rx >= init_pr_rect->right)
    {
		rect_temp.rx = init_pr_rect->right-1;
    }
	if (rect_temp.ty < init_pr_rect->top)
    {
		rect_temp.ty = init_pr_rect->top;
    }
	if (rect_temp.by >= init_pr_rect->bottom)
    {
		rect_temp.by = init_pr_rect->bottom-1;
    }
    /*if ((rect_temp.lx >= img_w - 1) || (rect_temp.rx <= 40)
        || (rect_temp.ty >= img_h - 1) || (rect_temp.by <= 10))
    {
        memset(&rect_temp, 0, sizeof(struct rect));
    }*/

    //// 对于不符合条件的预测块，丢弃；
    //if (((rect_temp.ty + rect_temp.by) < 2 * pr_up_line)
    //    || (rect_temp.by == 0))
    //{
    //    return HIK_MPR_S_OK;
    //}

	//// 对于不符合条件的预测块，丢弃；
	if (rect_temp.by - rect_temp.ty <= thred_ex || rect_temp.rx - rect_temp.lx <= 2*thred_ex)
	{
		return HIK_MPR_S_OK;
	}

    // 预测位置输出
    obj_rect            = &pr_rect_cur->obj_rect[pr_rect_cur->pr_num];
    obj_rect->plate_id  = trig_plate->latest_res_for_cur_plt.obj_id;
    obj_rect->rect_type = MPR_PRE_PR_RECT;
    obj_rect->direct    = trig_plate->latest_res_for_cur_plt.ps_info.direction;
    obj_rect->lane      = trig_plate->latest_res_for_cur_plt.lane;

    obj_rect->pr_rect.left   = rect_temp.lx;
    obj_rect->pr_rect.right  = rect_temp.rx;
    obj_rect->pr_rect.top    = rect_temp.ty;
    obj_rect->pr_rect.bottom = rect_temp.by;


    obj_rect->no_pr_num     = time_dis / 40;
    obj_rect->use_init_rect = 0;

    memcpy(obj_rect->pr_result, trig_plate->latest_res_for_cur_plt.pr_info.pr_result, 32);

    // 车牌共享数据赋值--车牌位置
    memcpy(&obj_rect->pr_share_info.pre_seg_rect, &rect_pre, sizeof(PR_RECT));

    pr_rect_cur->pr_num++;

    return HIK_MPR_S_OK;
}

/*****************************************************************************************************************
* 功  能：判断各车道是否处于拥堵状态；分车道进行统计，各车道的拥堵状态可能不同
* 参  数：
*         handle             -I  多帧句柄
*         frame              -I  当前帧信息
*         jam_flag           -O  拥堵标志
* 返回值：返回状态码
*****************************************************************************************************************/
HRESULT VTS_MPR_get_jam_flag(void                 *pr_handle,
                             ITS_FRM              *frame,
                             unsigned char        *jam_flag,
                             MPR_MODULE_INTERFACE *module_interface)
{
    int                 i, lane;
    int                 cmp_num = 0;                                     // 已考察过的历史车牌总数
    int                 jam_plate_num;                                   // 单个车道当前拥堵车辆总数
    int                 mode          = 0;                               // 正常车辆识别类型
    int                 speed_thred   = MPR_JAM_SPEED_THRED;             // 像素速度阈值
    int                 section       = 0;
    MPR_PS_PLATE_QUEUE *ps_plate_all  = NULL;
    MPR_PLATE_INFO     *plate_cur     = NULL;
    MPR_PR_HANDLE      *mpr_pr_handle = (MPR_PR_HANDLE *)pr_handle;          // 获取当前句柄
    VCA_BOX_S          *cur_rect      = NULL;

    // 参数校验
    CHECK_ERROR((NULL == mpr_pr_handle || NULL == frame), HIK_MPR_FLW_PTR_NULL);
    CHECK_ERROR(NULL == module_interface, HIK_MPR_PS_PTR_NULL);

    // ps_hdl = &mpr_pr_handle->ps_hdl;
    memset(jam_flag, 0, ITS_MAX_LANE_NUM);                                 // 各车道拥堵标记 初始化为0
    for (lane = 0; lane < ITS_MAX_LANE_NUM; lane++)
    {
        mode = mpr_pr_handle->pr_param_build.dwRecogMode;

        ps_plate_all = &mpr_pr_handle->ps_plate_all[lane];

        if (!VTS_MPR_trig_queue_empty(ps_plate_all))
        {
            i = (ps_plate_all->back - 1 + MPR_PS_QUEUE_CAP) % MPR_PS_QUEUE_CAP;

            // 队列车辆两个或以上
            if (ps_plate_all->count > 1)
            {
                jam_plate_num = 0;
                cmp_num       = 0;
                while (i != ps_plate_all->back && cmp_num < ps_plate_all->count)
                {
                    cmp_num++;
                    plate_cur   = &ps_plate_all->ps_plate_array[i].latest_res_for_cur_plt;
                    cur_rect    = (VCA_BOX_S *)(&(plate_cur->pr_info.segment_rect));
                    section     = (module_interface->img_h - cur_rect->bottom) / 50;
                    speed_thred = MPR_JAM_SPEED_THRED * module_interface->plate_w_h_info.pr_w_h.height_thred[lane][section]
                    / 20;

                    // 在阈值时间内 像素速度小于阈值 拥堵数++
                    if ((abs(plate_cur->ps_info.pixel_vy) < speed_thred)
                        && (frame->frm_header.time_stamp - plate_cur->frm_info.time_stamp < 120000))
                    {
                        jam_plate_num++;
                    }
                    i = (i - 1 + MPR_PS_QUEUE_CAP) % MPR_PS_QUEUE_CAP;
                }
                // 当前车道存在2个以上拥堵车辆，对应车道为拥堵
                if (jam_plate_num > 1)
                {
                    jam_flag[lane] = 1;
                }
            }

            // 牌识区域高度大于阈值
            else if (mpr_pr_handle->init_pr_rect.bottom - mpr_pr_handle->init_pr_rect.top >= 600 - mpr_pr_handle->sss * 100)
            {
                plate_cur = &ps_plate_all->ps_plate_array[i].latest_res_for_cur_plt;
                cur_rect  = (VCA_BOX_S *)(&(plate_cur->pr_info.segment_rect));

                // 背向识别，像素速度小于阈值，牌识区域高度大于阈值
                if ((mode == 1) && (abs(plate_cur->ps_info.pixel_vy) < speed_thred)
                    && (cur_rect->bottom - mpr_pr_handle->init_pr_rect.top > 400 - mpr_pr_handle->sss * 80))
                {
                    // 对应车道拥堵标记 记为1
                    jam_flag[lane] = 1;
                }
                // 正向识别，像素速度小于阈值，
                else if ((mode == 0) && (abs(plate_cur->ps_info.pixel_vy) < MPR_JAM_SPEED_THRED / 2)
                    && (mpr_pr_handle->init_pr_rect.bottom - cur_rect->bottom > 350 - mpr_pr_handle->sss * 30))
                {
                    jam_flag[lane] = 1;
                }
            }
        }
    }

    return HIK_MPR_S_OK;
}

/****************************************************************************************************************
* 函  数：MPR_pr_sort_speed
* 功  能：根据有车牌ID（牌选目标）的车牌速度对牌识调用顺序进行牌识，速度快的优先。
* 参  数：
*         obj_pr_last            -I  上一帧牌识目标区域
*         obj_pr_cur             -I  牌识目标区域
*         pr_turn                -O  当前模块排序后的顺序
*         sort_num               -O  当前模块参与排序的目标数
*         sort_flag              -O  已排序的目标编号标记
* 返回值：错误状态码
*****************************************************************************************************************/
HRESULT VTS_MPR_pr_sort_speed(MPR_OBJ_PR_RECT_ARRAY *obj_pr_last,
                              MPR_OBJ_PR_RECT_ARRAY *obj_pr_cur,
                              int                   *pr_turn,
                              int                   *sort_num,
                              int                   *sort_flag)
{
    int              num_temp = 0;                         // 当前模块参与排序的目标数
    int              i, j;
    int              turn_src[ITS_MAX_VEH_NUM]    = { 0 }; // 排序前的顺序
    int              turn_sorted[ITS_MAX_VEH_NUM] = { 0 }; // 排序后的顺序
    int              turn_temp     = 0;
    MPR_OBJ_PR_RECT *obj_rect      = NULL;
    MPR_OBJ_PR_RECT *obj_rect_last = NULL;
    VCA_BOX_S       *cur_rect      = NULL;
    VCA_BOX_S       *last_rect     = NULL;
    short            rect_dis      = 0;

    CHECK_ERROR((NULL == obj_pr_last || NULL == obj_pr_cur), HIK_MPR_PS_PTR_NULL);

    memset(turn_src, 0, sizeof(int)*ITS_MAX_VEH_NUM);
    memset(turn_sorted, 0, sizeof(int)*ITS_MAX_VEH_NUM);

    obj_rect      = obj_pr_cur->obj_rect;
    obj_rect_last = obj_pr_last->obj_rect;

    // 找出上一帧有ID的目标区域，并进行标记
    for (i = 0; i < obj_pr_cur->pr_num; i++)
    {
        if (sort_flag[i] == 1)
        {
            continue;
        }
        if (obj_rect[i].plate_id > 0)
        {
            sort_flag[i]       = 1;
            turn_src[num_temp] = i;
            num_temp++;
        }
    }

    // 单个目标，直接插入；多个目标，需要根据历史车牌下边界进行排序
    if (num_temp == 1)
    {
        turn_sorted[0] = turn_src[0];
    }
    else if (num_temp > 1)
    {
        for (i = 0; i < num_temp - 1; i++)
        {
            for (j = i + 1; j < num_temp; j++)
            {
                if (obj_rect[turn_src[i]].pr_rect.bottom < obj_rect[turn_src[j]].pr_rect.bottom)
                {
                    turn_temp   = turn_src[i];
                    turn_src[i] = turn_src[j];
                    turn_src[j] = turn_temp;
                }
            }
        }
        memcpy(turn_sorted, turn_src, sizeof(int) * num_temp);
    }
    else
    {
        // do nothing;
    }

    memcpy(pr_turn, turn_sorted, sizeof(int) * num_temp);
    *sort_num = num_temp;

    // 对上一帧已经识别过、且位置基本没有变化的目标取消，即隔帧识别；
    // 只针对牌选确认的目标个数超过2个的情况sort_num[1]>2
    if (num_temp > 1)
    {
        for (i = 0; i < num_temp; i++)
        {
            cur_rect = &obj_rect[turn_src[i]].pr_rect;
            for (j = 0; j < obj_pr_last->pr_num; j++)
            {
                if (obj_rect[turn_src[i]].plate_id == obj_rect_last[j].plate_id)
                {
                    last_rect = &obj_rect_last[j].pr_rect;
                    rect_dis  = abs(last_rect->left - cur_rect->left) + abs(last_rect->top - cur_rect->top);

                    if (rect_dis < 16)
                    {
                        obj_rect[turn_src[i]].pr_status = 3;
                    }
                }
            }
        }
    }
    return HIK_MPR_S_OK;
}

/****************************************************************************************************************
* 函  数：MPR_judge_obj_same
* 功  能：判断当前帧目标与前一帧某一目标是否属于同一目标。
非拥堵情况下，只要车道号相同，就是同一目标；而拥堵情况下，需要目标区域有交叉，才认为是同一目标。
* 参  数：
*         obj_last                -I  上一帧的牌识区域参数
*         obj_cur                 -I  当前帧的牌识区域参数
*         same                    -O  同一目标标志
* 返回值：错误状态码
*****************************************************************************************************************/
HRESULT VTS_MPR_judge_obj_same(MPR_OBJ_PR_RECT *obj_last,
                               MPR_OBJ_PR_RECT *obj_cur,
                               int             *same)
{
    int   same_temp = 0;
    short sx1, sy1, sx2, sy2, sx3, sy3, sx4, sy4;           // 临时点坐标

    if (obj_last->lane == obj_cur->lane)
    {
        // 非拥堵情况下，只要车道号相同就认为是统一目标；拥堵情况下需要判断目标框相交

        sx1 = obj_last->pr_rect.left;
        sy1 = obj_last->pr_rect.top;
        sx2 = obj_last->pr_rect.right;
        sy2 = obj_last->pr_rect.bottom;

        sx3 = obj_cur->pr_rect.left;
        sy3 = obj_cur->pr_rect.top;
        sx4 = obj_cur->pr_rect.right;
        sy4 = obj_cur->pr_rect.bottom;

        // 矩形框相交
        if ((abs((sx1 + sx2) / 2 - (sx3 + sx4) / 2) <= (abs(sx1 - sx2) + abs(sx3 - sx4)) / 2)
            && (abs((sy1 + sy2) / 2 - (sy3 + sy4) / 2) <= (abs(sy1 - sy2) + abs(sy3 - sy4)) / 2))
        {
            same_temp = 1;
        }
    }

    *same = same_temp;
    return HIK_MPR_S_OK;
}

/****************************************************************************************************************
* 函  数：MPR_pr_sort_cost_time
* 功  能：对上一帧没有牌选结果但识别为有车牌的目标区域按耗时情况进行排序，耗时小的优先。
* 参  数：
*         obj_pr_last            -I  上一帧的牌识区域参数
*         obj_pr_cur             -I  牌识目标区域
*         pr_turn                -O  当前模块排序后的顺序
*         sort_num               -O  当前模块参与排序的目标数
*         sort_flag              -O  已排序的目标编号标记
* 返回值：错误状态码
*****************************************************************************************************************/
HRESULT VTS_MPR_pr_sort_cost_time(MPR_OBJ_PR_RECT_ARRAY *obj_pr_last,
                                  MPR_OBJ_PR_RECT_ARRAY *obj_pr_cur,
                                  int                   *pr_turn,
                                  int                   *sort_num,
                                  int                   *sort_flag)
{
    HRESULT ret      = HIK_MPR_S_OK;
    int     num_temp = 0;                              // 当前模块参与排序的目标数
    int     i, j;
    int     same_temp = 0;
    int     turn_src[ITS_MAX_VEH_NUM]    = { 0 };      // 排序前的顺序
    int     turn_sorted[ITS_MAX_VEH_NUM] = { 0 };      // 排序后的顺序
    int     turn_temp = 0;

    CHECK_ERROR((NULL == obj_pr_last || NULL == obj_pr_cur), HIK_MPR_PS_PTR_NULL);

    // 找出上一帧有车牌的目标区域，并进行标记
    for (i = 0; i < obj_pr_cur->pr_num; i++)
    {
        if(sort_flag[i] == 1 )
        {
            continue;
        }

        same_temp = 0;
        if (obj_pr_cur->obj_rect[i].plate_id <= 0)
        {
            // 在前一帧的目标中找出对应的目标
            for (j = 0; j < obj_pr_last->pr_num; j++)
            {
                // 判断2个目标区域是否是统一目标对应的区域
                ret = VTS_MPR_judge_obj_same(&obj_pr_last->obj_rect[j], &obj_pr_cur->obj_rect[i], &same_temp);
                CHECK_ERROR(ret != HIK_MPR_S_OK, ret);
                if (same_temp)
                {
                    break;
                }
            }
            if (same_temp && obj_pr_last->obj_rect[j].pr_ok)
            {
                sort_flag[i]       = 1;
                turn_src[num_temp] = i;
                obj_pr_cur->obj_rect[i].cost_time = obj_pr_last->obj_rect[j].cost_time;
                num_temp++;
            }
        }
    }

    // 单个目标，直接插入；多个目标，需要根据上一帧的耗时进行排序
    if (num_temp == 1)
    {
        turn_sorted[0] = turn_src[0];
    }
    else if (num_temp > 1)
    {
        for (i = 0; i < num_temp - 1; i++)
        {
            for (j = i + 1; j < num_temp; j++)
            {
                if (obj_pr_cur->obj_rect[turn_src[i]].cost_time > obj_pr_cur->obj_rect[turn_src[j]].cost_time)
                {
                    turn_temp   = turn_src[i];
                    turn_src[i] = turn_src[j];
                    turn_src[j] = turn_temp;
                }
            }
        }
        memcpy(turn_sorted, turn_src, sizeof(TSint32) * num_temp);
    }
    else
    {
        // do nothing;
    }
    memcpy(pr_turn, turn_sorted, sizeof(TSint32) * num_temp);
    *sort_num = num_temp;

    return HIK_MPR_S_OK;
}

/****************************************************************************************************************
* 函  数：MPR_pr_sort_no_plate
* 功  能：对于上一帧识别为无车牌的或新出现的目标区域按耗时情况进行排序，耗时小的优先。
* 参  数：
*         obj_pr_last            -I  上一帧的牌识区域参数
*         obj_pr_cur             -I  牌识目标区域
*         pr_turn                -O  当前模块排序后的顺序
*         sort_num               -O  当前模块参与排序的目标数
*         sort_flag              -O  已排序的目标编号标记
* 返回值：错误状态码
*****************************************************************************************************************/
HRESULT VTS_MPR_pr_sort_no_plate(MPR_OBJ_PR_RECT_ARRAY *obj_pr_last,
                                 MPR_OBJ_PR_RECT_ARRAY *obj_pr_cur,
                                 int                   *pr_turn,
                                 int                   *sort_num,
                                 int                   *sort_flag)
{
    HRESULT ret      = HIK_MPR_S_OK;
    int     num_temp = 0;                         // 当前模块参与排序的目标数
    int     i, j;
    int     same_temp = 0;
    int     turn_src[ITS_MAX_VEH_NUM]    = { 0 }; // 排序前的顺序
    int     turn_sorted[ITS_MAX_VEH_NUM] = { 0 }; // 排序后的顺序
    int     turn_temp = 0;

    CHECK_ERROR((NULL == obj_pr_last || NULL == obj_pr_cur), HIK_MPR_PS_PTR_NULL);

    // 找出未标记的目标区域，并进行标记
    for (i = 0; i < obj_pr_cur->pr_num; i++)
    {
        if (sort_flag[i] == 0 && obj_pr_cur->obj_rect[i].rect_type >= 3)
        {
            // 在前一帧的目标中找出对应的目标
            same_temp = 0;
            for (j = 0; j < obj_pr_last->pr_num; j++)
            {
                //只挑选上一帧
                if (obj_pr_last->obj_rect[j].pr_ok == 1)
                {
                    continue;
                }
                // 判断2个目标区域是否是同一目标对应的区域
                ret = VTS_MPR_judge_obj_same(&obj_pr_last->obj_rect[j], &obj_pr_cur->obj_rect[i], &same_temp);
                CHECK_ERROR(ret != HIK_MPR_S_OK, ret);
                if (same_temp)
                {
                    break;
                }
            }
            if (same_temp)
            {
                obj_pr_cur->obj_rect[i].cost_time = obj_pr_last->obj_rect[j].cost_time;
                obj_pr_cur->obj_rect[i].direct    = 3;
                if (obj_pr_last->obj_rect[j].direct == 3)
                {
                    obj_pr_cur->obj_rect[i].direct = 2;
                }
            }
            else
            {
                obj_pr_cur->obj_rect[i].cost_time = 0;
            }
            sort_flag[i]       = 1;
            turn_src[num_temp] = i;
            num_temp++;
            if (num_temp >= 8)
            {
                break;
            }
        }
    }

    // 单个目标，直接插入；多个目标，需要根据上一帧的耗时进行排序
    if (num_temp == 1)
    {
        turn_sorted[0] = turn_src[0];
    }
    else if (num_temp > 1)
    {
        for (i = 0; i < num_temp - 1; i++)
        {
            for (j = i + 1; j < num_temp; j++)
            {
                if (obj_pr_cur->obj_rect[turn_src[i]].cost_time > obj_pr_cur->obj_rect[turn_src[j]].cost_time)
                {
                    turn_temp   = turn_src[i];
                    turn_src[i] = turn_src[j];
                    turn_src[j] = turn_temp;
                }
            }
        }
        memcpy(turn_sorted, turn_src, sizeof(TSint32) * num_temp);
    }
    else
    {
        // do nothing;
    }
    memcpy(pr_turn, turn_sorted, sizeof(TSint32) * num_temp);
    *sort_num = num_temp;
    return HIK_MPR_S_OK;
}


/****************************************************************************************************************
* 函  数：MPR_pr_sort_cost_time
* 功  能：对上一帧没有牌选结果但识别为有车牌的目标区域按耗时情况进行排序，耗时小的优先。
* 参  数：
*         obj_pr_last            -I  上一帧的牌识区域参数
*         obj_pr_cur             -I  牌识目标区域
*         pr_turn                -O  当前模块排序后的顺序
*         sort_num               -O  当前模块参与排序的目标数
*         sort_flag              -O  已排序的目标编号标记
* 返回值：错误状态码
*****************************************************************************************************************/
HRESULT VTS_MPR_pr_sort_boost(MPR_OBJ_PR_RECT_ARRAY *obj_pr_last,
                              MPR_OBJ_PR_RECT_ARRAY *obj_pr_cur,
                              int                   *pr_turn,
                              int                   *sort_num,
                              int                   *sort_flag)
{
    HRESULT ret = HIK_MPR_S_OK;
    int     num_temp = 0;                              // 当前模块参与排序的目标数
    int     i=0, j=0;
    int     same_temp = 0;
    int     turn_src[ITS_MAX_VEH_NUM] = { 0 };      // 排序前的顺序
    int     turn_sorted[ITS_MAX_VEH_NUM] = { 0 };      // 排序后的顺序
    int     turn_temp = 0;

    CHECK_ERROR((NULL == obj_pr_last || NULL == obj_pr_cur), HIK_MPR_PS_PTR_NULL);

    // 找出上一帧有车牌的目标区域，并进行标记
    for (i = 0; i < obj_pr_cur->pr_num; i++)
    {
        if (sort_flag[i] == 1)
        {
            continue;
        }
        if (obj_pr_cur->obj_rect[i].rect_type == 5)
        {
            sort_flag[i] = 1;
            turn_src[num_temp] = i;
            num_temp++;         
        }
    }

    // 单个目标，直接插入；多个目标，需要根据上一帧的耗时进行排序
    if (num_temp == 1)
    {
        turn_sorted[0] = turn_src[0];
    }
    else if (num_temp > 1)
    {
        for (i = 0; i < num_temp - 1; i++)
        {
            for (j = i + 1; j < num_temp; j++)
            {

                turn_temp = turn_src[i];
                turn_src[i] = turn_src[j];
                turn_src[j] = turn_temp;

            }
        }
        memcpy(turn_sorted, turn_src, sizeof(TSint32)* num_temp);
    }
    else
    {
        // do nothing;
    }
    memcpy(pr_turn, turn_sorted, sizeof(TSint32)* num_temp);
    *sort_num = num_temp;

    return HIK_MPR_S_OK;
}



/****************************************************************************************************************
* 函  数：MPR_rect_pr_sort
* 功  能：对所有目标牌识区域的调用顺序进行排序。
* 参  数：
*         obj_pr_last             -I     上一帧的牌识区域参数
*         obj_pr_cur              -I/O   牌识目标区域
* 返回值：错误状态码
*****************************************************************************************************************/
HRESULT VTS_MPR_rect_pr_sort(MPR_OBJ_PR_RECT_ARRAY *pr_rect_last,
    MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur)
{
    HRESULT ret = HIK_MPR_S_OK;
    int     i=0, j=0;
    int     pr_order[ITS_MAX_VEH_NUM]         = { 0 };                               // 各目标区域的牌识顺序,汇总的
    int     pr_turn_speed[ITS_MAX_VEH_NUM]    = { 0 };                               // 各目标区域的牌识顺序,有ID的
    int     pr_turn_pr_boost[ITS_MAX_VEH_NUM] = { 0 };
    int     pr_turn_pr_ok[ITS_MAX_VEH_NUM]    = { 0 };                               // 各目标区域的牌识顺序,有车牌的
    int     pr_turn_no_plate[ITS_MAX_VEH_NUM] = { 0 };                               // 各目标区域的牌识顺序,无车牌的
    int     sort_flag_temp[ITS_MAX_VEH_NUM]   = { 0 };                               // 对应目标已排序标志，1-已排序，0-未排序
    int     sort_num[5] = { 0 };                                                     // 参与排序的目标数，对应4种情况

    CHECK_ERROR((NULL == pr_rect_last || NULL == pr_rect_cur), HIK_MPR_PS_PTR_NULL); // 指针为空
    memset(pr_order, 0,sizeof(int)*ITS_MAX_VEH_NUM);
    memset(pr_turn_speed, 0, sizeof(int)*ITS_MAX_VEH_NUM);
    memset(pr_turn_pr_boost, 0, sizeof(int)*ITS_MAX_VEH_NUM);
    memset(pr_turn_pr_ok, 0, sizeof(int)*ITS_MAX_VEH_NUM);
    memset(pr_turn_no_plate, 0, sizeof(int)*ITS_MAX_VEH_NUM);
    memset(sort_flag_temp, 0, sizeof(int)*ITS_MAX_VEH_NUM);
    memset(sort_num, 0, sizeof(int)* 5);

    // 1. 有抓拍标记::外部告诉MPR此处有目标，优先识别，排第一位
    for (i = 0; i < pr_rect_cur->pr_num; i++)
    {
        if (pr_rect_cur->obj_rect[i].cap_flag)
        {
            pr_order[sort_num[0]] = i;
            sort_flag_temp[i]     = 1;
            sort_num[0]++;
        }
    }

    // 1. 有boost车头检测的
    ret = VTS_MPR_pr_sort_boost(pr_rect_last, pr_rect_cur, pr_turn_pr_boost, &sort_num[1], sort_flag_temp);
    CHECK_ERROR(ret != HIK_MPR_S_OK, ret);
    memcpy(&pr_order[sort_num[0]], pr_turn_pr_boost, sizeof(int)* sort_num[1]);

   // 2.有上一帧牌识结果(PR)::按耗时排序;
     ret = VTS_MPR_pr_sort_cost_time(pr_rect_last, pr_rect_cur, pr_turn_pr_ok, &sort_num[2], sort_flag_temp);
    CHECK_ERROR(ret != HIK_MPR_S_OK, ret);
    memcpy(&pr_order[sort_num[0] + sort_num[1]], pr_turn_pr_ok, sizeof(int)* sort_num[2]);

   // 3.上一帧无牌识结果::按耗时排序
    ret = VTS_MPR_pr_sort_no_plate(pr_rect_last, pr_rect_cur, pr_turn_no_plate, &sort_num[3], sort_flag_temp);
    CHECK_ERROR(ret != HIK_MPR_S_OK, ret);
    memcpy(&pr_order[sort_num[0] + sort_num[1] +sort_num[2]], pr_turn_no_plate, sizeof(int)* sort_num[3]);
    
    // 4. 有历史挑选(PS)车牌车牌的区域::对速度排序
    ret = VTS_MPR_pr_sort_speed(pr_rect_last, pr_rect_cur, pr_turn_speed, &sort_num[4], sort_flag_temp);
    CHECK_ERROR(ret != HIK_MPR_S_OK, ret);
    memcpy(&pr_order[sort_num[0] + sort_num[1] + sort_num[2] + sort_num[3]], pr_turn_speed, sizeof(int)* sort_num[4]);
     
    // 按顺序号对目标牌识区域进行排序
    memset(pr_rect_last, 0, sizeof(MPR_OBJ_PR_RECT_ARRAY));
    for (i = 0; i < pr_rect_cur->pr_num; i++)
    {
        if (i >= ITS_MAX_VEH_NUM)
        {
            break;
        }
        j = pr_order[i];
		if ((pr_rect_cur->obj_rect[j].pr_status != 3) && (pr_rect_last->pr_num < HIK_MPR_MAX_PR_AREA_NUM))
        {
            memcpy(&pr_rect_last->obj_rect[pr_rect_last->pr_num], &pr_rect_cur->obj_rect[j], sizeof(MPR_OBJ_PR_RECT));
            pr_rect_last->pr_num++;
        }
    }
    memcpy(pr_rect_cur, pr_rect_last, sizeof(MPR_OBJ_PR_RECT_ARRAY));



    return HIK_MPR_S_OK;
}

/**************************************************************************************************
* 功 能：计算牌识区域个数和对应的牌识参数
* 参 数：*
*         handle                 - I    牌识部分模块句柄
*         dynamic_pr_rect        - I    动态牌识区域
*         trace_info             - I    跟踪信息
*         obj_pr_cur             - O    牌识区域参数
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
//HRESULT VTS_MPR_pr_rect_from_track(void                  *pr_handle,
//                                   ITS_FRM               *frame,
//                                   MPR_TRACK_UNIT        *trace_info,
//                                   MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur,
//                                   int                    pr_up_line)
//{
//    int              i;
//    MPR_PR_HANDLE   *mpr_pr_hdl = (MPR_PR_HANDLE *)pr_handle;      // 获取当前句柄
//    MPR_OBJ_PR_RECT *obj_rect   = NULL;                            // 牌识区域目标框
//    MPR_TRACK_UNIT  *tr_msg     = NULL;
//    int              img_w, img_h;
//    VCA_BOX_S       *trace_rect;
//    VCA_BOX_S       *cur_rect = NULL;
//
//    // 参数校验
//    CHECK_ERROR((NULL == mpr_pr_hdl || NULL == frame), HIK_MPR_PS_PTR_NULL);
//    CHECK_ERROR((NULL == trace_info), HIK_MPR_PS_PTR_NULL);
//
//    tr_msg = trace_info;
//    img_w  = frame->frm_data.image_w;
//    img_h  = frame->frm_data.image_h;
//
//    // 1、跟踪牌识区域；
//    for (i = 0; i < VCA_MAX_OBJ_NUM; i++)
//    {
//        // 跟踪目标有效且未超过牌识区域上边界
//        if ((tr_msg[i].ms_process_info.track_valid == VCA_TR_ON)
//            && (tr_msg[i].rect.bottom > pr_up_line) && (pr_rect_cur->pr_num < ITS_MAX_VEH_NUM - 1))
//        {
//            obj_rect   = &pr_rect_cur->obj_rect[pr_rect_cur->pr_num];
//            trace_rect = &tr_msg[i].rect;
//
//            obj_rect->lane      = 0;
//            obj_rect->rect_type = 2;      // 跟踪区域类型
//
//            obj_rect->pr_rect.left   = max(trace_rect->left - 120, 0);
//            obj_rect->pr_rect.right  = min(trace_rect->right + 120, (short)img_w - 1);
//            obj_rect->pr_rect.top    = max(trace_rect->top - 70, pr_up_line);
//            obj_rect->pr_rect.bottom = min(trace_rect->bottom + 70, (short)img_h - 1);
//
//
//            obj_rect->plate_id  = tr_msg[i].obj_id;
//            obj_rect->no_pr_num = 0;
//
//            memcpy((void *)(pr_rect_cur->obj_rect[pr_rect_cur->pr_num].pr_result),
//                (const void *)(tr_msg[i].pr_result), 32);
//
//            //共享数据赋值
//            memcpy(&obj_rect->pr_share_info.pre_seg_rect, trace_rect, sizeof(PR_RECT));
//
//            pr_rect_cur->pr_num++;
//        }
//    }
//
//    return HIK_MPR_S_OK;
//}

/**************************************************************************************************
* 功 能：判断当前车道号
* 参 数：*
*         handle                 - I    牌识部分模块句柄
*         dynamic_pr_rect        - I    动态牌识区域
*         frame                  - I    图像数据
*         obj_pr_cur             - O    牌识区域参数
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
void VTS_MPR_rect_num_decide(MPR_PR_HANDLE   *pr_handle,
                             MPR_OBJ_PR_RECT *obj_rect)
{
    short x = (obj_rect->pr_rect.left + obj_rect->pr_rect.right) / 2;

    if (pr_handle->x_cross_m > 0)
    {
        if (x < pr_handle->x_cross_m)
        {
            obj_rect->lane = 0;
        }

        else
        {
            obj_rect->lane = 1;
        }
    }
}

/**************************************************************************************************
* 功 能：从历史车牌获取预测牌识区域
* 参 数：*
*         handle                 - I    牌识部分模块句柄
*         dynamic_pr_rect        - I    动态牌识区域
*         frame                  - I    图像数据
*         obj_pr_cur             - O    牌识区域参数
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
HRESULT VTS_MPR_pr_rect_from_old_plate(void                  *pr_handle,
									   VCA_FRAME_HEADER      *frame_header,
									   int					  img_w,
									   int					  img_h,	
                                       MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur,
                                       MPR_MODULE_INTERFACE  *module_interface,
                                       int                   *flag_super_big)
{
    HRESULT             ret = HIK_MPR_S_OK;
    int                 i, j, k, m;
    MPR_PR_HANDLE      *mpr_pr_hdl          = (MPR_PR_HANDLE *)pr_handle;             // 获取当前句柄
    MPR_PS_PLATE_QUEUE *ps_history          = NULL;
    MPR_PLATE_INFO     *plate_cur           = NULL;
    int                 same_id             = 0;
    int                 time_dis            = 0, time_dis_2 = 0;
    unsigned int        cnt_zero_time_stamp = 0;
    unsigned int        cnt_zero_time_dis   = 0;
    int                 frame_dis           = 0;
    int                 pr_up_line;
    int                 count = 0;
    //int                 img_w, img_h;
    short               rect_ct_y = 0, his_rect_ct_y = 0;
    int                 pr_r_wdt_lst = 0, pr_r_hgt_lst = 0, pr_r_dir_lst = 0;

	CHECK_ERROR((NULL == mpr_pr_hdl || NULL == frame_header), HIK_MPR_PS_PTR_NULL);
    CHECK_ERROR((NULL == pr_rect_cur || NULL == module_interface), HIK_MPR_PS_PTR_NULL);

    //img_w      = frame->frm_data.image_w;
    //img_h      = frame->frm_data.image_h;
    pr_up_line = module_interface->pr_up_line;

    // 2、历史车牌预测；如果对应车牌目标已在跟踪牌识区域中存在，则丢弃该目标；
    for (i = 0; i < ITS_MAX_LANE_NUM; i++)
    {
        ps_history = &module_interface->ps_plate_queue[i];
        if (!VTS_MPR_trig_queue_empty(ps_history))
        {
            count = 0;
            j     = (ps_history->back - 1 + MPR_PS_QUEUE_CAP) % MPR_PS_QUEUE_CAP;
            while (j != ps_history->back && count < ps_history->count)
            {
                count++;
                plate_cur = &ps_history->ps_plate_array[j].latest_res_for_cur_plt;
                pr_r_wdt_lst = plate_cur->pr_info.segment_rect.rx - plate_cur->pr_info.segment_rect.lx;
                pr_r_hgt_lst = plate_cur->pr_info.segment_rect.ty - plate_cur->pr_info.segment_rect.by;
                pr_r_dir_lst = plate_cur->ps_info.direction;

                if ((module_interface->product_type == 4)&&(pr_r_dir_lst == 0) && (pr_r_wdt_lst>160))
                {
                    *flag_super_big = 1;
                }

                // 存在跟踪目标区域，则不进行历史车牌预测
                same_id = 0;
                for (k = 0; k < pr_rect_cur->pr_num; k++)
                {
                    if (pr_rect_cur->obj_rect[k].plate_id == plate_cur->obj_id)
                    {
                        same_id = 1;
                        // 利用历史车牌的运动方向给目标块运动方向赋值
                        pr_rect_cur->obj_rect[k].direct = plate_cur->ps_info.direction;
                        rect_ct_y     = (pr_rect_cur->obj_rect[k].pr_rect.top + pr_rect_cur->obj_rect[k].pr_rect.bottom) / 2;
                        his_rect_ct_y = (plate_cur->ps_info.first_rect.ty + plate_cur->ps_info.first_rect.by) / 2;
                        time_dis_2 = (plate_cur->frm_info.time_stamp - plate_cur->ps_info.first_frm_info.time_stamp);

                        // 方向未知，计算方向
                        if (pr_rect_cur->obj_rect[k].direct == 2)
                        {
                            // 当前识别位置y > 首次出现y， 正向
                            if (rect_ct_y - his_rect_ct_y >= 0)
                            {
                                pr_rect_cur->obj_rect[k].direct = 0;
                            }
                            else
                            {
                                pr_rect_cur->obj_rect[k].direct = 1;
                            }
                        }
                        /*if (plate_cur->ps_info.pixel_vy == 0)
                        {
                            for (m = 0; m < ITS_MAX_VEH_NUM; m++)
                            {
                                if (trace_info[i].obj_id == plate_cur->obj_id)
                                {
                                    if (trace_info[i].ms_process_info.track_valid == VCA_TR_ON)
                                    {
                                        if (time_dis_2 == 0)
                                        {
                                            plate_cur->ps_info.pixel_vy = 0;
                                            break;
                                        }
                                        plate_cur->ps_info.pixel_vy = (rect_ct_y - his_rect_ct_y) * 100 / time_dis_2;
                                        plate_cur->ps_info.pixel_vy = min(plate_cur->ps_info.pixel_vy, 100);
                                        plate_cur->ps_info.pixel_vy = max(plate_cur->ps_info.pixel_vy, -100);
                                    }
                                    break;
                                }
                            }
                        }*/
                        pr_rect_cur->obj_rect[k].use_init_rect = 0;
                        pr_rect_cur->obj_rect[k].lane          = plate_cur->lane;
                        VTS_MPR_rect_num_decide(pr_handle, &pr_rect_cur->obj_rect[k]);

                        break;
                    }
                }

                // 单帧挑选结果不进行预测
                if ((plate_cur->ps_info.sig_select_flag == 1)
                    || (same_id == 1))
                {
                    j = (j - 1 + MPR_PS_QUEUE_CAP) % MPR_PS_QUEUE_CAP;
                    continue;
                }

                // 跟踪块中找不到的情况下，采用预测
				if (frame_header->time_stamp >= plate_cur->frm_info.time_stamp)
                {
					frame_dis = frame_header->frame_num - plate_cur->frm_info.frm_num;
					time_dis = frame_header->time_stamp - plate_cur->frm_info.time_stamp;

#if _WIN32
                    // 时间戳跳变
                    if (time_dis * mpr_pr_hdl->frm_rate > 1000 * frame_dis)
                    {
                        time_dis = 1000 * frame_dis / mpr_pr_hdl->frm_rate;
                    }
#endif
                    // 时间戳校验
                    if ((module_interface->proccessed_frm) <= 100)
                    {
						if (frame_header->time_stamp <= 0)
                        {
                            module_interface->cnt_zero_time_stamp++;
                        }

                        if (time_dis <= 0)
                        {
                            module_interface->cnt_zero_time_dis++;
                        }

                        if ((module_interface->cnt_zero_time_stamp > 40)
                            && (module_interface->cnt_zero_time_dis > 30))
                        {
                            return HIK_MPR_PR_S_FAIL;
                        }
                    }

                    if (time_dis <= 400)
                    {
                        // 进行历史车牌预测
                        ret = VTS_MPR_get_pre_rect(&ps_history->ps_plate_array[j],
							&mpr_pr_hdl->init_pr_rect,
                            time_dis,
                            frame_dis,
                            img_w,
                            img_h,
                            pr_rect_cur);
                        CHECK_ERROR(ret != HIK_MPR_S_OK, ret);
                    }
                }

                j = (j + MPR_PS_QUEUE_CAP - 1) % MPR_PS_QUEUE_CAP;
            }
        }
    }

    return HIK_MPR_S_OK;
}

/***************************************************************************************
* 功 能：改写图像数据，叠加跟踪框，用于调试
* 参 数：data_y      - I/O    图像Y通道指针
*        data_u      - I/O    图像U通道指针
*        image_w     - I      图像宽度
*        image_h     - I      图像高度
*        prg         - I      跟踪句柄
* 返回值：无
* 备 注：
********************************************************************************************/
void VTS_MPR_show_cutted_pr_rect(unsigned char          *data_y,
                                 unsigned char          *data_u,
                                 unsigned short          image_w,
                                 unsigned short          image_h,
                                 MPR_OBJ_PR_RECT_ARRAY  *pr_rect_cur)
{
    int             i=0, m=0, n=0;
    unsigned char   y=0, u=0, v=0;
    unsigned int    offset_y=0, offset_uv=0;
    VCA_BOX_S       draw_rect = { 0 };
    int             line_thickness_tr=0;

    line_thickness_tr = 4;

    // 叠加跟踪库
    for (i = 0; i < pr_rect_cur->pr_num; i++)
    {
        if ((pr_rect_cur->obj_rect[i].pr_rect.bottom == 0)
            && (pr_rect_cur->obj_rect[i].pr_rect.top == 0)
            && (pr_rect_cur->obj_rect[i].pr_rect.left == 0)
            && (pr_rect_cur->obj_rect[i].pr_rect.right == 0))
        {
            continue;
        }

       /* if ((pr_rect_cur->obj_rect[i].pr_rect.bottom >= image_h - line_thickness_tr)
            || (pr_rect_cur->obj_rect[i].pr_rect.top >= image_h - line_thickness_tr)
            || (pr_rect_cur->obj_rect[i].pr_rect.left >= image_w - line_thickness_tr)
            || (pr_rect_cur->obj_rect[i].pr_rect.right >= image_w - line_thickness_tr))
        {
            continue;
        }*/

        if (pr_rect_cur->obj_rect[i].use_init_rect == 1)
        {
            y = 200;        // mint
            u = 53;
            v = 34;
        }
        else if (pr_rect_cur->obj_rect[i].use_init_rect == 0)
        {
            y = 200;       // burgandy
            u = 239;
            v = 239;
        }
        draw_rect = pr_rect_cur->obj_rect[i].pr_rect;
		
		draw_rect.top = (draw_rect.top < 4) ? 4 : draw_rect.top;
		draw_rect.left = (draw_rect.left < 4) ? 4 : draw_rect.left;
		draw_rect.right = (draw_rect.right >image_w - 4) ? image_w - 4 : draw_rect.right;
		draw_rect.bottom = (draw_rect.bottom >image_h - 4) ? image_h - 4 : draw_rect.bottom;

        // 上横
        m = ((draw_rect.top > line_thickness_tr) ? (draw_rect.top - line_thickness_tr) : 0);
        for (; m < (draw_rect.top); m++)
        {
            // 依据图像格式解析
            offset_y = image_w * m;
            offset_uv = image_w * (m / 2);
            for (n = draw_rect.left; n < draw_rect.right; n++)
            {
                data_y[n + offset_y] = y;
                data_u[(n + offset_uv) & 0xFFFFFFF0] = u;
                data_u[((n + offset_uv) & 0xFFFFFFFE) + 1] = v;
            }
        }
        // 下横
        m = ((draw_rect.bottom > 0) ? (draw_rect.bottom) : 0);
        for (; m < draw_rect.bottom + line_thickness_tr; m++)
        {
            // 依据图像格式解析
            offset_y = image_w * m;
            offset_uv = image_w * (m / 2);
            for (n = draw_rect.left; n < draw_rect.right; n++)
            {
                data_y[n + offset_y] = y;
                data_u[(n + offset_uv) & 0xFFFFFFF0] = u;
                data_u[((n + offset_uv) & 0xFFFFFFFE) + 1] = v;
            }
        }
        // 左竖
        m = ((draw_rect.top > line_thickness_tr) ? (draw_rect.top - line_thickness_tr) : 0);
        for (; m < draw_rect.bottom + line_thickness_tr; m++)
        {
            // 依据图像格式解析
            offset_y = image_w * m;
            offset_uv = image_w * (m / 2);
            n = ((draw_rect.left < line_thickness_tr) ? 0 : draw_rect.left - line_thickness_tr);
            for (n = draw_rect.left - line_thickness_tr; n < (draw_rect.left); n++)
            {
                data_y[n + offset_y] = y;
                data_u[(n + offset_uv) & 0xFFFFFFF0] = u;
                data_u[((n + offset_uv) & 0xFFFFFFFE) + 1] = v;
            }
        }
        // 右竖
        m = ((draw_rect.top > line_thickness_tr) ? (draw_rect.top - line_thickness_tr) : 0);
        for (;
            m < draw_rect.bottom + line_thickness_tr;
            m++)
        {
            // 依据图像格式解析
            offset_y = image_w * m;
            offset_uv = image_w * (m / 2);
            n = ((draw_rect.right + line_thickness_tr > image_w) ? image_w : draw_rect.right + line_thickness_tr);
            for (n = draw_rect.right; n < draw_rect.right + line_thickness_tr; n++)
            {
                data_y[n + offset_y] = y;
                data_u[(n + offset_uv) & 0xFFFFFFF0] = u;
                data_u[((n + offset_uv) & 0xFFFFFFFE) + 1] = v;
            }
        }
    }

}

/****************************************************************************************************************
* 函  数：VTS_MPR_box_cross
* 功  能：判断两矩形框是否相交。。
* 参  数：
*         rect1,rect2              -I    待比较的矩形框
* 返回值：相交标志
*****************************************************************************************************************/
int VTS_MPR_box_cross(VCA_BOX_S  rect1, VCA_BOX_S  rect2)
{
    short      sx1, sy1, sx2, sy2, sx3, sy3, sx4, sy4;           // 临时点坐标
    int        cross_flg = 0;

    sx1 = rect1.left;
    sy1 = rect1.top;
    sx2 = rect1.right;
    sy2 = rect1.bottom;
    sx3 = rect2.left;
    sy3 = rect2.top;
    sx4 = rect2.right;
    sy4 = rect2.bottom;

    // 矩形框相交：：匹配
    if ((abs((sx1 + sx2) / 2 - (sx3 + sx4) / 2) <= (abs(sx1 - sx2) + abs(sx3 - sx4)) / 2)
        && (abs((sy1 + sy2) / 2 - (sy3 + sy4) / 2) <= (abs(sy1 - sy2) + abs(sy3 - sy4)) / 2))
    {
        cross_flg = 1;
    }

    return cross_flg;
}

/****************************************************************************************************************
* 函  数：VTS_MPR_check_mod_data
* 功  能：对所有mod数据进行校验，将包含车牌跟踪预测块的目标置无效。
* 参  数：
*         obj_pr_cur              -I   牌识目标区域
*         img_w,img_h             -I   宽高
*         obj_list              -I/O boost信息
* 返回值：错误状态码
*****************************************************************************************************************/
void VTS_MPR_check_mod_data(void                    *pr_handle, 
                              MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur,
                              unsigned int           img_w,
                              unsigned int           img_h,
                              int                    pr_up_line,
                              VCA_OBJ_LIST          *det_obj_list)
{
    int             i = 0, j = 0;
    int             pr_num = pr_rect_cur->pr_num;
    VCA_BOX_S       det_obj_rect;                                       //目标牌识区域
	short			obj_width, obj_height;
    MPR_PR_HANDLE  *mpr_pr_hdl = (MPR_PR_HANDLE *)pr_handle;			//获取当前句柄
    int            *lane_abc = &mpr_pr_hdl->lane_abc[3];
    short           obj_cx, obj_cy;
    int             cy_cross;
	int				top_offset = 0, bottom_offset = 0, obj_cut_dis = 0, x_offset = 0;
	int             plate_w_ave = 0;

	plate_w_ave = mpr_pr_hdl->plate_w_ave;
	PRTF("plate_w_ave %d\n",plate_w_ave);
	PRTF("det_obj_list->obj_num %d\n",det_obj_list->obj_num);
	//没有检测目标的情况下，取消前面预测区域
	if (det_obj_list->obj_num == 0)
	{
		//pr_rect_cur->pr_num = 0;
		return;
	}
	for (j = 0; j < det_obj_list->obj_num; j++)
    {
		VTS_MPR_convert_f2s(&det_obj_list->obj[j].rect, img_w, img_h, &det_obj_rect);
		
		obj_width = det_obj_rect.right - det_obj_rect.left;
		obj_height = det_obj_rect.bottom - det_obj_rect.top;
		PRTF("obj_width %d obj_height %d\n",obj_width,obj_height);

		///检测框宽高过小，则过滤掉
		//if (obj_width <= 300 || obj_height <= 200 || obj_height <= obj_width/3)
		if (obj_width <= plate_w_ave * 3 || obj_height <= plate_w_ave * 2)
		{
			det_obj_list->obj[j].valid = 0;
			continue;
		}

		//针对MA检测，对检测框进行调整，抑制框不到车牌的情况
		obj_cut_dis = *((int *)det_obj_list->obj[j].reserved);
		if (mpr_pr_hdl->pr_param->dwRecogMode == LPR_RECOG_HVT)
		{
			top_offset = obj_width / 3;
			bottom_offset = obj_width / 4;
			if (obj_cut_dis > 0)
			{
				top_offset -= obj_cut_dis;
				top_offset = top_offset >= 0 ? top_offset : 0;
			}
		}
		else if (mpr_pr_hdl->pr_param->dwRecogMode == LPR_RECOG_VET)
		{
			top_offset = obj_height / 5;
			bottom_offset = obj_height / 4;
			x_offset = obj_width / 10;
		}
		else
		{
			return;
		}
		
 // 非二次识别下，需要调整； 二次识别关掉，不调整
        if (img_w > MAX_VET_WIDTH || img_h > MAX_VET_HEIGHT)
        {
            det_obj_rect.top += top_offset;


            det_obj_rect.bottom += bottom_offset;
            det_obj_rect.bottom = min(det_obj_rect.bottom, mpr_pr_hdl->init_pr_rect.bottom - 1);


			//x方向扩展区域
			if (mpr_pr_hdl->dir_flag == 0)
			{
				det_obj_rect.left -= x_offset;
			}
			else if (mpr_pr_hdl->dir_flag == 1)
			{
			 det_obj_rect.right += x_offset;
			}
			else if (mpr_pr_hdl->dir_flag == 2)
			{
			 det_obj_rect.left -= x_offset;
			 det_obj_rect.right += x_offset;
			}

			det_obj_rect.left = max(mpr_pr_hdl->init_pr_rect.left, det_obj_rect.left);
			det_obj_rect.right = min(mpr_pr_hdl->init_pr_rect.right - 1, det_obj_rect.right);     
        }
		
        
        for (i = 0; i < pr_rect_cur->pr_num; i++)
        {
            // 矩形框相交：：匹配
			if (VTS_MPR_box_cross(det_obj_rect, pr_rect_cur->obj_rect[i].pr_rect))
            {
				det_obj_list->obj[j].valid = 0;
                break;
            }
        }

        //对有效的boost目标，加入到牌识区域目标中，并记录车道号
		if (det_obj_list->obj[j].valid && pr_num < ITS_MAX_VEH_NUM)
        {
            pr_rect_cur->obj_rect[pr_num].boost_flag = 1;
            pr_rect_cur->obj_rect[pr_num].direct = 3;
			pr_rect_cur->obj_rect[pr_num].pr_rect = det_obj_rect;
            pr_rect_cur->obj_rect[pr_num].use_init_rect = 0;
            pr_rect_cur->obj_rect[pr_num].rect_type = 5;
			obj_cx = (det_obj_rect.left + det_obj_rect.right) / 2;
			obj_cy = (det_obj_rect.top + 3 * det_obj_rect.bottom) / 4;

            if (mpr_pr_hdl->lane_num == 1 || lane_abc[0] == 0)
            {
                pr_rect_cur->obj_rect[pr_num].lane = 0;
            }
            else
            {
                cy_cross = -(lane_abc[1] * obj_cy + lane_abc[2]) / lane_abc[0];
                if (obj_cx <= cy_cross)
                {
                    pr_rect_cur->obj_rect[pr_num].lane = 0;
                }
                else
                {
                    pr_rect_cur->obj_rect[pr_num].lane = 1;
                }
            }
            pr_num++;
        }
    }

    pr_rect_cur->pr_num = pr_num;
}

/****************************************************************************************************************
* 函  数：VTS_MPR_check_pr_rect_valide
* 功  能：对切分的牌识区域进行有效性验证，不符合要求的删除：
*          1、宽度、高度太小；2、在左右车道线以外。
* 参  数：
*         pr_rect                 -I   牌识目标区域
*         plate_w_ave             -I   平均车牌宽度
*         lane_abc                -I   车道线参数
*         lane_num                -I   车道总数
* 返回值：错误状态码
*****************************************************************************************************************/
int VTS_MPR_check_pr_rect_valide(VCA_BOX_S  *pr_rect,
                                int         plate_w_ave,
                                int        *lane_abc,
                                int         lane_num)
{
    int   valid = 1;
    short rect_cx, rect_cy;
    short rect_w, rect_h;
    int   cross_x[2] = {0,0};
    int   *right_abc = NULL;

    rect_w = pr_rect->right - pr_rect->left;
    rect_h = pr_rect->bottom - pr_rect->top;
    if ((rect_w < plate_w_ave * 3 / 2) || (rect_h < plate_w_ave / 4))
    {
        valid = 0;
        return valid;
    }

    if (lane_num = 1)
    {
        right_abc = &lane_abc[3];
    }
    else
    {
        right_abc = &lane_abc[6];
    }

    rect_cx = (pr_rect->right + pr_rect->left) / 2;
    rect_cy = pr_rect->bottom;

    if (lane_abc[0] != 0)
    {
        cross_x[0] = -(lane_abc[1] * rect_cy + lane_abc[2]) / lane_abc[0];
    }
    else
    {
        cross_x[0] = rect_cx;
    }

    if (cross_x[0] > rect_cx && rect_h < plate_w_ave * 3 / 2)
    {
        valid = 0;
        return valid;
    }

    if (right_abc[0] != 0)
    {
        cross_x[1] = -(right_abc[1] * rect_cy + right_abc[2]) / right_abc[0];
    }
    else
    {
        cross_x[1] = rect_cx;
    }

    if (cross_x[1] > rect_cx && rect_h < plate_w_ave * 3 / 2)
    {
        valid = 0;
    }

    return valid;
}


/****************************************************************************************************************
* 函  数：VTS_MPR_check_blob_data
* 功  能：对所有boost数据进行校验，将包含车牌跟踪预测块的目标置无效。 - 
* 参  数：
*         obj_pr_cur              -I   牌识目标区域
*         img_w,img_h             -I   宽高
*         boost_list              -I/O boost信息
* 返回值：错误状态码
*****************************************************************************************************************/
void VTS_MPR_check_blob_data(void                  *pr_handle,
                             MPR_DYNAMIC_RECT      *dynamic_pr_rect,
                             int                    pr_up_line,
                             unsigned int           img_w,
                             unsigned int           img_h, 
                             MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur,
                             unsigned char          is_cap_frm)
{
    int        i = 0, j = 0,k;
    int        blob_type;
    short      blob_cx, blob_cy;
    short      rect_cx[18], rect_cy[18];
    VCA_BOX_S  *rect_tmp,*rect_tmp1,*rect_tmp2;
    VCA_BOX_S  blob_tmp;                                        //目标牌识区域
    VCA_BOX_S  blob_rect[ITS_MAX_VEH_NUM];  
    int        blob_direct[ITS_MAX_VEH_NUM];
    int        rect_index[ITS_MAX_VEH_NUM];
    int        contain_num = 0;
    int        blob_num = 0;
    short      quadrant_tmp[18];
    int        pr_num = pr_rect_cur->pr_num;
    MPR_PR_HANDLE  *mpr_pr_hdl = (MPR_PR_HANDLE *)pr_handle;                       // 获取当前句柄
    int            *lane_abc = &mpr_pr_hdl->lane_abc[3];
    int            plate_w_ave = mpr_pr_hdl->plate_w_ave;

    memset(blob_direct, 3, sizeof(int)*ITS_MAX_VEH_NUM);
    for (k = 0; k < ITS_MAX_VEH_NUM; k++)
    {
        if (dynamic_pr_rect[k].valid)
        {
            VTS_MPR_convert_f2s(&dynamic_pr_rect[k].rect, img_w, img_h, &blob_tmp);
            
            // 曝闪模式下，Blob不与牌识区域上边界关联；LYP，161104
            if (!is_cap_frm)
            {
                blob_tmp.top = max(blob_tmp.top, pr_up_line);
            }
            if (blob_tmp.bottom - blob_tmp.top < 20)
            {
                continue;
            }

            memset(rect_index, -1, sizeof(int)*ITS_MAX_VEH_NUM);
            contain_num = 0;

            //搜索Blob包含的已有的牌识区域块，进行标记
            for (i = 0; i < pr_rect_cur->pr_num; i++)
            {
                if (VTS_MPR_box_cross(blob_tmp, pr_rect_cur->obj_rect[i].pr_rect))
                {
                    rect_index[contain_num] = i;
                    contain_num++;
                }
            }

            //1、确定Blob的结构，上下、左右、上下左右、单个目标区域
            blob_type = 0;
            if (blob_tmp.bottom - blob_tmp.top <= 4 * plate_w_ave
                && blob_tmp.right - blob_tmp.left <= 5 * plate_w_ave)
            {
                blob_type = 0; // 单个
            }
            else if (blob_tmp.bottom - blob_tmp.top > 4 * plate_w_ave
                && blob_tmp.right - blob_tmp.left <= 5 * plate_w_ave)
            {
                blob_type = 1;  //上下
            }
            else if (blob_tmp.bottom - blob_tmp.top <= 4 * plate_w_ave
                && blob_tmp.right - blob_tmp.left > 5 * plate_w_ave)
            {
                blob_type = 2;   // 左右
            }
            else
            {
                blob_type = 3;   // 上下左右,大Blob
            }
            blob_cx = (blob_tmp.right + blob_tmp.left) / 2;
            blob_cy = (blob_tmp.top + blob_tmp.bottom) / 2;

            //2、在包含已有目标区域的情况下，判断Blob与已有目标的位置关系，确定对Blob进行删除或切分操作；
            if (contain_num > 0)
            {
                //获取包含的目标牌识区域在Blob中所处象限
                for (i = 0; i < contain_num; i++)
                {
                    rect_tmp = &pr_rect_cur->obj_rect[rect_index[i]].pr_rect;
                    rect_cx[i] = (rect_tmp->left + rect_tmp->right) / 2;
                    rect_cy[i] = (rect_tmp->top + rect_tmp->bottom) / 2;

                    if (rect_cy[i] <= blob_cy &&  rect_cx[i] <= blob_cx)
                    {
                        pr_rect_cur->obj_rect[rect_index[i]].quadrant = 2;
                    }
                    else if (rect_cy[i] >= blob_cy &&  rect_cx[i] <= blob_cx)
                    {
                        pr_rect_cur->obj_rect[rect_index[i]].quadrant = 3;
                    }
                    else if (rect_cy[i] >= blob_cy &&  rect_cx[i] >= blob_cx)
                    {
                        pr_rect_cur->obj_rect[rect_index[i]].quadrant = 4;
                    }
                    else
                    {
                        pr_rect_cur->obj_rect[rect_index[i]].quadrant = 1;
                    }
                }

                if (contain_num == 1)
                {
                    rect_tmp = &pr_rect_cur->obj_rect[rect_index[contain_num - 1]].pr_rect;
                    quadrant_tmp[0] = pr_rect_cur->obj_rect[rect_index[contain_num - 1]].quadrant;
                    rect_cx[0] = (rect_tmp->left + rect_tmp->right) / 2;
                    rect_cx[1] = (rect_tmp->top + rect_tmp->bottom) / 2;
                    if (blob_type == 0)
                    {
                        continue;
                    }
                    else if (blob_type == 1)
                    {
                        //上下切分
                        if (rect_cy[0] <= blob_cy)
                        {
                            if (rect_tmp->bottom + 4 * plate_w_ave < blob_tmp.bottom)
                            {
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].top = rect_tmp->bottom + 3 * plate_w_ave;
                                blob_num++;
                            }
                        }
                        else
                        {
                            if (rect_tmp->top - 4 * plate_w_ave > blob_tmp.top)
                            {
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].bottom = rect_tmp->top - 3 * plate_w_ave;
                                blob_num++;
                            }
                        }
                    }
                    else if (blob_type == 2)
                    {
                        //左右切分
                        if (rect_cx[0] < blob_cx)
                        {
                            if (rect_tmp->right + 4 * plate_w_ave < blob_tmp.right)
                            {
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].left = rect_tmp->right + 3 * plate_w_ave;
                                blob_num++;
                            }
                        }
                        else
                        {
                            if (rect_tmp->left - 4 * plate_w_ave > blob_tmp.left)
                            {
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].right = rect_tmp->left - 3 * plate_w_ave;
                                blob_num++;
                            }
                        }
                    }
                    else
                    {
                        //暂时只考虑左右切分
                        if (quadrant_tmp[0] == 1 || quadrant_tmp[0] == 4)
                        {
                            if (rect_tmp->left - 4 * plate_w_ave > blob_tmp.left)
                            {
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].right = rect_tmp->left - 3 * plate_w_ave;
                                blob_num++;
                            }
                        }
                        else
                        {
                            if (rect_tmp->right + 4 * plate_w_ave < blob_tmp.right)
                            {
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].left = rect_tmp->right + 3 * plate_w_ave;
                                blob_num++;
                            }
                        }

                    }
                }
                else if (contain_num == 2)
                {
                    if (blob_type < 3)
                    {
                        continue;
                    }
                    else
                    {
                        quadrant_tmp[0] = pr_rect_cur->obj_rect[rect_index[0]].quadrant;
                        quadrant_tmp[1] = pr_rect_cur->obj_rect[rect_index[1]].quadrant;
                        rect_tmp = &pr_rect_cur->obj_rect[rect_index[0]].pr_rect;
                        rect_tmp1 = &pr_rect_cur->obj_rect[rect_index[1]].pr_rect;
                        if (quadrant_tmp[0] + quadrant_tmp[1] == 3)
                        {
                            //切分下面
                            memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                            blob_rect[blob_num].top = max(rect_tmp->bottom, rect_tmp1->bottom) + 3 * plate_w_ave;
                            blob_num++;

                        }
                        else if (quadrant_tmp[0] + quadrant_tmp[1] == 7)
                        {
                            //切分上面
                            memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                            blob_rect[blob_num].bottom = min(rect_tmp->top, rect_tmp1->top) - 3 * plate_w_ave;
                            blob_num++;
                        }
                        else if (quadrant_tmp[0] + quadrant_tmp[1] == 5)
                        {
                            if (abs(quadrant_tmp[0] - quadrant_tmp[1]) == 1)
                            {
                                //切分右面
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].left = max(rect_tmp->right, rect_tmp1->right) + 3 * plate_w_ave;
                                blob_num++;
                            }
                            else
                            {
                                //切分左面
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].right = min(rect_tmp->left, rect_tmp1->left) - 3 * plate_w_ave;
                                blob_num++;
                            }
                        }
                    }

                }
                else if (contain_num == 3)
                {
                    if (blob_type < 3)
                    {
                        continue;
                    }
                    else
                    {
                        quadrant_tmp[0] = pr_rect_cur->obj_rect[rect_index[0]].quadrant;
                        quadrant_tmp[1] = pr_rect_cur->obj_rect[rect_index[1]].quadrant;
                        quadrant_tmp[2] = pr_rect_cur->obj_rect[rect_index[2]].quadrant;
                        rect_tmp = &pr_rect_cur->obj_rect[rect_index[0]].pr_rect;
                        rect_tmp1 = &pr_rect_cur->obj_rect[rect_index[1]].pr_rect;
                        rect_tmp2 = &pr_rect_cur->obj_rect[rect_index[2]].pr_rect;
                        if (quadrant_tmp[0] + quadrant_tmp[1] + quadrant_tmp[2] <= 5)
                        {
                            //切分下面
                            short by_tmp = max(rect_tmp->bottom, rect_tmp1->bottom);
                            by_tmp = max(by_tmp, rect_tmp2->bottom);
                            memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                            blob_rect[blob_num].top = by_tmp + 3 * plate_w_ave;
                            blob_num++;

                        }
                        else if (quadrant_tmp[0] + quadrant_tmp[1] + quadrant_tmp[2] >= 10)
                        {
                            //切分上面
                            short ty_tmp = min(rect_tmp->top, rect_tmp1->top);
                            ty_tmp = min(ty_tmp, rect_tmp2->top);
                            memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                            blob_rect[blob_num].bottom = ty_tmp - 3 * plate_w_ave;
                            blob_num++;
                        }
                        else if (quadrant_tmp[0] + quadrant_tmp[1] + quadrant_tmp[2] == 7
                            || quadrant_tmp[0] + quadrant_tmp[1] + quadrant_tmp[2] == 8)
                        {
                            //切分右面
                            short rx_tmp = max(rect_tmp->right, rect_tmp1->right);
                            short quadrant_mid = mid(quadrant_tmp[0], quadrant_tmp[1], quadrant_tmp[2]);
                            short quadrant_min = min(quadrant_tmp[0], quadrant_tmp[1]);
                            short quadrant_max = max(quadrant_tmp[0], quadrant_tmp[1]);
                            quadrant_min = min(quadrant_min, quadrant_tmp[2]);
                            quadrant_max = max(quadrant_max, quadrant_tmp[2]);
                            if (quadrant_min > 1)
                            {
                                rx_tmp = max(rx_tmp, rect_tmp2->right);
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].left = rx_tmp + 3 * plate_w_ave;
                                blob_num++;
                            }
                        }
                        else
                        {
                            //切分左面
                            short lx_tmp = min(rect_tmp->left, rect_tmp1->left);
                            short quadrant_mid = mid(quadrant_tmp[0], quadrant_tmp[1], quadrant_tmp[2]);
                            short quadrant_min = min(quadrant_tmp[0], quadrant_tmp[1]);
                            short quadrant_max = max(quadrant_tmp[0], quadrant_tmp[1]);
                            quadrant_min = min(quadrant_min, quadrant_tmp[2]);
                            quadrant_max = max(quadrant_max, quadrant_tmp[2]);
                            if (quadrant_min == 1)
                            {
                                lx_tmp = min(lx_tmp, rect_tmp2->left);
                                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                                blob_rect[blob_num].right = lx_tmp - 3 * plate_w_ave;
                                blob_num++;
                            }
                            
                        }
                    }
                }
                else
                {
                    //大于等于4块，不做切分，Blob无效；
                }
            }
            else
            {
                //不包含已有牌识区域的Blob，采用原始Blob
                memcpy(&blob_rect[blob_num], &blob_tmp, sizeof(VCA_BOX_S));
                if (is_cap_frm)
                {
                    if (dynamic_pr_rect[k].move_dis.y > 0.00001f)
                    {
                        blob_direct[blob_num] = 0;            //正向
                    }
                    else if (dynamic_pr_rect[k].move_dis.y < 0.00001f)
                    {
                        blob_direct[blob_num] = 1;            //背向
                    }
                }
                blob_num++;
            }
        }
    }

    // 对切分处理后的Blob，添加进入牌识区域数组;如果某个Blob跨2个车道，则继续切分；
    for (i = 0; i < blob_num; i++)
    {
        if (0 == VTS_MPR_check_pr_rect_valide(&blob_rect[i], plate_w_ave, mpr_pr_hdl->lane_abc, mpr_pr_hdl->lane_num))
        {
            continue;
        }
        if (pr_num < ITS_MAX_VEH_NUM)
        {
            if (mpr_pr_hdl->lane_num == 1 || mpr_pr_hdl->lane_num == 2 && lane_abc[0] == 0)
            {
                //不需切分
                memcpy(&pr_rect_cur->obj_rect[pr_num].pr_rect, &blob_rect[i], sizeof(VCA_BOX_S));
                pr_rect_cur->obj_rect[pr_num].direct = blob_direct[i];
                pr_rect_cur->obj_rect[pr_num].rect_type = 4;
                pr_rect_cur->obj_rect[pr_num].lane = 0;
                pr_rect_cur->obj_rect[pr_num].use_init_rect = 0;
                pr_rect_cur->obj_rect[pr_num].boost_flag = 0;
                pr_num++;
            }
            else
            {
                //判断车道线与Blob是否相交，在上下边界都相交的情况下再考虑是否需要切分
                int ty_cross, by_cross;
                ty_cross = -(lane_abc[1] * blob_rect[i].top + lane_abc[2]) / lane_abc[0];
                by_cross = -(lane_abc[1] * blob_rect[i].bottom + lane_abc[2]) / lane_abc[0];

                if (ty_cross > blob_rect[i].left && ty_cross < blob_rect[i].right
                    || by_cross > blob_rect[i].left && by_cross < blob_rect[i].right)
                {
                    memcpy(&pr_rect_cur->obj_rect[pr_num].pr_rect, &blob_rect[i], sizeof(VCA_BOX_S));
                    pr_rect_cur->obj_rect[pr_num].direct = blob_direct[i];
                    pr_rect_cur->obj_rect[pr_num].rect_type = 4;
                    pr_rect_cur->obj_rect[pr_num].lane = 0;
                    pr_rect_cur->obj_rect[pr_num].use_init_rect = 0;
                    pr_rect_cur->obj_rect[pr_num].boost_flag = 0;
                    pr_num++;

                    memcpy(&pr_rect_cur->obj_rect[pr_num].pr_rect, &blob_rect[i], sizeof(VCA_BOX_S));
                    pr_rect_cur->obj_rect[pr_num].direct = blob_direct[i];
                    pr_rect_cur->obj_rect[pr_num].rect_type = 4;
                    pr_rect_cur->obj_rect[pr_num].lane = 1;
                    pr_rect_cur->obj_rect[pr_num].use_init_rect = 0;
                    pr_rect_cur->obj_rect[pr_num].boost_flag = 0;
                    pr_num++;
                }
                else
                {
                    memcpy(&pr_rect_cur->obj_rect[pr_num].pr_rect, &blob_rect[i], sizeof(VCA_BOX_S));
                    pr_rect_cur->obj_rect[pr_num].direct = blob_direct[i];
                    pr_rect_cur->obj_rect[pr_num].rect_type = 4;
                    pr_rect_cur->obj_rect[pr_num].use_init_rect = 0;
                    pr_rect_cur->obj_rect[pr_num].boost_flag = 0;
                    if (ty_cross < blob_rect[i].left && by_cross < blob_rect[i].left)
                    {
                        pr_rect_cur->obj_rect[pr_num].lane = 1;
                        
                    }
                    else
                    {
                        pr_rect_cur->obj_rect[pr_num].lane = 0;
                    }
                    pr_num++;
                }
            }
            
        }
    }
    pr_rect_cur->pr_num = pr_num;
}

/****************************************************************************************************************
* 函  数：VTS_MPR_check_obj_pr_rect_data
* 功  能：对所有计算的牌识区域进行简单校验，去掉重复的、无效的区域。
* 参  数：
*         pr_rect_last            -I   上一帧数据，用于缓存
*         obj_pr_cur              -I/O 当前帧牌识区域数据
* 返回值：错误状态码
*****************************************************************************************************************/
void VTS_MPR_check_obj_pr_rect_data(MPR_OBJ_PR_RECT_ARRAY *pr_rect_last,
                                    MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur)
{
    int                    k, m, h, l, p;
    int                    del[ITS_MAX_VEH_NUM];
    VCA_BOX_S             *obj_rect = NULL;                        // 牌识区域目标框
    VCA_BOX_S             *obj_rect_other = NULL;                        // 牌识区域目标框
    MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur_ready = pr_rect_last;
    memset(del, 0, sizeof(int)*ITS_MAX_VEH_NUM);

    // 过滤
    for (k = 0; k < pr_rect_cur->pr_num - 1; k++)
    {
        obj_rect = &pr_rect_cur->obj_rect[k].pr_rect;
        for (m = k + 1; m < pr_rect_cur->pr_num; m++)
        {
            obj_rect_other = &pr_rect_cur->obj_rect[m].pr_rect;
            if ((pr_rect_cur->obj_rect[k].lane == pr_rect_cur->obj_rect[m].lane)
                &&(obj_rect->bottom == obj_rect_other->bottom)
                && (obj_rect->left == obj_rect_other->left)
                && (obj_rect->right == obj_rect_other->right)
                && (obj_rect->top == obj_rect_other->top))
            {
                del[m] = 1;
            }
        }
    }

    for (p = 0; p < pr_rect_cur->pr_num; p++)
    {
        obj_rect = &pr_rect_cur->obj_rect[p].pr_rect;
        /*if ((((obj_rect->bottom == 0) && (obj_rect->left == 0)
            && (obj_rect->right == 0) && (obj_rect->top == 0)))
            || ((obj_rect->bottom <= 0) || (obj_rect->left <= 0)
            || (obj_rect->right <= 0) || (obj_rect->top <= 0)))
        {
            del[p] = 1;
        }*/
        if (obj_rect->bottom - obj_rect->top < 30)
        {
            del[p] = 1;
        }

		if (obj_rect->right - obj_rect->left < 100)
		{
			del[p] = 1;
		}
    }

    l = 0;
    pr_rect_cur_ready->pr_num = 0;
    for (h = 0; h < pr_rect_cur->pr_num; h++)
    {
        if (del[h] != 1)
        {
            memcpy(&pr_rect_cur_ready->obj_rect[l], &pr_rect_cur->obj_rect[h], sizeof(MPR_OBJ_PR_RECT));
            l++;
        }
    }
    pr_rect_cur_ready->pr_num = l;
    memset(pr_rect_cur, 0, sizeof(MPR_OBJ_PR_RECT_ARRAY));
    memcpy(pr_rect_cur, pr_rect_cur_ready, sizeof(MPR_OBJ_PR_RECT_ARRAY));
}

/***************************************************************************************************
* 功  能: 计算两条线的交点
* 参  数: pt1                    - I   点1
*         pt2                    - I   点2
*         pt3                    - I   点3
*         pt4                    - I   点4
*         pt_res                 - I/O 输出交点
* 返回值:
* 备  注：与VTS中的函数存在差异，VTS中用的是斜率，MPR中用的是斜率倒数
***************************************************************************************************/
void MPR_calc_line_cross(VCA_POINT_F       *pt1,
                         VCA_POINT_F       *pt2,
                         VCA_POINT_F       *pt3,
                         VCA_POINT_F       *pt4,
                         VCA_POINT_F       *pt_res)
{
    // 其中1和2组成第1条线，3和4组成第2条线
    VCA_POINT_F cross_pt;
    float       a1, a2;
    float       b1, b2;

    CHECK_ERROR(pt_res == NULL, FALSE);

    if ((pt1->y - pt2->y) * (pt3->x - pt4->x) == (pt1->x - pt2->x) * (pt3->y - pt4->y))
    {
        // 给定的两条线平行，出现异常
        cross_pt.x = (pt1->x + pt2->x + pt3->x + pt4->x) * 0.25f;
        cross_pt.y = (pt1->y + pt2->y + pt3->y + pt4->y) * 0.25f;
        *pt_res = cross_pt;

        return FALSE;
    }
    else
    {
        // 判断是否有某条线是垂直的
        if (pt1->x - pt2->x < 0.000001 && pt1->x - pt2->x > -0.000001)
        {
            // 线条1是垂直线
            cross_pt.x = pt1->x;
            a2 = MPR_CALC_LINE_INSLOPE(*pt3, *pt4);
            if (a2 > 0.0000001 || a2 < -0.0000001)
            {
                cross_pt.y = (pt1->x - pt3->x) / a2 + pt3->y;
            }
            else
            {
                cross_pt.y = pt3->y;
            }
        }
        else if (pt3->x - pt4->x < 0.000001 &&pt3->x - pt4->x > -0.000001)
        {
            // 线条2是垂直线
            cross_pt.x = pt3->x;
            a1 = MPR_CALC_LINE_INSLOPE(*pt1, *pt2);
            if (a1 > 0.0000001 || a1 < -0.0000001)
            {
                cross_pt.y = (pt3->x - pt1->x) / a1 + pt1->y;
            }
            else
            {
                cross_pt.y = pt1->y;
            }
        }
        else
        {
            // 两条线是斜交叉线
            a1 = MPR_CALC_LINE_INSLOPE(*pt1, *pt2);
            a2 = MPR_CALC_LINE_INSLOPE(*pt3, *pt4);

            if (pt1->y - pt2->y > 0.000001 || pt1->y - pt2->y < -0.000001)
            {
               // b1 = (pt2->x * pt1->y - pt1->x * pt2->y) * 1.f / (pt2->x - pt1->x);
                b1 = (pt2->x * pt1->y - pt1->x * pt2->y) * 1.f / (pt1->y - pt2->y);
            }
            else
            {
                b1 = ITS_FLT_MAX;
            }
            if (pt3->y - pt4->y > 0.000001 || pt3->y - pt4->y < -0.000001)
            {
               // b2 = (pt4->x * pt3->y - pt3->x * pt4->y) * 1.f / (pt4->x - pt3->x);
                b2 = (pt4->x * pt3->y - pt3->x * pt4->y) * 1.f / (pt3->y - pt4->y);
            }
            else
            {
                b2 = ITS_FLT_MAX;
            }
            //cross_pt.x = (b2 - b1) / (a1 - a2);
            //cross_pt.y = (a1 * b2 - a2 * b1) / (a1 - a2);
            cross_pt.y = (b2 - b1) / (a1 - a2);
            cross_pt.x = (a1 * b2 - a2 * b1) / (a1 - a2);
        }

        *pt_res = cross_pt;
    }
}


/****************************************************************************************************************
* 函  数：VTS_MPR_select_obj_pr_rect_data
* 功  能：挑选出一个离卡口触发线最近的车牌区域。
* 参  数：
*         pr_rect_last            -I   上一帧数据，用于缓存
*         obj_pr_cur              -I/O 当前帧牌识区域数据
* 返回值：错误状态码
*****************************************************************************************************************/
void VTS_MPR_select_obj_pr_rect_data(MPR_OBJ_PR_RECT_ARRAY *pr_rect_last,
	MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur, 
	ITS_LINE_F            *snap_line,
	int                    image_width,
	int                    image_height,
        unsigned char          mpr_mode,
        ITS_LANE_PARAM        *lanes_list,
	int                    dir_flag)
{
	int              i, j;
	MPR_OBJ_PR_RECT *obj_rect = NULL;                        // 牌识区域目标框
	VCA_BOX_S       *pr_rect = NULL;
	VCA_BOX_S        *rect_tmp = NULL;
	ITS_LINE_SLOPE   trig_line;
	float A, B, C, ps, AB2, dist, dist_temp;
	VCA_BOX_S       *pr_rect_temp = NULL;
	int              obj_width;
	//MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur_ready = pr_rect_cur;
	int              image_w = MAX_VET_WIDTH;
	int              image_h = MAX_VET_HEIGHT;

	CHECK_ERROR((NULL == pr_rect_last), HIK_MPR_PS_PTR_NULL);

	memset(pr_rect_cur, 0, sizeof(MPR_OBJ_PR_RECT_ARRAY));
	//挑选牌识区域，挑选出离触发线最近的一个牌识区域
	if (pr_rect_last->pr_num > 0)
	{
	    if (pr_rect_last->pr_num == 1)
	    {
			obj_rect = &pr_rect_last->obj_rect[0];
			rect_tmp = &obj_rect->pr_rect;
	    }
	    else
            {
            	//printf("mpr==mode=%d,pr_num=%d\n",mpr_mode,pr_rect_last->pr_num);
                if (mpr_mode == EVIDENCE_MPR)    //取证多帧中，使用图像中心点距离作为选择的依据
                {
                    //计算触发线重点到目标中心的距离，选取距离最小的目标
                    VCA_POINT_F pt_res[2];
                    VCA_POINT_F center_pt;
                    VCA_POINT_F center_obj;
                    //MPR_calc_line_cross(&lanes_list->lane_left_line.pt1, &lanes_list->lane_left_line.pt2,
                    //    &snap_line->pt1, &snap_line->pt2, &pt_res[0]);
                   // MPR_calc_line_cross(&lanes_list->lane_rgt_line.pt1, &lanes_list->lane_rgt_line.pt2,
                   //     &snap_line->pt1, &snap_line->pt2, &pt_res[1]);
                   // center_pt.x = (pt_res[0].x + pt_res[1].x)*0.5f;
                   // center_pt.y = (pt_res[0].y + pt_res[1].y)*0.5f;
                    center_pt.x = (snap_line->pt1.x + snap_line->pt2.x)*0.5f;
                    center_pt.y = (snap_line->pt1.y + snap_line->pt2.y)*0.5f;

                    rect_tmp = &pr_rect_last->obj_rect[0].pr_rect;
                    dist_temp = 10000000.f;
                    //printf("mpr==snap_line:(%f,%f,%f,%f)\n",snap_line->pt1.x,snap_line->pt1.y,snap_line->pt2.x,snap_line->pt2.y);
                    //printf("mpr==center_pt:%f,%f,lane_line:(%f,%f,%f,%f)-(%f,%f,%f,%f)\n",center_pt.x,center_pt.y,
                    //lanes_list->lane_left_line.pt1.x,lanes_list->lane_left_line.pt1.y,lanes_list->lane_left_line.pt2.x,lanes_list->lane_left_line.pt2.y,
                    //lanes_list->lane_rgt_line.pt1.x,lanes_list->lane_rgt_line.pt1.y,lanes_list->lane_rgt_line.pt2.x,lanes_list->lane_rgt_line.pt2.y);
                    //选取目标中心距离触发线重点最近的目标区域作为最后的牌识区域
                    for (i = 0; i < pr_rect_last->pr_num; i++)
                    {
                        obj_rect = &pr_rect_last->obj_rect[i];
                        pr_rect = &obj_rect->pr_rect;
                        center_obj.x = (pr_rect->left + pr_rect->right)*0.5f / image_width;
                        center_obj.y = (pr_rect->top + pr_rect->bottom)*0.5f / image_height;
                        dist = (center_obj.x - center_pt.x)*(center_obj.x - center_pt.x) + (center_obj.y - center_pt.y)*(center_obj.y - center_pt.y);
                        //printf("mpr==i=%d,dist=%f,rect:%d,%d,wh=%d,%d\n",i,dist,pr_rect->left,pr_rect->top,image_width,image_height);
                        if (dist < dist_temp)
                        {
                            rect_tmp = pr_rect;
                            dist_temp = dist;
                        }
                    }
                    //printf("mpr==rect_tmp:%d,%d,%d,%d\n",rect_tmp->left,rect_tmp->top,rect_tmp->right,rect_tmp->bottom);
                }
                else
                {
                    memset(&trig_line, 0, sizeof(ITS_LINE_SLOPE));
                    trig_line.x = (snap_line->pt1.x + snap_line->pt2.x) * 0.5f;
                    trig_line.y = (snap_line->pt1.y + snap_line->pt2.y) * 0.5f;
                    trig_line.inv_slope = MPR_CALC_LINE_INSLOPE(snap_line->pt1, snap_line->pt2);
                    trig_line.type = snap_line->type;

                    dist_temp = 10000000.f;

                    // 直线转换成一般式，即A*X+B*Y+C=0
                    A = 1.0f;
                    B = -trig_line.inv_slope;
                    C = trig_line.inv_slope * trig_line.y - trig_line.x;


                    for (i = 0; i < pr_rect_last->pr_num; i++)
                    {
                        obj_rect = &pr_rect_last->obj_rect[i];
                        pr_rect = &obj_rect->pr_rect;

                        // 计算分子 A*X+B*Y+C

                        ps = A * 0.5 * (pr_rect->left + pr_rect->right) / image_width + B * 0.5 * (pr_rect->top + pr_rect->bottom) / image_height + C;

                        AB2 = A * A + B * B;

                        // 计算距离A*X+B*Y+C/SQRT(A*A+B*B),避免开平方，因此分子、分母都平方，外部调用处也需平方

                        dist = ps * ps / AB2;


                        if (dist < dist_temp)
                        {
                            rect_tmp = pr_rect;
                            dist_temp = dist;

                        }
                    }
                }
            }
	    pr_rect_cur->pr_num = 1;

	    if (rect_tmp->bottom - rect_tmp->top + 1 > image_h)
	    {
		rect_tmp->top = rect_tmp->bottom - image_h + 1;
	    }   
		//宽度大于800，左右分别裁剪一半	
	    obj_width = rect_tmp->right - rect_tmp->left + 1;
	    if (obj_width > image_w)
	    {
		//向左，右侧切掉
		if (dir_flag == 0)
		{
		    rect_tmp->right = rect_tmp->right - (obj_width - image_w);
		}
		//向右，左侧切掉
		else if (dir_flag == 1)
		{
		    rect_tmp->left = rect_tmp->left + (obj_width - image_w);
		}
		//1左1右，左右各切一半
		else if (dir_flag == 2)
		{
			rect_tmp->left = rect_tmp->left + 0.5 * (obj_width - image_w);
			rect_tmp->right = rect_tmp->right - 0.5 * (obj_width - image_w);
		}			
	    }

	    memcpy(&obj_rect->pr_rect, rect_tmp, sizeof(VCA_BOX_S));
	    memcpy(&pr_rect_cur->obj_rect[0], obj_rect, sizeof(MPR_OBJ_PR_RECT));
	}
	return;
}

/**************************************************************************************************
* 功 能：牌识区域切分，结合动态牌识区域和跟踪预测区域进行
* 参 数：*
*         pr_handle                 - I    牌识部分模块句柄
*         frame                     - I    帧信息
*         dynamic_pr_rect           - I    动态牌识区域
*         obj_pr_cur                - O    牌识区域数据
*         pr_up_line                - I    牌识区域上边界
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
HRESULT VTS_MPR_pr_rect_cut(void                  *pr_handle,
                            int					   img_w,
							int                    img_h, 
                            MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur,
                            int                    pr_up_line,
                            int                    plate_w_ave,
                            unsigned char          is_cap_frm)
{
    HRESULT        ret = HIK_MPR_S_OK;
    MPR_PR_HANDLE *mpr_pr_hdl = (MPR_PR_HANDLE *)pr_handle;                       // 获取当前句柄
    //int            img_w, img_h;


	CHECK_ERROR((NULL == mpr_pr_hdl || NULL == pr_rect_cur), HIK_MPR_PS_PTR_NULL);

    pr_rect_cur = &mpr_pr_hdl->pr_rect_cur;
    //img_w       = frame->frm_data.image_w;
    //img_h       = frame->frm_data.image_h;


    // 记录跟踪和预测区域的个数
    memset(mpr_pr_hdl->contain_index, 0, sizeof(int) * VCA_MAX_OBJ_NUM * ITS_MAX_VEH_NUM);

    //对车头车尾目标进行校验，将包含跟踪预测块的目标删除
	VTS_MPR_check_mod_data(mpr_pr_hdl, pr_rect_cur, img_w, img_h, pr_up_line, mpr_pr_hdl->det_obj_list);

    return HIK_MPR_S_OK;
}

/**************************************************************************************************
 * 功 能：转换牌识处理参数
 * 参 数：*
 *         pr_rect_cur_frm        - I    计算的牌识区域
 *         frame                  - I    当前图像
 *         img                    - I    图像地址
 *         pr_param               - I    初始牌识参数
 *         plate_w_ave            - I    车牌平均宽度
 *         pr_proc_param          - O    牌识参数
 * 返回值：状态码
 * 备 注：无
 ***************************************************************************************************/
HRESULT VTS_MPR_pr_proc_param_build(MPR_OBJ_PR_RECT_ARRAY   *pr_rect_cur_frm,
                                    ITS_FRM                 *frame,
									MPR_PR_HANDLE			*pr_handle,
                                    int                      plate_w_ave,
                                    HIK_LPR_PROC_PARAM      *pr_proc_param,
                                    int                      pr_up_line)
{
    int              i, j;
    MPR_OBJ_PR_RECT *obj_rect = NULL;                        // 牌识区域目标框
    PR_POINT        *vertexs  = NULL;                        // 牌识区域顶点
    VCA_BOX_S       *pr_rect  = NULL;
    VCA_BOX_S        rect_tmp;
    int              pr_num = 0, flag_multi = 0, count = 0;
	int				 stitch_img_size;
	unsigned char	*src_data_y = NULL, *src_data_uv = NULL;
	unsigned char	*dst_data_y = NULL, *dst_data_uv = NULL;
	VCA_YUV_DATA    *frm_yuv_data = NULL;
	int				 rect_w, rect_h;
	int				 offset, sum_offset_h = 0;
	int				 max_rect_w = 0;

	int              image_w = 0;
	int              image_h = 0;


	char bmpname[128];

	CHECK_ERROR((NULL == pr_rect_cur_frm || NULL == frame || NULL == pr_handle), HIK_MPR_PS_PTR_NULL);

	if (pr_handle->pr_param->dwRecogMode == LPR_RECOG_VET)
	{
		image_w = MAX_VET_WIDTH;
		image_h = MAX_VET_HEIGHT;

	}
	else if (pr_handle->pr_param->dwRecogMode == LPR_RECOG_HVT)
	{
		image_w = MAX_DOWNSAMPLE_STITCH_WIDTH;
		image_h = MAX_DOWNSAMPLE_STITCH_HEIGHT;

	}
	else
	{
		return HIK_MPR_FLW_PARAM_ERR;
	}
	///清空拼接图像buf
	stitch_img_size = image_w * image_h;
	memset(pr_handle->stitch_img_buf, 0, 3*stitch_img_size/2);

	/// 将所有牌识框截取拼成一张小图，送给牌识做一次
	pr_proc_param->debug_data = frame->debug_ptr;
	pr_proc_param->iUseInitRect = 0;
	pr_proc_param->PolygonInfo.iLaneNum = 1;
	pr_proc_param->iLaneNo = 0;

	pr_proc_param->imgyuv = &pr_handle->yuv_data;
	pr_proc_param->imgyuv->format = frame->frm_data.format;
	pr_proc_param->imgyuv->image_w = image_w;
	pr_proc_param->imgyuv->image_h = image_h;
	pr_proc_param->imgyuv->pitch_y = image_w;
	pr_proc_param->imgyuv->pitch_uv = image_w;
	pr_proc_param->imgyuv->y = pr_handle->stitch_img_buf;
	pr_proc_param->imgyuv->u = pr_handle->stitch_img_buf + stitch_img_size;

	frm_yuv_data = &frame->frm_data;

	dst_data_y = pr_proc_param->imgyuv->y;
	dst_data_uv = pr_proc_param->imgyuv->u;
	///竖直拼接
	for (i = 0; i < pr_rect_cur_frm->pr_num; i++)
	{
		obj_rect = &pr_rect_cur_frm->obj_rect[i];
		pr_rect = &obj_rect->pr_rect;
		rect_w = pr_rect->right - pr_rect->left + 1;
		rect_h = pr_rect->bottom - pr_rect->top + 1;

		if (rect_w <= 200 || rect_h <= 100)
		{
			continue;
		}

		if (pr_rect->left % 2 != 0)
			pr_rect->left -= 1;

		if (pr_rect->top % 2 != 0)
			pr_rect->top -= 1;

		if (rect_w % 2 != 0)
			rect_w -= 1;

		if (rect_h % 2 != 0)
			rect_h -= 1;

		if (rect_w > image_w)
		{
			rect_w = image_w;
			pr_rect->right = pr_rect->left + rect_w - 1;
		}

		if (sum_offset_h + rect_h > image_h)
		{
			break;
		}
		
		if (max_rect_w <= rect_w)
		{
			max_rect_w = rect_w;
		}

		///y分量拼接
		offset = pr_rect->top * frm_yuv_data->image_w + pr_rect->left;
		src_data_y = frm_yuv_data->y + offset;
		for (j = 0; j < rect_h; ++j)
		{
			memcpy(dst_data_y, src_data_y, rect_w);

			dst_data_y += image_w;
			src_data_y += frm_yuv_data->image_w;
		}

		///uv分量拼接
		offset = (pr_rect->top >> 1) * frm_yuv_data->image_w + pr_rect->left;
		src_data_uv = frm_yuv_data->u + offset;
		for (j = 0; j < rect_h >> 1; ++j)
		{
			memcpy(dst_data_uv, src_data_uv, rect_w);

			dst_data_uv += image_w;
			src_data_uv += frm_yuv_data->image_w;
		}

		pr_handle->ori_pr_rect[i].left = pr_rect->left;
		pr_handle->ori_pr_rect[i].right = pr_rect->right;
		pr_handle->ori_pr_rect[i].top = pr_rect->top;
		pr_handle->ori_pr_rect[i].bottom = pr_rect->bottom;

		pr_handle->stitch_pr_rect[i].left = 0;
		pr_handle->stitch_pr_rect[i].right = rect_w - 1;
		pr_handle->stitch_pr_rect[i].top = sum_offset_h;
		pr_handle->stitch_pr_rect[i].bottom = pr_handle->stitch_pr_rect[i].top + rect_h - 1;

		sum_offset_h += rect_h;
		++count;
	}

	//sprintf(bmpname, "..\\data\\tmp\\stitch_img_%d.bmp", frame->frm_header.frame_num);
	//save_bmp_pic(pr_proc_param->imgyuv->y, MAX_STITCH_WIDTH, MAX_STITCH_HEIGHT, bmpname, 0);

	pr_handle->stitch_pr_rect_num = count;
	pr_proc_param->uchLocateNum = count;
	pr_proc_param->iOutputMultiRes = 0;
	if (count >= 2)
		pr_proc_param->iOutputMultiRes = 1;
	
	if (count > 0)
	{
		vertexs = pr_proc_param->PolygonInfo.aLanes[0].point;
		pr_proc_param->PolygonInfo.aLanes[0].vertex_num = 4;

		vertexs[0].x = 0;
		vertexs[0].y = 0;
		vertexs[1].x = 0;
		vertexs[1].y = sum_offset_h-1;
		vertexs[2].x = max_rect_w;
		vertexs[2].y = sum_offset_h - 1;
		vertexs[3].x = max_rect_w;
		vertexs[3].y = 0;
	}
	else
	{
		pr_proc_param->PolygonInfo.aLanes[0].vertex_num = 0;
	}

    // 将属于同一车道的牌识区域归属到一起，取最外边界作为最终的牌识区域
	//for (j = 0; j < pr_param->PolygonInfo.iLaneNum; j++)
	//{
	//	rect_tmp.left   = 10000;
	//	rect_tmp.top    = 10000;
	//	rect_tmp.right  = 0;
	//	rect_tmp.bottom = 0;
	//	count = 0;
	//	flag_multi = 1;
 //       for (i = 0; i < pr_rect_cur_frm->pr_num; i++)
	//	{
	//		obj_rect = &pr_rect_cur_frm->obj_rect[i];
	//		pr_rect = &obj_rect->pr_rect;

	//		if (obj_rect->lane == j)
	//		{
	//			rect_tmp.left = MIN(rect_tmp.left, pr_rect->left);
	//			rect_tmp.top = MIN(rect_tmp.top, pr_rect->top);
	//			rect_tmp.right = MAX(rect_tmp.right, pr_rect->right);
 //               rect_tmp.bottom = MAX(rect_tmp.bottom, pr_rect->bottom);
	//			
	//			count++;
	//			if (count == 1 && (obj_rect->rect_type == 2 || obj_rect->boost_flag))
	//			{
	//				flag_multi = 0;
	//			}
	//			else
	//			{
	//				flag_multi = 1;
	//			}
	//		}
	//	}

	//	if (rect_tmp.right > 0)
	//	{
	//		// 填充牌识区域
	//		pr_proc_param[j].iOutputMultiRes = flag_multi;
	//		pr_proc_param[j].debug_data = frame->debug_ptr;
	//		pr_proc_param[j].imgyuv = &frame->frm_data;
	//		pr_proc_param[j].iUseInitRect = 0;
	//		pr_proc_param[j].uchLocateNum = (count < 3) ? 3 : count;
	//		pr_proc_param[j].PolygonInfo.iLaneNum = pr_param->PolygonInfo.iLaneNum;
	//		pr_proc_param[j].iLaneNo = j;

	//		vertexs = pr_proc_param[j].PolygonInfo.aLanes[j].aVertex;
	//		pr_proc_param[j].PolygonInfo.aLanes[j].iVertexNum = 4;

	//		vertexs[0].x = rect_tmp.left;
	//		vertexs[0].y = max(rect_tmp.top, pr_up_line);
	//		vertexs[1].x = rect_tmp.left;
	//		vertexs[1].y = max(rect_tmp.bottom, pr_up_line);
	//		vertexs[2].x = rect_tmp.right;
	//		vertexs[2].y = max(rect_tmp.bottom, pr_up_line);
	//		vertexs[3].x = rect_tmp.right;
	//		vertexs[3].y = max(rect_tmp.top, pr_up_line);
	//	}
	//	else
	//	{
	//		pr_proc_param[j].PolygonInfo.aLanes[j].iVertexNum = 0;
	//		pr_proc_param[j].iLaneNo = j;
	//	}
	//} 
    return HIK_MPR_S_OK;
}

/**************************************************************************************************
* 功 能：转换牌识处理参数
* 参 数：*
*         pr_rect_cur_frm        - I    计算的牌识区域
*         frame                  - I    当前图像
*         img                    - I    图像地址
*         pr_param               - I    初始牌识参数
*         plate_w_ave            - I    车牌平均宽度
*         pr_proc_param          - O    牌识参数
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
//int MPR_get_capfrm_pr_proc_param(MPR_PROC_PARAM          *run_param,
//                                 ITS_FRM                 *frame,
//                                 PSM_LPR_INIT_PARAM      *pr_param,
//                                 HIK_LPR_PROC_PARAM      *pr_proc_param)
//{
//    int               i = 0;
//    MPR_OBJ_PR_RECT  *obj_rect = NULL;                        // 牌识区域目标框
//    PR_POINT         *vertexs = NULL;                        // 牌识区域顶点
//    MPR_DYNAMIC_RECT *dynamic_pr_rect = NULL;
//    VCA_BOX_S         pr_rect;
//	VCA_BOX_S         rect_tmp;
//	int               count = 0;
//
//	if (pr_param->PolygonInfo.iLaneNum == 1)
//	{
//		rect_tmp.left = 10000;
//		rect_tmp.top = 10000;
//		rect_tmp.right = 0;
//		rect_tmp.bottom = 0;
//		count = 0;
//
//		dynamic_pr_rect = run_param->dynamic_pr_rect;
//
//		// 生成牌识Proc参数
//		for (i = 0; i < ITS_MAX_VEH_NUM; i++)
//		{
//			if (dynamic_pr_rect[i].valid == 0)
//			{
//				continue;
//			}
//			VTS_MPR_convert_f2s(&dynamic_pr_rect[i].rect,
//				frame->frm_data.image_w,
//				frame->frm_data.image_h,
//				&pr_rect);
//			rect_tmp.left = MIN(rect_tmp.left, pr_rect.left);
//			rect_tmp.top = MIN(rect_tmp.top, pr_rect.top);
//			rect_tmp.right = MIN(rect_tmp.right, pr_rect.right);
//			rect_tmp.bottom = MIN(rect_tmp.bottom, pr_rect.bottom);
//			count++;
//		}
//		if (count > 0)
//		{
//			// 填充牌识区域
//			pr_proc_param[0].iOutputMultiRes = (count > 1) ? 1 : 0;
//			pr_proc_param[0].debug_data = frame->debug_ptr;
//			pr_proc_param[0].imgyuv = &frame->frm_data;
//			pr_proc_param[0].iUseInitRect = 0;
//			pr_proc_param[0].uchLocateNum = (count < 3) ? 3 : count;
//			pr_proc_param[0].PolygonInfo.iLaneNum = pr_param->PolygonInfo.iLaneNum;
//			pr_proc_param[0].iLaneNo = 0;
//
//			vertexs = pr_proc_param[0].PolygonInfo.aLanes[0].aVertex;
//			pr_proc_param[0].PolygonInfo.aLanes[0].iVertexNum = 4;
//
//			vertexs[0].x = rect_tmp.left;
//			vertexs[0].y = rect_tmp.top;
//			vertexs[1].x = rect_tmp.left;
//			vertexs[1].y = rect_tmp.bottom;
//			vertexs[2].x = rect_tmp.right;
//			vertexs[2].y = rect_tmp.bottom;
//			vertexs[3].x = rect_tmp.right;
//			vertexs[3].y = rect_tmp.top;
//		}
//	}
//
//	return count;
//}

/**************************************************************************************************
 * 功 能：从跟踪获取牌识共享信息
 * 参 数：*
 *         plate_cur              - I    历史车牌最近一次牌识结果
 *         pr_share_info          - O    牌识共享数据
 * 返回值：状态码
 * 备 注：无
 ***************************************************************************************************/
HRESULT MPR_get_pr_share_info_one(MPR_ID_DATA   *plate_cur,
                                  PR_PRIOR_INFO *pr_share_info)
{
    int                  i;
    MPR_OBJ_PR_RECT     *obj_rect       = NULL;              // 牌识区域目标框
    MPR_PLATE_RAW_QUEUE *queue          = NULL;
    MPR_PRRESULT        *pr_info        = NULL;
    short                dis_sum[2]     = { 0 };             // 分别对应x、y方向
    char                 plate_color[3] = { 0 };
    PR_RECT             *rect_tmp       = NULL;
    PR_RECT             *rect_arr       = NULL;
    int                  count          = 0;

    queue    = &plate_cur->queue;
    i        = (queue->back - 1 + MPR_RAW_QUEUE_CAP) % MPR_RAW_QUEUE_CAP;
    pr_info  = &queue->plate_array[i].pr_info;
    rect_tmp = &pr_info->segment_rect;

    pr_share_info->plate_cred     = pr_info->plate_cred;
    pr_share_info->plate_gradient = pr_info->plate_gradient;
    pr_share_info->plate_type     = pr_info->plate_type;
    pr_share_info->char_gradient  = pr_info->char_gradient;
    pr_share_info->symbol_num     = pr_info->symbol_num;

    memcpy(plate_color, pr_info->pr_result, 2);
    if (strcmp(plate_color, "蓝") == 0)
    {
        pr_share_info->plate_color = 3;
        pr_share_info->pros_cons   = 1;
    }
    else if (strcmp(plate_color, "黄") == 0)
    {
        pr_share_info->plate_color = 4;
        pr_share_info->pros_cons   = 2;
    }
    else if (strcmp(plate_color, "白") == 0)
    {
        pr_share_info->plate_color = 1;
        pr_share_info->pros_cons   = 2;
    }
    else if (strcmp(plate_color, "黑") == 0)
    {
        pr_share_info->plate_color = 2;
        pr_share_info->pros_cons   = 1;
    }
    else if (strcmp(plate_color, "绿") == 0)
    {
        pr_share_info->plate_color = 5;
        pr_share_info->pros_cons   = 1;
    }
    else
    {
        pr_share_info->plate_color = 0;
        pr_share_info->pros_cons   = 0;
    }

    memcpy(&pr_share_info->rect_last, &pr_info->segment_rect, sizeof(PR_RECT));

    memcpy(pr_share_info->plate_symbol, pr_info->plate_symbol, sizeof(char) * ITS_MAX_CHAR_NUM);
    memcpy(pr_share_info->symbol_cred, pr_info->symbol_cred, sizeof(short) * ITS_MAX_CHAR_NUM);
    memcpy(pr_share_info->symbol_pos, pr_info->symbol_pos, sizeof(PR_POINT) * ITS_MAX_CHAR_NUM);

    // 判断车牌是否停止，车牌位置是否可用
    pr_share_info->symbol_pos_valid = 0;
    if (queue->count > 4)
    {
        i = (i - 1 + MPR_RAW_QUEUE_CAP) % MPR_RAW_QUEUE_CAP;
        while (count < 5)
        {
            count++;
            rect_arr   = &queue->plate_array[i].pr_info.segment_rect;
            dis_sum[0] = abs(rect_arr->lx - rect_tmp->lx);
            dis_sum[1] = abs(rect_arr->ty - rect_tmp->ty);
        }
        if (dis_sum[0] + dis_sum[0] < 10)
        {
            pr_share_info->symbol_pos_valid = 1;
        }
    }

    return HIK_MPR_S_OK;
}

/**************************************************************************************************
* 功 能：从跟踪获取牌识共享信息
* 参 数：*
*         handle                 - I    牌识部分模块句柄
*         dynamic_pr_rect        - I    动态牌识区域
*         trace_info             - I    跟踪信息
*         obj_pr_cur             - O    牌识区域参数
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
HRESULT MPR_get_pr_share_info(MPR_ID_QUEUE          *id_queue,
                              MPR_OBJ_PR_RECT_ARRAY *obj_pr_cur)
{
    HRESULT          ret = HIK_MPR_S_OK;
    int              i, j;
    MPR_OBJ_PR_RECT *obj_rect      = NULL;                   // 牌识区域目标框
    ITS_PLATE_QUEUE *queue         = NULL;
    PR_PRIOR_INFO   *pr_share_info = NULL;
    MPR_ID_DATA     *plate_cur     = NULL;
    MPR_PRRESULT    *pr_info       = NULL;
    int              find_flg      = 0;
    int              count         = 0;

    for (i = 0; i < obj_pr_cur->pr_num; i++)
    {
        obj_rect      = &obj_pr_cur->obj_rect[i];
        pr_share_info = &obj_rect->pr_share_info;
        find_flg      = 0;
        if (obj_rect->plate_id > 0)
        {
            count = 0;
            j     = (id_queue->back - 1 + MPR_ID_QUEUE_CAP) % MPR_ID_QUEUE_CAP;
            while (j != id_queue->back && count < id_queue->count)
            {
                count++;
                plate_cur = &id_queue->id_array[j];
                if (obj_rect->plate_id == plate_cur->plate_id)
                {
                    pr_share_info->pre_seg = obj_rect->rect_type - 2;
                    ret = MPR_get_pr_share_info_one(plate_cur, pr_share_info);
                    CHECK_ERROR(ret != HIK_MPR_S_OK, ret);
                    find_flg = 1;
                    break;
                }
                j = (j - 1 + MPR_ID_QUEUE_CAP) % MPR_ID_QUEUE_CAP;  
            }
        }
    }

    return HIK_MPR_S_OK;
}

/**************************************************************************************************
* 功 能：计算牌识区域个数和对应的牌识参数
* 参 数：*
*         handle                 - I    牌识部分模块句柄
*         dynamic_pr_rect        - I    初始动态牌识区域（外部传进多帧）
*         trace_info             - I    跟踪信息
*         frame                  - I    图像帧信息
*         pr_proc_param          - O    牌识区域参数
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
HRESULT VTS_MPR_calc_pr_rect(void                    *pr_handle,
                             MPR_DYNAMIC_RECT        *dynamic_pr_rect,
                             ITS_FRM                 *frame,
                             HIK_LPR_PROC_PARAM      *pr_proc_param,
                             MPR_MODULE_INTERFACE    *module_interface)
{
   /* HRESULT                rtn = HIK_MPR_S_OK;
    MPR_PR_HANDLE         *mpr_pr_hdl       = (MPR_PR_HANDLE *)pr_handle;    // 获取当前句柄
    MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur_frm  = NULL;                          // 当前帧牌识区域数据
    MPR_OBJ_PR_RECT_ARRAY *pr_rect_last_frm = NULL;                          // 上一帧牌识区域数据
    int                    time_dis         = 0, n = 0, flag_super_big=0;

    // 初始化
    pr_rect_cur_frm  = &mpr_pr_hdl->pr_rect_cur;
    pr_rect_last_frm = &mpr_pr_hdl->pr_rect_last;

    // 上一帧牌识区域清零
    memcpy(pr_rect_last_frm, pr_rect_cur_frm, sizeof(MPR_OBJ_PR_RECT_ARRAY));
    memset(pr_rect_cur_frm, 0, sizeof(MPR_OBJ_PR_RECT_ARRAY));
    memset(mpr_pr_hdl->pr_rect_in, 0, sizeof(VCA_OBJ_INFO) * ITS_MAX_VEH_NUM);
    mpr_pr_hdl->pr_in_index     = 0;
    *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 32;

    // 1、获取帧率信息
    rtn = VTS_MPR_get_frame_rate(mpr_pr_hdl, &frame->frm_header, &module_interface->frm_rate);
    CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
    *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 33;

    // 2、拥堵判断：：默认jam_flag = 1 - 拥堵
    rtn = VTS_MPR_get_jam_flag(mpr_pr_hdl, frame, mpr_pr_hdl->jam_flag, module_interface);
    CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
    *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 34;

    // 3、跟踪牌识区域；
    if (module_interface->is_cap_frm == FALSE)
    {   // 当曝闪帧时不采用跟踪结果
    //    rtn = VTS_MPR_pr_rect_from_track(mpr_pr_hdl,
    //        frame,
    //        trace_info,
    //        pr_rect_cur_frm,
    //        module_interface->pr_up_line);
    //    CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);

    //    *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 35;

        // 4、历史车牌预测；如果对应车牌目标已在跟踪牌识区域中存在，则丢弃该目标；
        rtn = VTS_MPR_pr_rect_from_old_plate(mpr_pr_hdl,
            frame,
            pr_rect_cur_frm,
            module_interface,
            &flag_super_big);
        CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
        *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 36;
    }
        // 获取牌识共享数据
    rtn = MPR_get_pr_share_info(&module_interface->id_queue, pr_rect_cur_frm);
    CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);

    *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 37;

    // 5、动态牌识区域切分
    rtn = VTS_MPR_pr_rect_cut(mpr_pr_hdl,
                                  frame,
                                  dynamic_pr_rect,
                                  pr_rect_cur_frm,
                                  module_interface->pr_up_line,
                                  module_interface->plate_w_ave,
                                  module_interface->is_cap_frm);
    CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
    *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 38;

    // 6、牌识区域优先级排序
    if (mpr_pr_hdl->stop_sort_flag != 1)
    {
        rtn = VTS_MPR_rect_pr_sort(pr_rect_last_frm, pr_rect_cur_frm);
        CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
        *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 39;
    }

    // 对计算的牌识区域进行简单的过滤、合并处理
    VTS_MPR_check_obj_pr_rect_data(pr_rect_last_frm, pr_rect_cur_frm);
    
    // 7、生成牌识Proc参数
    rtn = VTS_MPR_pr_proc_param_build(pr_rect_cur_frm,
                                      frame,
                                      mpr_pr_hdl,
                                      module_interface->plate_w_ave,
                                      pr_proc_param,
                                      module_interface->pr_up_line);
    CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
    *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 40;

    memcpy(&module_interface->pr_objs_cur_frm, pr_rect_cur_frm, sizeof(MPR_OBJ_PR_RECT_ARRAY));

#if _WIN32 && MPR_DEBUG_VISUALIZER
    VTS_MPR_show_cutted_pr_rect(frame->frm_data.y,
                                frame->frm_data.u,
                                frame->frm_data.image_w,
                                frame->frm_data.image_h,
                                pr_rect_cur_frm);
#endif
    *(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 41;*/
    return HIK_MPR_S_OK;
}

/**************************************************************************************************
* 功 能：爆闪模式，没有牌识区域的情况下，创建牌识区域
* 参 数：*
*         snap_line              - I    卡口触发线
*         lane_num               - I    车道数
*         lanes_list             - O    车傲参数
*         image_w                - I    图像宽度
*         image_h                - I    图像高度
*         pr_rect_cur_frm        - I    牌识区域列表
* 返回值：状态码
* 备 注：无
***************************************************************************************************/

//爆闪没有牌识区域的情况下，创建牌识区域
void VTS_MPR_Create_pr_rect(MPR_OBJ_PR_RECT_ARRAY* pr_rect_cur_frm, ITS_LINE_F* snap_line, int lane_num, ITS_LANE_PARAM* lanes_list, int image_w, int image_h)
{
	VCA_POINT_F left_pt, rgt_pt;
	float center_y = 0.f;
	float center_x = 0.f;

	//有车道线的情况下，求卡口触发线和车道线的交点，取两个交点的中心点
	if (lane_num > 0)
	{
		VTS_MPR_calc_line_cross(&snap_line->pt1, &snap_line->pt2, &lanes_list->lane_left_line.pt1, &lanes_list->lane_left_line.pt2, &left_pt);
		VTS_MPR_calc_line_cross(&snap_line->pt1, &snap_line->pt2, &lanes_list->lane_rgt_line.pt1, &lanes_list->lane_rgt_line.pt2, &rgt_pt);

		left_pt.x = MAX(left_pt.x, 0.0);
		rgt_pt.x  = MIN(rgt_pt.x, 1.0);

		left_pt.y = MAX(left_pt.y, 0.0);
		left_pt.y = MIN(left_pt.y, 1.0);
		rgt_pt.y  = MAX(rgt_pt.y, 0.0);
		rgt_pt.y  = MIN(rgt_pt.y, 1.0);

		center_x = 0.5 * (left_pt.x + rgt_pt.x);
		center_y = 0.5 * (left_pt.y + rgt_pt.y);
	}
	//没有车道线的情况下，x方向取图像的中心，y方向取卡口触发线的中心
	else
	{
		center_x = 0.5;
		center_y = 0.5 * (snap_line->pt1.y + snap_line->pt2.y);
	}

	//计算y方向上下边界，取高度为600的范围
	pr_rect_cur_frm->pr_num = 1;
	pr_rect_cur_frm->obj_rect[0].pr_rect.top = (int)(center_y * image_h) - 300;
	pr_rect_cur_frm->obj_rect[0].pr_rect.bottom = pr_rect_cur_frm->obj_rect[0].pr_rect.top + 599;

	//判断是否出界
	if (pr_rect_cur_frm->obj_rect[0].pr_rect.top < 0)
	{
		pr_rect_cur_frm->obj_rect[0].pr_rect.top = 0;
		pr_rect_cur_frm->obj_rect[0].pr_rect.bottom = 599;
	}

	if (pr_rect_cur_frm->obj_rect[0].pr_rect.bottom >= image_h)
	{
		pr_rect_cur_frm->obj_rect[0].pr_rect.bottom = image_h - 1;
		pr_rect_cur_frm->obj_rect[0].pr_rect.top = image_h - 600;
	}

	//计算x方向上下边界，取高度为800的范围
	pr_rect_cur_frm->obj_rect[0].pr_rect.left = (int)(center_x * image_w) - 400;
	pr_rect_cur_frm->obj_rect[0].pr_rect.right = pr_rect_cur_frm->obj_rect[0].pr_rect.left + 799;
	

	//判断是否出界
	if (pr_rect_cur_frm->obj_rect[0].pr_rect.left < 0)
	{
		pr_rect_cur_frm->obj_rect[0].pr_rect.left = 0;
		pr_rect_cur_frm->obj_rect[0].pr_rect.right = 799;
	}

	if (pr_rect_cur_frm->obj_rect[0].pr_rect.right >= image_w)
	{
		pr_rect_cur_frm->obj_rect[0].pr_rect.right = image_w - 1;
		pr_rect_cur_frm->obj_rect[0].pr_rect.left = image_w - 800;
	}
	return;
}

/**************************************************************************************************
* 功 能：计算牌识区域
* 参 数：*
*         pr_handle              - I    牌识部分模块句柄
*         frame_header           - I    帧头信息
*         module_interface       - O    牌识公共接口
*         frm_data               - I    图像帧数据
* 返回值：状态码
* 备 注：无
***************************************************************************************************/
HRESULT VTS_MPR_calc_pr_rect_MX(void                    *pr_handle,
								VCA_FRAME_HEADER        *frame_header,
								MPR_MODULE_INTERFACE    *module_interface)
{
	HRESULT                rtn = HIK_MPR_S_OK;
	MPR_PR_HANDLE         *mpr_pr_hdl = (MPR_PR_HANDLE *)pr_handle;    // 获取当前句柄
	MPR_OBJ_PR_RECT_ARRAY *pr_rect_cur_frm = NULL;                          // 当前帧牌识区域数据
	MPR_OBJ_PR_RECT_ARRAY *pr_rect_last_frm = NULL;                          // 上一帧牌识区域数据

	int                    time_dis = 0, n = 0, flag_super_big = 0;

	// 初始化
	pr_rect_cur_frm = &mpr_pr_hdl->pr_rect_cur;
	pr_rect_last_frm = &mpr_pr_hdl->pr_rect_last;
	CHECK_ERROR(NULL == mpr_pr_hdl ,HIK_LPR_ITF_PTR_NULL);


	// 上一帧牌识区域清零
	memcpy(pr_rect_last_frm, pr_rect_cur_frm, sizeof(MPR_OBJ_PR_RECT_ARRAY));
	memset(pr_rect_cur_frm, 0, sizeof(MPR_OBJ_PR_RECT_ARRAY));
	memset(mpr_pr_hdl->pr_rect_in, 0, sizeof(VCA_OBJ_INFO)* ITS_MAX_VEH_NUM);
	mpr_pr_hdl->pr_in_index = 0;
	//*(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 32;

	// 1、获取帧率信息
	rtn = VTS_MPR_get_frame_rate(mpr_pr_hdl, frame_header, &module_interface->frm_rate);
	CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
	//*(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 33;
	PRTF("module_interface->is_cap_frm boool = %d\n",module_interface->is_cap_frm);

	//2、历史车牌预测
	if (module_interface->is_cap_frm == FALSE && module_interface->is_manual_frm == FALSE)
	//if (0)//测试图片xdm 关掉历史车牌预测
	{
		rtn = VTS_MPR_pr_rect_from_old_plate(mpr_pr_hdl,
			frame_header,
			mpr_pr_hdl->full_image_width,
			mpr_pr_hdl->full_image_height,
			pr_rect_cur_frm,
			module_interface,
			&flag_super_big);
		CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
		//*(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 34;
	}

	// 获取牌识共享数据
	rtn = MPR_get_pr_share_info(&module_interface->id_queue, pr_rect_cur_frm);
	CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);

	//*(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 35;

	// 3、动态牌识区域切分
	rtn = VTS_MPR_pr_rect_cut(mpr_pr_hdl,
		mpr_pr_hdl->full_image_width,
		mpr_pr_hdl->full_image_height,
		pr_rect_cur_frm,
		module_interface->pr_up_line,
		module_interface->plate_w_ave,
		module_interface->is_cap_frm);
	CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
	//*(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 36;

	// 4、牌识区域优先级排序
	if (mpr_pr_hdl->stop_sort_flag != 1)
	{
		rtn = VTS_MPR_rect_pr_sort(pr_rect_last_frm, pr_rect_cur_frm);
		CHECK_ERROR(rtn != HIK_MPR_S_OK, rtn);
		//*(mpr_pr_hdl->debug_status) = GDW_MPR_DEBUG_STATUS + 37;
	}

	// 对计算的牌识区域进行简单的过滤、合并处理raw
	VTS_MPR_check_obj_pr_rect_data(pr_rect_last_frm, pr_rect_cur_frm);



	// 根据拼图策略和规则过滤牌识框
	//挑选出一个离卡口触发线最近的牌识区域
	if (module_interface->product_type == 4)
	{
		VTS_MPR_select_obj_pr_rect_data(pr_rect_last_frm, pr_rect_cur_frm, module_interface->snap_line, mpr_pr_hdl->full_image_width,
			mpr_pr_hdl->full_image_height,module_interface->mpr_mode,module_interface->lanes_list, mpr_pr_hdl->dir_flag);
		//爆闪模式下，没有牌识区域，建立默认牌识区域
		if (pr_rect_cur_frm->pr_num == 0)
		{
			if (module_interface->is_cap_frm == TRUE || module_interface->is_manual_frm == TRUE)
			{
				/*VTS_MPR_Create_pr_rect(pr_rect_cur_frm, module_interface->snap_line, module_interface->lane_num, module_interface->lanes_list, mpr_pr_hdl->full_image_width,
				mpr_pr_hdl->full_image_height);*/
				pr_rect_cur_frm->obj_rect[0].pr_rect.bottom = module_interface->extre_rect->bottom;
				pr_rect_cur_frm->obj_rect[0].pr_rect.left = module_interface->extre_rect->left;
				pr_rect_cur_frm->obj_rect[0].pr_rect.top = module_interface->extre_rect->top;
				pr_rect_cur_frm->obj_rect[0].pr_rect.right = module_interface->extre_rect->right;
				pr_rect_cur_frm->pr_num = 1;
                                module_interface->last_pr_frm = frame_header->frame_num;
			}
			else if (frame_header->frame_num - module_interface->last_pr_frm >= 4)
                        {
                            pr_rect_cur_frm->pr_num = 1;
                            pr_rect_cur_frm->obj_rect[0].pr_rect.left = module_interface->extre_rect->left;
                            pr_rect_cur_frm->obj_rect[0].pr_rect.top = module_interface->extre_rect->top;
                            pr_rect_cur_frm->obj_rect[0].pr_rect.right = module_interface->extre_rect->right;
                            pr_rect_cur_frm->obj_rect[0].pr_rect.bottom = module_interface->extre_rect->bottom;
                            module_interface->last_pr_frm = frame_header->frame_num;
                        }

		/*	else
			{
				pr_rect_cur_frm->obj_rect[0].pr_rect.bottom = mpr_pr_hdl->full_image_height / 2 + 300;
				pr_rect_cur_frm->obj_rect[0].pr_rect.left = mpr_pr_hdl->full_image_width / 2 -  400;
				pr_rect_cur_frm->obj_rect[0].pr_rect.top = mpr_pr_hdl->full_image_height / 2 - 300;
				pr_rect_cur_frm->obj_rect[0].pr_rect.right = mpr_pr_hdl->full_image_height / 2 + 400;
				pr_rect_cur_frm->pr_num = 1;
			}*/
		}
		else
		{
		    module_interface->last_pr_frm = frame_header->frame_num;
		}
		
	}
	
	memcpy(&module_interface->pr_objs_cur_frm, pr_rect_cur_frm, sizeof(MPR_OBJ_PR_RECT_ARRAY));

	return HIK_MPR_S_OK;
}


HRESULT VTS_MPR_get_pr_area_result(MPR_OBJ_PR_RECT_ARRAY *pr_objs_cur_frm,
								   int					  img_w,
								   int					  img_h,
								   VCA_OBJ_LIST			 *lpr_obj_list,
								   int                    product_type)
{
	int i, offset;
	VCA_BOX_S *pr_rect = NULL;
	int              image_w = 0;
	int              image_h = 0;

	if (product_type == 2)
	{
		image_w = MAX_ORIGIN_STITCH_WIDTH - 32;
		image_h = MAX_ORIGIN_STITCH_HEIGHT - 32;
	}
	else if (product_type == 4)
	{
        if (img_w > MAX_VET_WIDTH && img_h > MAX_VET_HEIGHT)
        {
            image_w = MAX_VET_WIDTH;
            image_h = MAX_VET_HEIGHT;
        }
        else
        {
            image_w = img_w;
            image_h = img_h;
        }
		
	}
	else
	{
		return HIK_MPR_FLW_PARAM_ERR;
	}

	lpr_obj_list->obj_num = pr_objs_cur_frm->pr_num;

	for (i = 0; i < pr_objs_cur_frm->pr_num; ++i)
	{
		pr_rect = &pr_objs_cur_frm->obj_rect[i].pr_rect;

		///限制牌识区域宽高，不能超过拼图尺寸
		if (pr_rect->bottom - pr_rect->top >= image_h)
		{
			offset = pr_rect->bottom - pr_rect->top - image_h;
			pr_rect->top += offset;
		}

		if (pr_rect->right - pr_rect->left >= image_w)
		{
			offset = pr_rect->right - pr_rect->left - image_w;
			pr_rect->left += offset / 2;
			pr_rect->right -= offset / 2;
		}

		lpr_obj_list->obj[i].rect.x = pr_rect->left / (float)img_w;
		lpr_obj_list->obj[i].rect.y = pr_rect->top / (float)img_h;
		lpr_obj_list->obj[i].rect.width = (pr_rect->right - pr_rect->left) / (float)img_w;
		lpr_obj_list->obj[i].rect.height = (pr_rect->bottom - pr_rect->top) / (float)img_h;

		lpr_obj_list->obj[i].id = i + 1;
	}

	return HIK_MPR_S_OK;
}