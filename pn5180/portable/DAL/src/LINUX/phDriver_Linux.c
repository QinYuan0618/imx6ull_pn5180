/*----------------------------------------------------------------------------*/
/* Copyright 2017-2022 NXP                                                    */
/*                                                                            */
/* NXP Confidential. This software is owned or controlled by NXP and may only */
/* be used strictly in accordance with the applicable license terms.          */
/* By expressly accepting such terms or by downloading, installing,           */
/* activating and/or otherwise using the software, you are agreeing that you  */
/* have read, and that you agree to comply with and are bound by, such        */
/* license terms. If you do not agree to be bound by the applicable license   */
/* terms, then you may not retain, install, activate or otherwise use the     */
/* software.                                                                  */
/*----------------------------------------------------------------------------*/

/** \file
* Generic phDriver Component of Reader Library Framework.
* $Author$		qinyuan
* $Revision$	v1.0
* $Date$		2025/07/23
*
*/

#include "phDriver.h"
#include "BoardSelection.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <time.h>
#include <sys/time.h>
#include <sys/timerfd.h>
#include <sys/epoll.h>
#include <poll.h>
#include <pthread.h>
#include <gpiod.h>

#define LPC_TIMER_MAX_32BIT            0xFFFFFFFFU
#define LPC_TIMER_MAX_32BIT_W_PRE      0xFFFFFFFE00000000U /* max 32bit x 32bit. */

static pphDriver_TimerCallBck_t pTimerIsrCallBack;
static volatile uint32_t dwTimerExp;

static void phDriver_TimerIsrCallBack(void);

/* add static arguments */
static int  			timerfd = -1;
static int  			epollfd = -1;
static pthread_t 		timer_thread;
static vloatile int 	timer_active = 0;
static volatile int 	thread_should_exit = 0;
static pthread_mutex_t 	timer_mutex = PTHREAD_MUTEX_INITIALIZER;

/*
 * GPIO上下文结构体 - 用于管理libgpiod资源
 * 每个GPIO引脚需要独立的chip、settings、config和request
 */
typedef struct {
    struct gpiod_chip *chip;              /* GPIO芯片句柄 */
    struct gpiod_line_request *request;   /* GPIO线请求句柄 */
    struct gpiod_line_settings *settings; /* GPIO线设置 */
    struct gpiod_line_config *line_cfg;   /* GPIO线配置 */
    struct gpiod_request_config *req_cfg; /* GPIO请求配置 */
    uint32_t gpio_num;                    /* 全局GPIO编号 */
    int chip_num;                         /* 芯片编号 */
    int line_num;                         /* 线编号 */
    int is_initialized;                   /* 初始化标志 */
	volatile int interrupt_pending;	      /* 软件:中断标志*/
} gpio_context_t;

/* 预定义的GPIO上下文 - 对应板级配置中的引脚 */
static gpio_context_t gpio_reset = {0};     /* 复位引脚：GPIO编号87 */
static gpio_context_t gpio_irq = {0};       /* 中断引脚：GPIO编号10 */
static gpio_context_t gpio_busy = {0};      /* 忙碌引脚：GPIO编号88 */

/* 私有函数声明 */
static void *phDriver_TimerThread(void *arg);
static int phDriver_GpioToChipLine(uint32_t gpio_num, int *chip_num, int *line_num);
static gpio_context_t* phDriver_GetGpioContext(uint32_t dwPinNumber);
static int phDriver_InitGpioContext(gpio_context_t *ctx, uint32_t gpio_num, int direction);
static void phDriver_DeinitGpioContext(gpio_context_t *ctx);

/* ******************************************************************/
/*  						      新增函数					    	*/
/* **************************************************************** */

/* 
 * GPIO编号转换函数：将全局GPIO编号转换成 gpiochip编号 和 line编号
 * GPIO编号 = (控制器编号-1) * 32 + IO编号 
 */
static phDriver_GpioToChipLine(uint32_t gpio_num, int *chip_num, int *line_num)
{
	/*
     * 基于引脚配置进行映射：
     * GPIO10 (GPIO1_IO10) -> chip0, line10
     * GPIO87 (GPIO3_IO23) -> chip2, line23
     * GPIO88 (GPIO3_IO24) -> chip2, line24
     *
     * 注意：这个映射关系需要根据实际系统调整
     * 可以通过 gpioinfo 命令查看实际的chip和line对应关系
     */

    if (gpio_num == PHDRIVER_PIN_IRQ)
	{
        /* GPIO1_IO10 -> 假设在chip0 */
        *chip_num = 0;
        *line_num = 10;
        return 0;
    }
	else if (gpio_num == PHDRIVER_PIN_RESET)
	{
        /* GPIO3_IO23 -> 假设在chip2 */
        *chip_num = 2;
        *line_num = 23;
        return 0;
    } 
	else if (gpio_num == PHDRIVER_PIN_BUSY) 
	{
        /* GPIO3_IO24 -> 假设在chip2 */
        *chip_num = 2;
        *line_num = 24;
        return 0;
    }

    return -1; /* 未知的GPIO编号 */
}

/* 根据GPIO编号获取对应的上下文 */
static gpio_context_t* phDriver_GetGpioContext(uint32_t dwPinNumber)
{
    if (dwPinNumber == PHDRIVER_PIN_RESET)
	{
        return &gpio_reset;
    }
	else if (dwPinNumber == PHDRIVER_PIN_IRQ)
	{
        return &gpio_irq;
    } 
	else if (dwPinNumber == PHDRIVER_PIN_BUSY) 
	{
        return &gpio_busy;
    }

    return NULL;
}

static gpio_context_t* phDriver_GetGpioContext(uint32_t dwPinNumber)
{
    if (dwPinNumber == PHDRIVER_PIN_RESET) {
        return &gpio_reset;
    } else if (dwPinNumber == PHDRIVER_PIN_IRQ) {
        return &gpio_irq;
    } else if (dwPinNumber == PHDRIVER_PIN_BUSY) {
        return &gpio_busy;
    }
    return NULL;
}

/*
 * 反初始化GPIO上下文 - 简化版本
 */
static void phDriver_DeinitGpioContext(gpio_context_t *ctx)
{
    if (!ctx || !ctx->is_initialized) {
        return;
    }

    if (ctx->request) {
        gpiod_line_request_release(ctx->request);
    }
    if (ctx->req_cfg) {
        gpiod_request_config_free(ctx->req_cfg);
    }
    if (ctx->line_cfg) {
        gpiod_line_config_free(ctx->line_cfg);
    }
    if (ctx->settings) {
        gpiod_line_settings_free(ctx->settings);
    }
    if (ctx->chip) {
        gpiod_chip_close(ctx->chip);
    }

    /* 清除软件状态 */
    ctx->interrupt_pending = 0;

    memset(ctx, 0, sizeof(gpio_context_t));
}

/* ******************************************************************/
/*  							原有接口函数						*/
/* **************************************************************** */

/* 定时器启动函数 - timerfd + epoll 实现 */
phStatus_t phDriver_TimerStart(phDriver_Timer_Unit_t eTimerUnit, uint32_t dwTimePeriod, pphDriver_TimerCallBck_t pTimerCallBack)
{
	struct itimerspec timer_spec;
    struct epoll_event ev;
    
    pthread_mutex_lock(&timer_mutex);

    if(pTimerCallBack == NULL)
    {
        dwTimerExp = 0;
        pTimerIsrCallBack = phDriver_TimerIsrCallBack;
    }else
    {   
        pTimerIsrCallBack = pTimerCallBack;
    }

    if (timerfd == -1) {
        timerfd = timerfd_create(CLOCK_REALTIME, TFD_CLOEXEC);
        if (timerfd == -1) {
            pthread_mutex_unlock(&timer_mutex);
            return PH_DRIVER_ERROR | PH_COMP_DRIVER;
        }
        
        epollfd = epoll_create1(EPOLL_CLOEXEC);
        if (epollfd == -1) {
            close(timerfd);
            timerfd = -1;
            pthread_mutex_unlock(&timer_mutex);
            return PH_DRIVER_ERROR | PH_COMP_DRIVER;
        }
        
        ev.events = EPOLLIN;
        ev.data.fd = timerfd;
        if (epoll_ctl(epollfd, EPOLL_CTL_ADD, timerfd, &ev) == -1) {
            close(epollfd);
            close(timerfd);
            timerfd = -1;
            epollfd = -1;
            pthread_mutex_unlock(&timer_mutex);
            return PH_DRIVER_ERROR | PH_COMP_DRIVER;
        }
        
        thread_should_exit = 0;
        if (pthread_create(&timer_thread, NULL, phDriver_TimerThread, NULL) != 0) {
            close(epollfd);
            close(timerfd);
            timerfd = -1;
            epollfd = -1;
            pthread_mutex_unlock(&timer_mutex);
            return PH_DRIVER_ERROR | PH_COMP_DRIVER;
        }
    }

    memset(&timer_spec, 0, sizeof(timer_spec));
    
    switch(eTimerUnit) {
        case PH_DRIVER_TIMER_SECS:
            timer_spec.it_value.tv_sec = dwTimePeriod;
            timer_spec.it_value.tv_nsec = 0;
            break;
        case PH_DRIVER_TIMER_MILLI_SECS:
            timer_spec.it_value.tv_sec = dwTimePeriod / 1000;
            timer_spec.it_value.tv_nsec = (dwTimePeriod % 1000) * 1000000;
            break;
        case PH_DRIVER_TIMER_MICRO_SECS:
            timer_spec.it_value.tv_sec = dwTimePeriod / 1000000;
            timer_spec.it_value.tv_nsec = (dwTimePeriod % 1000000) * 1000;
            break;
        default:
            pthread_mutex_unlock(&timer_mutex);
            return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    if (timerfd_settime(timerfd, 0, &timer_spec, NULL) == -1) {
        pthread_mutex_unlock(&timer_mutex);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    timer_active = 1;
    pthread_mutex_unlock(&timer_mutex);

    if(pTimerCallBack == NULL)
    {
        while (timer_active) {
            usleep(1000);
        }
    }

    return PH_DRIVER_SUCCESS;
}

phStatus_t phDriver_TimerStop(void)
{
    struct itimerspec timer_spec;
    
    pthread_mutex_lock(&timer_mutex);
    
    if (timerfd != -1 && timer_active) {
        memset(&timer_spec, 0, sizeof(timer_spec));
        timerfd_settime(timerfd, 0, &timer_spec, NULL);
        timer_active = 0;
    }
    
    pthread_mutex_unlock(&timer_mutex);
    return PH_DRIVER_SUCCESS;
}

/*
 * GPIO中断配置
 */
static phStatus_t phDriver_PinConfigInterrupt(uint32_t dwPinNumber, phDriver_Pin_Config_t *pPinConfig)
{
    gpio_context_t *ctx = phDriver_GetGpioContext(dwPinNumber);
    char 			chip_dev[32];
    int 			chip_num, line_num;
    unsigned int 	offset;
    
    if (!ctx || !pPinConfig) {
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }
    
    /* 如果已经初始化过，先清理 */
    if (ctx->is_initialized) {
        phDriver_DeinitGpioContext(ctx);
    }
    
    /* 将全局GPIO编号转换为chip+line */
    if (phDriver_GpioToChipLine(dwPinNumber, &chip_num, &line_num) < 0) {
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }
    
    ctx->gpio_num = dwPinNumber;
    ctx->chip_num = chip_num;
    ctx->line_num = line_num;
    
    /* 1. 打开GPIO芯片 */
    snprintf(chip_dev, sizeof(chip_dev), "/dev/gpiochip%d", chip_num);
    ctx->chip = gpiod_chip_open(chip_dev);
    if (!ctx->chip) {
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }
    
    /* 2. 创建 line_settings 结构体, 设置单个GPIO的行为 */
    ctx->settings = gpiod_line_settings_new(); 
    if (!ctx->settings) {
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }
    
    /* 2.1 配置GPIO为输入模式 */
    gpiod_line_settings_set_direction(ctx->settings, GPIOD_LINE_DIRECTION_INPUT);
    
    /* 2.2 配置上拉/下拉电阻 */
    if (pPinConfig->bPullSelect == PH_DRIVER_PULL_DOWN)
	{
        gpiod_line_settings_set_bias(ctx->settings, GPIOD_LINE_BIAS_PULL_DOWN);
    }
	else
	{
        gpiod_line_settings_set_bias(ctx->settings, GPIOD_LINE_BIAS_PULL_UP);
    }
    
    /* 2.3 配置中断边沿检测 */
    switch(pPinConfig->eInterruptConfig)
    {
    case PH_DRIVER_INTERRUPT_RISINGEDGE:
        gpiod_line_settings_set_edge_detection(ctx->settings, GPIOD_LINE_EDGE_RISING);
        break;
    case PH_DRIVER_INTERRUPT_FALLINGEDGE:
        gpiod_line_settings_set_edge_detection(ctx->settings, GPIOD_LINE_EDGE_FALLING);
        break;
    case PH_DRIVER_INTERRUPT_EITHEREDGE:
        gpiod_line_settings_set_edge_detection(ctx->settings, GPIOD_LINE_EDGE_BOTH);
        break;
    default:
        /* 不配置边沿检测，仅作为普通输入 */
        break;
    }
    
    /* 3. 创建 line_config 结构体，管理settings绑定到指定的line */
    ctx->line_cfg = gpiod_line_config_new();
    if (!ctx->line_cfg)
	{
        gpiod_line_settings_free(ctx->settings);
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }
    
    /* 3.1 将settings设置添加到配置line中 */
    offset = line_num;
    if (gpiod_line_config_add_line_settings(ctx->line_cfg, &offset, 1, ctx->settings) < 0) 
	{
        gpiod_line_config_free(ctx->line_cfg);
        gpiod_line_settings_free(ctx->settings);
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }
    
    /* 4. 创建 request_condig 请求配置, 配置consumer名称 "pn5180-nfc-irq" 便于调试 */
    ctx->req_cfg = gpiod_request_config_new();
    if (!ctx->req_cfg)
	{
        gpiod_line_config_free(ctx->line_cfg);
        gpiod_line_settings_free(ctx->settings);
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }
    
    gpiod_request_config_set_consumer(ctx->req_cfg, "pn5180-nfc-irq");
    
    /* 5. 申请GIPO资源，所有配置好后，向chip发出请求 */
    ctx->request = gpiod_chip_request_lines(ctx->chip, ctx->req_cfg, ctx->line_cfg);
    if (!ctx->request) {
        gpiod_request_config_free(ctx->req_cfg);
        gpiod_line_config_free(ctx->line_cfg);
        gpiod_line_settings_free(ctx->settings);
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }
    
    ctx->is_initialized = 1;
    return PH_DRIVER_SUCCESS;
}

/* 
 * 普通GPIO配置
 * 配置GPIO为输入或者输出模式
 */
static phStatus_t phDriver_PinConfigGpio(uint32_t dwPinNumber, phDriver_Pin_Func_t ePinFunc,
        phDriver_Pin_Config_t *pPinConfig)
{
    gpio_context_t *ctx = phDriver_GetGpioContext(dwPinNumber);
    char chip_dev[32];
    int chip_num, line_num;
    unsigned int offset;

    if (!ctx || !pPinConfig) {
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    /* 如果已经初始化过，先清理 */
    if (ctx->is_initialized) {
        phDriver_DeinitGpioContext(ctx);
    }

    /* 将全局GPIO编号转换为chip+line */
    if (phDriver_GpioToChipLine(dwPinNumber, &chip_num, &line_num) < 0) {
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    ctx->gpio_num = dwPinNumber;
    ctx->chip_num = chip_num;
    ctx->line_num = line_num;
    ctx->interrupt_pending = 0;

    /* 1. 打开GPIO芯片 */
    snprintf(chip_dev, sizeof(chip_dev), "/dev/gpiochip%d", chip_num);
    ctx->chip = gpiod_chip_open(chip_dev);
    if (!ctx->chip) {
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    /* 2. 创建线设置结构体 */
    ctx->settings = gpiod_line_settings_new();
    if (!ctx->settings) {
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    /* 3. 根据功能类型配置GPIO */
    switch(ePinFunc)
    {
    case PH_DRIVER_PINFUNC_INPUT:
        /* 配置为输入模式 */
        gpiod_line_settings_set_direction(ctx->settings, GPIOD_LINE_DIRECTION_INPUT);
        /* 配置上拉/下拉电阻 */
        if (pPinConfig->bPullSelect == PH_DRIVER_PULL_DOWN) {
            gpiod_line_settings_set_bias(ctx->settings, GPIOD_LINE_BIAS_PULL_DOWN);
        } else {
            gpiod_line_settings_set_bias(ctx->settings, GPIOD_LINE_BIAS_PULL_UP);
        }
        break;

    case PH_DRIVER_PINFUNC_OUTPUT:
        /* 配置为输出模式 */
        gpiod_line_settings_set_direction(ctx->settings, GPIOD_LINE_DIRECTION_OUTPUT);
        gpiod_line_settings_set_output_value(ctx->settings,
            pPinConfig->bOutputLogic ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
        break;

    default:
        gpiod_line_settings_free(ctx->settings);
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    /* 4. 创建线配置 */
    ctx->line_cfg = gpiod_line_config_new();
    if (!ctx->line_cfg) {
        gpiod_line_settings_free(ctx->settings);
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    /* 5. 将线设置添加到配置中 */
    offset = line_num;
    if (gpiod_line_config_add_line_settings(ctx->line_cfg, &offset, 1, ctx->settings) < 0) {
        gpiod_line_config_free(ctx->line_cfg);
        gpiod_line_settings_free(ctx->settings);
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    /* 6. 创建请求配置 */
    ctx->req_cfg = gpiod_request_config_new();
    if (!ctx->req_cfg) {
        gpiod_line_config_free(ctx->line_cfg);
        gpiod_line_settings_free(ctx->settings);
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    /* 普通GPIO使用通用的consumer名称 */
    gpiod_request_config_set_consumer(ctx->req_cfg, "pn5180-nfc");

    /* 7. 请求GPIO线 */
    ctx->request = gpiod_chip_request_lines(ctx->chip, ctx->req_cfg, ctx->line_cfg);
    if (!ctx->request) {
        gpiod_request_config_free(ctx->req_cfg);
        gpiod_line_config_free(ctx->line_cfg);
        gpiod_line_settings_free(ctx->settings);
        gpiod_chip_close(ctx->chip);
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

    ctx->is_initialized = 1;
    return PH_DRIVER_SUCCESS;
}

/* GPIO 配置入口函数 */
phStatus_t phDriver_PinConfig(uint32_t dwPinNumber, phDriver_Pin_Func_t ePinFunc, phDriver_Pin_Config_t *pPinConfig)
{
    phStatus_t wStatus;

    do{
        if(pPinConfig == NULL)
        {
            wStatus = PH_DRIVER_ERROR | PH_COMP_DRIVER;
            break;
        }
		
		/* irq */
        if(ePinFunc == PH_DRIVER_PINFUNC_INTERRUPT)
        {
            /* libgpiod不支持电平触发中断，只支持边沿触发 */
            if((pPinConfig->eInterruptConfig == PH_DRIVER_INTERRUPT_LEVELZERO) ||
                    (pPinConfig->eInterruptConfig == PH_DRIVER_INTERRUPT_LEVELONE))
            {
                wStatus = PH_DRIVER_ERROR | PH_COMP_DRIVER;
            }
            else
            {
                wStatus = phDriver_PinConfigInterrupt(dwPinNumber, pPinConfig);
            }
        }
        else /* basic gpio */
        {
            wStatus = phDriver_PinConfigGpio(dwPinNumber, ePinFunc, pPinConfig);
        }
    }while(0);

    return wStatus;
}

/* 两种模式：
 * 1.中断状态检测
 * 2.GPIO电平读取 
 */
uint8_t phDriver_PinRead(uint32_t dwPinNumber, phDriver_Pin_Func_t ePinFunc)
{
	gpio_context_t *ctx = phDriver_GetGpioContext(dwPinNumber);
	uint8_t			bPinStatus = 0;

	if(!ctx || !ctx->is_initialized)
	{
		return 0;
	}

	/* 模式1：irq: 检查是否有边沿事件 */
    if(ePinFunc == PH_DRIVER_PINFUNC_INTERRUPT)
    {
		/* 检查软件中断标志 */
		if(ctx->interrupt_pending)
		{
			return 1;
		}

		/* 检查有没有新事件发生 */
		int fd = gpoid_line_request_get_fd(ctx->request);
		if(fd >= 0)
		{
			struct pollfd pfd = 
			{
				.fd = fd,
				.events = POLLIN
			}
			
			int ret = poll(&pfd, 1, 0);
			if(ret>0 && (pfd.revents & POLLIN))
			{
				ctx->interrupt = 1;
				bPinStatus = 1;
			}
		}
	}
	else
	{
		 /* 模式2——普通GPIO模式：读取实际引脚电平 */
        enum gpiod_line_value value = gpiod_line_request_get_value(ctx->request, ctx->line_num);
        bPinStatus = (value == GPIOD_LINE_VALUE_ACTIVE) ? 1 : 0;
	}

    return bPinStatus;
}

/* 
 * GPIO中断轮询
 * 轮询等待GPIO状态变化
 */
phStatus_t phDriver_IRQPinPoll(uint32_t dwPinNumber, phDriver_Pin_Func_t ePinFunc, phDriver_Interrupt_Config_t eInterruptType)
{
    uint8_t    bGpioState = 0;

    if ((eInterruptType != PH_DRIVER_INTERRUPT_RISINGEDGE) && (eInterruptType != PH_DRIVER_INTERRUPT_FALLINGEDGE))
    {
        return PH_DRIVER_ERROR | PH_COMP_DRIVER;
    }

	/* 等待电平：高->低 */
    if (eInterruptType == PH_DRIVER_INTERRUPT_FALLINGEDGE)
    {
        bGpioState = 1;
    }

	/* 轮询等待状态变化：标志位 */
	while(phDriver_PinRead(dwPinNumber, ePinFunc) == bGpioState);

    return PH_DRIVER_SUCCESS;
}

/* 写入GPIO电平 */
void phDriver_PinWrite(uint32_t dwPinNumber, uint8_t bValue)
{
	gpio_context_t *ctx = phDriver_GetGpioContext(dwPinNumber);
    
    if (!ctx || !ctx->is_initialized) {
        return;
    }
    
    enum gpiod_line_value value = bValue ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE;
    gpiod_line_request_set_value(ctx->request, ctx->line_num, value);
}

/* 清除中断状态
 * 完整模拟NXP的寄存器清除操作
 */
void phDriver_PinClearIntStatus(uint32_t dwPinNumber)
{
	gpio_context_t *ctx = phDriver_GetGpioContext(dwPinNumber);
    
    if (!ctx || !ctx->is_initialized) {
        return;
    }
    
    /* 
     * 模拟NXP的 pGPIOINT->IO0.CLR = pins 操作
     * 需要同时清除软件标志和硬件事件队列
     */
    
    /* 第一步：清除软件中断标志（模拟硬件寄存器位清零） */
    ctx->interrupt_pending = 0;
    
    /* 第二步：清空内核事件队列（确保没有残留事件） */
    struct gpiod_edge_event_buffer *event_buffer = gpiod_edge_event_buffer_new(16);
    if (event_buffer) {
        int ret;
        /* 连续读取直到没有事件，相当于清空中断状态寄存器 */
        do
		{
            ret = gpiod_line_request_read_edge_events(ctx->request, event_buffer, 16);
        } while (ret > 0);
        
        gpiod_edge_event_buffer_free(event_buffer);
    }
}

void PH_DRIVER_LPC_TIMER_IRQ_HANDLER(void)
{
    Chip_TIMER_ClearMatch(PH_DRIVER_LPC_TIMER, PH_DRIVER_LPC_TIMER_MATCH_REGISTER);

    pTimerIsrCallBack();

    Chip_TIMER_Disable(PH_DRIVER_LPC_TIMER);
    Chip_TIMER_DeInit(PH_DRIVER_LPC_TIMER);

    /* Disable timer interrupt */
    NVIC_DisableIRQ(PH_DRIVER_LPC_TIMER_IRQ);
}

/* 默认的定时器回调函数 */
static void phDriver_TimerIsrCallBack(void)
{
    dwTimerExp = 1;
}

void phDriver_EnterCriticalSection(void)
{
//    __disable_irq();
}

void phDriver_ExitCriticalSection(void)
{
//    __enable_irq();
}

/* 读取IRQ引脚状态 */
phStatus_t phDriver_IRQPinRead(uint32_t dwPinNumber)
{
	phStatus_t bGpioVal = false;

	bGpioVal = phDriver_PinRead(dwPinNumber, PH_DRIVER_PINFUNC_INPUT);

	return bGpioVal;
}

