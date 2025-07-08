/*
 * min_process.c
 *
 *  Created on: May 5, 2025
 *      Author: HTSANG
 */

#include "min_process.h"
#include "scheduler.h"
#include "main.h"
#include "uart_driver.h"
#include "board.h"

MIN_Context_t EXP_MinCtx;


/* Private typedef -----------------------------------------------------------*/
typedef struct _MIN_TaskContextTypedef_ {
	SCH_TASK_HANDLE taskHandle;
	SCH_TaskPropertyTypedef taskProperty;
} MIN_TaskContextTypedef;

/* Private function ----------------------------------------------------------*/
static void MIN_Processing(void);

void MIN_Timeout_Handler(MIN_Context_t *ctx) {
//    LOG("MIN-Timeout!");
	return;
}

static MIN_TaskContextTypedef s_MINTaskContext = {
		SCH_INVALID_TASK_HANDLE, 	// Will be updated by Schedular
		{ SCH_TASK_SYNC,        	// taskType;
		SCH_TASK_PRIO_0,         	// taskPriority;
		10,                      	// taskPeriodInMS;
		MIN_Processing, 			// taskFunction;
		9 }
};

void MIN_Process_Init(void){
	MIN_Context_Init(&EXP_MinCtx, OBC_EXP_PORT);
	MIN_RegisterTimeoutCallback(&EXP_MinCtx, MIN_Timeout_Handler);
}

void MIN_Processing(void){
	while (UART_Driver_IsDataAvailable(EXP_UART_OBC_HANDLE)) {
        int data = UART_Driver_Read(EXP_UART_OBC_HANDLE);
        if (data >= 0) {
            uint8_t byte = (uint8_t)data;
            MIN_App_Poll(&EXP_MinCtx, &byte, 1);
        }
	}
	MIN_App_Poll(&EXP_MinCtx, NULL, 0);
}

void MIN_CreateTask(void) {
	SCH_TASK_CreateTask(&s_MINTaskContext.taskHandle, &s_MINTaskContext.taskProperty);
}







