/*
 * min_process.h
 *
 *  Created on: May 5, 2025
 *      Author: HTSANG
 */

#ifndef MIN_PROTO_MIN_PROCESS_H_
#define MIN_PROTO_MIN_PROCESS_H_

#include "min_app/min_app.h"

extern MIN_Context_t EXP_MinCtx;

void MIN_Process_Init(void);
//void MIN_Processing(void);

void MIN_CreateTask(void);

#endif /* MIN_PROTO_MIN_PROCESS_H_ */
