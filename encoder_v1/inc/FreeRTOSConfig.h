/*
 * FreeRTOS V202107.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* Scheduler Related */
#define configUSE_PREEMPTION                    1 // 1 preemptive RTOS scheduler            0 cooperative RTOS scheduler
#define configUSE_TICKLESS_IDLE                 0 // 1 low power tickless mode              0 to keep the tick interrupt running
#define configUSE_IDLE_HOOK                     0 // 1 if you wish to use an idle hook      0 to omit an idle hook
#define configUSE_MALLOC_FAILED_HOOK            0 // 1 app must define a malloc() failed hook       0 malloc() failed hook function will not be called
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0 // 1 app must define a daemon task startup hook   0 daemon task startup hook will not be calles
#define configUSE_TICK_HOOK                     0 // 1 if you wish to use a tick hook      0 to omit a tick hook.
#define configCPU_CLOCK_HZ                      48000000 // frequency in Hz at which the internal clock that drives the peripheral used to generate the tick interrupt will be executing
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 ) //  Frequency of the RTOS tick interrupt.
#define configMAX_PRIORITIES                    32 // The number of priorities available to the application tasks
#define configMINIMAL_STACK_SIZE                ( configSTACK_DEPTH_TYPE ) 256 //The number of priorities available to the application tasks
#define configMAX_TASK_NAME_LEN                 10 // The maximum permissible length of the descriptive name given to a task when the task is created
#define configUSE_16_BIT_TICKS                  0

#define configIDLE_SHOULD_YIELD                 1

/* Synchronization Related */
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_APPLICATION_TASK_TAG          0
#define configUSE_COUNTING_SEMAPHORES           1
#define configQUEUE_REGISTRY_SIZE               8
#define configUSE_QUEUE_SETS                    1
#define configUSE_TIME_SLICING                  1
#define configUSE_NEWLIB_REENTRANT              0
#define configENABLE_BACKWARD_COMPATIBILITY     0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS 5

/* System */
#define configSTACK_DEPTH_TYPE                  uint32_t
#define configMESSAGE_BUFFER_LENGTH_TYPE        size_t

/* Memory allocation related definitions. */
#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configTOTAL_HEAP_SIZE                   (128*1024)
#define configAPPLICATION_ALLOCATED_HEAP        0

/* Hook function related definitions. */
#define configCHECK_FOR_STACK_OVERFLOW          0

/* Run time and task stats gathering related definitions. */
#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                1
#define configUSE_STATS_FORMATTING_FUNCTIONS    0

/* Co-routine related definitions. */
#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         1

/* Software timer related definitions. */
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( configMAX_PRIORITIES - 1 )
#define configTIMER_QUEUE_LENGTH                10
#define configTIMER_TASK_STACK_DEPTH            1024

/* Interrupt nesting behaviour configuration. */
/*
#define configKERNEL_INTERRUPT_PRIORITY         [dependent of processor]
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    [dependent on processor and application]
#define configMAX_API_CALL_INTERRUPT_PRIORITY   [dependent on processor and application]
*/

/* SMP port only */
#define configNUM_CORES                         2 // Sets the number of available processor cores
#define configTICK_CORE                         0 // indicates which core should handle the SysTick
#define configRUN_MULTIPLE_PRIORITIES           1 // 0 multiple equal priority tasks...  1 multiple tasks with different priorities...  ...may run simultaneously
#define configUSE_CORE_AFFINITY                 1 // 0 scheduler execute tasks anywhere, 1 user controls in wich core a task execute 'vTaskCoreAffinitySet'
#define configUSE_TASK_PREEMPTION_DISABLE       0 // 1 each task can be set to either pre-emptive or cooperative mode

/* RP2040 specific */
#define configSUPPORT_PICO_SYNC_INTEROP         1 // 1 means that SDK pico_sync sem/mutex/queue etc. will work correctly when called from FreeRTOS tasks
#define configSUPPORT_PICO_TIME_INTEROP         1 // 1 means that SDK pico_time sleep_ms/sleep_us/sleep_until will work correctly when called from FreeRTOS tasks, and will actually block at the FreeRTOS level

#include <assert.h>
/* Define to trap errors during development. */
#define configASSERT(x)                         assert(x)

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_xQueueGetMutexHolder            1

/* A header file that defines trace macro can be included here. */

#endif /* FREERTOS_CONFIG_H */
