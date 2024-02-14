/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>

#include <ble_manager/ble_manager.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/
#define BLE_MULTI_ADV_NUM	2
#define BLE_MULTI_ADV_INTERVAL_RESOLUTION_MS 10
#define BLE_ADV_RAW_DATA_MAX_LEN 31
#define BLE_ADV_RESP_DATA_MAX_LEN 31

static int counter = 0;
static int multi_adv_task_running = false;

static pthread_t ble_multi_adv_task_id;

static uint8_t g_adv_raw[] = { 
	0x02, 0x01, 0x05, 0x03, 0x19, 0x80, 0x01, 0x05, 0x03, 0x12, 0x18, 0x0f, 0x17
};

static uint8_t g_adv_resp[] = {
	0x11, 0x09, '1', 'I', 'Z', 'E', 'N', 'R', 'T', ' ', 'T', 'E', 'S', 'T', '(', '0', '2', ')',
};

typedef struct {
    uint8_t adv_data[BLE_ADV_RAW_DATA_MAX_LEN];
    uint8_t scan_resp[BLE_ADV_RESP_DATA_MAX_LEN];
    uint8_t adv_type;
    uint8_t adv_interval;
    bool is_activate;
    uint64_t prev_adv_time;
}multi_ble_adv_t;


multi_ble_adv_t multi_ble_adv[] = {
	{
		.adv_data = {0x02, 0x01, 0x05, 0x03, 0x19, 0x80, 0x01, 0x05, 0x03, 0x12, 0x18, 0x0f, 0x17},
		.scan_resp = {0x11, 0x09, '1', 'I', 'Z', 'E', 'N', 'R', 'T', ' ', 'T', 'E', 'S', 'T', '(', '0', '2', ')',},
		.adv_type = 0,
		.adv_interval = 20,
		.is_activate = true,
		.prev_adv_time = 0,
	},
	{
		.adv_data = {0x02, 0x01, 0x05, 0x03, 0x19, 0x80, 0x01, 0x05, 0x03, 0x12, 0x18, 0x0f, 0x18},
		.scan_resp = {0x11, 0x09, '2', 'I', 'Z', 'E', 'N', 'R', 'T', ' ', 'T', 'T', 'T', 'T', '(', '0', '3', ')',},
		.adv_type = 3,
		.adv_interval = 150,
		.is_activate = true,
		.prev_adv_time = 0,
	},	
};

static uint64_t get_clock_time_ms(void)

{
    uint64_t running_time_ms = 0;
    struct timespec current_time = {0, };

    clock_gettime(CLOCK_REALTIME, &current_time);

    running_time_ms = ((uint64_t)(current_time.tv_sec) * 1000) + (current_time.tv_nsec / 1000000);

    return running_time_ms;

}


static void ble_multi_adv_task()
{
    // sleep(20);
    sleep(5);

    multi_adv_task_running = true;

    uint64_t task_tick = 0;
	ble_result_e ret;

    printf("[BLEMULTIADV] start!! \n"); 

    while(multi_adv_task_running) {
        uint64_t current_time = get_clock_time_ms();            

        for(int i = 0; i < BLE_MULTI_ADV_NUM; i++) {
            if(multi_ble_adv[i].is_activate == true && 
                multi_ble_adv[i].prev_adv_time + multi_ble_adv[i].adv_interval <= current_time) {              
				ble_data adv_data[1] = { 0, };
				ble_data scan_rsp_data[1] = { 0, };

                adv_data->data = multi_ble_adv[i].adv_data;
                adv_data->length = sizeof(multi_ble_adv[i].adv_data);

				scan_rsp_data->data = multi_ble_adv[i].scan_resp;
				scan_rsp_data->length = sizeof(multi_ble_adv[i].scan_resp);

                ret = ble_server_one_shot_adv(adv_data, scan_rsp_data,multi_ble_adv[i].adv_type);

				if(ret != BLE_MANAGER_SUCCESS) {
					printf("[BLEMULTIADV] one shot adv failed = %d, index = %d\n", ret, i);
					printf("adv data = %d %d %d %d \n", multi_ble_adv[i].adv_data[0], multi_ble_adv[i].adv_data[1], multi_ble_adv[i].adv_data[2], multi_ble_adv[i].adv_data[3]);
					printf("scan_rsp_data = %d %d %d %d \n", multi_ble_adv[i].scan_resp[0], multi_ble_adv[i].scan_resp[1], multi_ble_adv[i].scan_resp[2], multi_ble_adv[i].scan_resp[3]);

				}
				else {
                	printf("[BLEMULTIADV] one time %d\n", i); 
				}

                uint64_t adv_time = get_clock_time_ms();
                multi_ble_adv[i].prev_adv_time = adv_time;
            }
        }

        usleep(BLE_MULTI_ADV_INTERVAL_RESOLUTION_MS * 1000);

        task_tick++;
        if(task_tick % ((10 * 1000) / BLE_MULTI_ADV_INTERVAL_RESOLUTION_MS) == 0) { // 10 : every 10 sec

            printf("[BLEMULTIADV] multi adv is running\n");

        }
    }
    return;
}

static int hello_task_create(pthread_t *task, char *task_name, void *task_function, void *args, int priority, unsigned int stack_size)
{
	pthread_t tid;
	pthread_attr_t attr;
	struct sched_param sparam;

	if (!task || !task_name || !task_function ||
		priority < 10 ||
		priority > 200 || stack_size <= 0)
		return -1;

	pthread_attr_init(&attr);
	pthread_attr_setschedpolicy(&attr, SCHED_RR);
	pthread_attr_setstacksize(&attr, stack_size);
	sparam.sched_priority = priority;
	pthread_attr_setschedparam(&attr, &sparam);

	if (pthread_create(&tid, &attr, (pthread_startroutine_t)task_function,
				(pthread_addr_t)args) == 0) {
		pthread_setname_np(tid, task_name);
		pthread_detach(tid);
		*task = tid;
		return 0;
	}

	return -1;	
}


static void debug_multi_adv()
{
	if(hello_task_create(&ble_multi_adv_task_id, "ble_multi_adv", ble_multi_adv_task, NULL, 100, 1024*5) != 0) {
	 	printf("%d : %s", __LINE__, "Failed to run ble_multi_adv_task");
		return;
	}
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");

	counter++;
	if(counter >= 2) {
		debug_multi_adv();
	}

	return 0;
}
