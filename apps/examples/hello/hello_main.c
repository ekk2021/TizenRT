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
#include <stdbool.h>
#include <pthread.h>
#include <errno.h>
#include <tinyara/sched.h>
#include <tinyara/fs/mtd.h>
#include <security/security_api.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/

#define BLOCK_SIZE 4096
#define SE_DATA_SLOT_SIZE (208)

#define DEBUG_TASK_PRIORITY			100
// #define DEBUG_PARTITION_ADDRESS		0x700000
#define DEBUG_PARTITION_ADDRESS		0xE3E000
#define DEBUG_PARTITION_SIZE		(1024*1024)
#define DEBUG_SE_INFO_1				7
#define DEBUG_SE_INFO_2				8

static int _hello_task_create(pthread_t *task, char *task_name, void *task_function, void *args, int priority, unsigned int stack_size);
static int _hello_se_is_exist(int slot_name, bool *is_exist);
static int _hello_se_read(int slot_number, unsigned char *buffer, unsigned int buffer_size, unsigned int *stored_data_size);
static int _hello_flash_init(void);
static int _hello_flash_deinit(void);
static int _hello_flash_erase(unsigned int dev_addr, unsigned int erasesize);

static void _debug_hard_fault(void);
static void _debug_se_task_func(void);
static void _debug_flash_task_func(void);
static int _deblug_load_info(void);
static int _debug_flash_erase(unsigned int end_address);

FAR struct mtd_dev_s *dev_mtd = NULL;

static pthread_t _debug_se_task_id;
static pthread_t _debug_flash_task_id;


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");


	_debug_hard_fault();


	return 0;
}

/****************************************************************************/

static void _debug_hard_fault(void)
{
	if(_hello_task_create(&_debug_se_task_id, "_debug_se", _debug_se_task_func, NULL, DEBUG_TASK_PRIORITY, 1024 * 5) != 0)
	{
		printf("%d:: %s", __LINE__, "Failed run _debug_se task");
		return;
	}
	if(_hello_task_create(&_debug_flash_task_id, "_debug_flash", _debug_flash_task_func, NULL, DEBUG_TASK_PRIORITY, 1024 * 5) != 0)
	{
		printf("%d:: %s", __LINE__, "Failed run _debug_flash task");
		return;
	}
}

static void _debug_se_task_func(void)
{
	while (1)
	{
		if (_deblug_load_info() != 0)
		{
			printf("%d:: %s", __LINE__, "Failed _deblug_load_info()");
			break;
		}
		printf("%d:: %s", __LINE__, "Success _deblug_load_info()");
		usleep((1000*1) * 1000);
	}
}

static void _debug_flash_task_func(void)
{
	unsigned int end_address = DEBUG_PARTITION_ADDRESS + DEBUG_PARTITION_SIZE;

	while (1)
	{
		if (_debug_flash_erase(end_address) != 0)
		{
			printf("%d:: %s", __LINE__, "Failed _debug_flash_erase()");
			break;
		}
		printf("%d:: %s", __LINE__, "Success _debug_flash_erase()");
		usleep((1000*1) * 1000);
	}
}

static int _deblug_load_info(void)
{
	printf("%d:: %s", __LINE__, "Load info");

	unsigned char se_read_buffer[SE_DATA_SLOT_SIZE] = {0, };
	unsigned int se_read_size = 0;
	bool info_1_is_exist = false;
	bool info_2_is_exist = false;

	_hello_se_is_exist(DEBUG_SE_INFO_1, &info_1_is_exist);
	_hello_se_is_exist(DEBUG_SE_INFO_2, &info_1_is_exist);

	if (info_1_is_exist == false || info_1_is_exist == false)
	{
		printf("%d:: %s", __LINE__, "Not have cloud info data (in se)");
		return -1;
	}

	se_read_size = 0;
	memset(se_read_buffer, 0, sizeof(se_read_buffer));
	if(_hello_se_read(DEBUG_SE_INFO_1, se_read_buffer, SE_DATA_SLOT_SIZE, &se_read_size) != 0)
	{
		printf("%d:: %s", __LINE__, "se data read failure");
		return -1;
	}
	se_read_size = 0;
	memset(se_read_buffer, 0, sizeof(se_read_buffer));
	if(_hello_se_read(DEBUG_SE_INFO_2, se_read_buffer, SE_DATA_SLOT_SIZE, &se_read_size) != 0)
	{
		printf("%d:: %s", __LINE__, "se data read failure");
		return -1;
	}

	return 0;
}

static int _debug_flash_erase(unsigned int end_address)
{
	unsigned int start_erase_address = DEBUG_PARTITION_ADDRESS;
	unsigned int end_erase_address = end_address;
	unsigned int erase_sector_size = BLOCK_SIZE;
	unsigned int current_address = 0;

	// end_erase_address += DEBUG_PARTITION_ADDRESS + (2 * 512);

	if (end_erase_address <= DEBUG_PARTITION_ADDRESS + (2 * 512)|| end_erase_address > DEBUG_PARTITION_ADDRESS + DEBUG_PARTITION_SIZE)
	{
		printf("%d:: %s", __LINE__, "Invalid erase address: 0x%08lx / set max erase size", end_erase_address);
		end_erase_address = DEBUG_PARTITION_ADDRESS + DEBUG_PARTITION_SIZE;
	}

	start_erase_address = ( DEBUG_PARTITION_ADDRESS) & ~(erase_sector_size - 1);
	printf("%d:: %s", __LINE__, "start_erase_address: 0x%08lx / end_erase_address: 0x%08lx", start_erase_address, end_erase_address);

	current_address = start_erase_address;

	if (_hello_flash_init() != 0)
	{
		return -1;
	}

	while (current_address < end_erase_address)
	{
		if (_hello_flash_erase(current_address, erase_sector_size) != 0)
		{
			_hello_flash_deinit();
			return -1;
		}
		current_address += (2 * erase_sector_size);
	}

	if (_hello_flash_deinit() != 0)
	{
		return -1;
	}

	printf("%d:: %s", __LINE__, "Success to erase");
	return 0;
}

/****************************************************************************/

static int _hello_task_create(pthread_t *task, char *task_name, void *task_function, void *args, int priority, unsigned int stack_size)
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

static int _hello_se_is_exist(int slot_name, bool *is_exist)
{
	unsigned char exist_char = 0;
	unsigned int exist_int = 0;
	int ret = 0;

	ret = _hello_se_read(slot_name, &exist_char, 1, &exist_int);

	if(exist_int == 0) {
		*is_exist = false;
	} else {
		*is_exist = true;
	}

	return 0;
}

static int _hello_se_read(int slot_number, unsigned char *buffer, unsigned int buffer_size, unsigned int *stored_data_size)
{
	security_error ret = 0;

	if (slot_number >= 15 || (int)slot_number < 0 || buffer == NULL ||
				(int)buffer_size < 0 || stored_data_size == NULL)
		return -1;

	security_handle hnd;
	ret = security_init(&hnd);
	if (ret != SECURITY_OK) {
		return -1;
	}

	security_data ss_data = { NULL, buffer_size };
	char ss_path[7] = { 0, };
	snprintf(ss_path, 7, "ss/%d", slot_number);

	ret = ss_read_secure_storage(hnd, ss_path, 0, &ss_data);
	if (ret != SECURITY_OK) {
		security_deinit(hnd);
		return -1;
	}

	if(buffer_size < ss_data.length)
		memcpy(buffer, ss_data.data, buffer_size);
	else
		memcpy(buffer, ss_data.data, ss_data.length);
	free(ss_data.data);

	*stored_data_size = ss_data.length;

	(void)security_deinit(hnd);

	return 0;
}

static int _hello_flash_init(void)
{
	if(dev_mtd != NULL)
	{
		return 0;
	}

	dev_mtd = up_flashinitialize();

	if (!dev_mtd) {
		printf("%d:: %s", __LINE__, "up_flashinitialize failed\n");
		return -1;
	}

	return 0;
}

static int _hello_flash_deinit(void)
{
	return 0;
}

static int _hello_flash_erase(unsigned int dev_addr, unsigned int erasesize)
{
	unsigned int blockcnt = 0;
	int ret = 0;

	if (dev_addr == 0)
		return -1;

	blockcnt = (erasesize + BLOCK_SIZE - 1) / BLOCK_SIZE;

	if ((dev_addr % BLOCK_SIZE) || (erasesize % BLOCK_SIZE))
		return -1;

	ret = MTD_ERASE(dev_mtd , dev_addr / BLOCK_SIZE, blockcnt);
	if(ret < 0)
	{
		printf("%d:: %s", __LINE__, "MTD_ERASE error: %d\n",ret);
		return -1;
	}

	return 0;
}
