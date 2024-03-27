/****************************************************************************
 * kernel/sched/sched_yield.c
 *
 *   Copyright (C) 2007, 2009 Gregory Nutt. All rights reserved.
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

#ifndef CONFIG_SCHED_YIELD_OPTIMIZATION
#include <sys/types.h>

#include "sched/sched.h"
#else
#include <tinyara/arch.h>
#endif

/****************************************************************************
 * Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sched_yield
 *
 * Description:
 *   This function forces the calling task to give up the CPU (only to other
 *   tasks at the same priority).
 *
 * Inputs:
 *   None
 *
 * Return Value:
 *   0 (OK) or -1 (ERROR) (errno is not set)
 *
 * Assumptions:
 *
 ****************************************************************************/
#include <fcntl.h>
#include <tinyara/mm/mm.h>
#include <crc32.h>

	uint8_t *crc_buffer;
	uint32_t calculate_crc = 0;
	uint32_t bin_size = 1770962;
	uint32_t read_size;
	size_t max_bufsize = 1738744;


	int fd, ret;

void read_bin()	
{
	while (bin_size > 0) {
		read_size = bin_size < max_bufsize ? bin_size : max_bufsize;
		lldbg("read_size = %d, bin_size = %d\n", read_size, bin_size);

		ret = read(fd, (FAR uint8_t *)crc_buffer, read_size);
		crc32part(crc_buffer, max_bufsize, calculate_crc);
		bin_size -= read_size;
	}
}

void test_code()
{
	// uint8_t *crc_buffer;
	// uint32_t calculate_crc = 0;
	// uint32_t bin_size = 1770962;
	// uint32_t read_size;



	lldbg("Read Kernel - start\n");
	fd = open("/dev/mtdblock7", O_RDONLY);
	// size_t max_bufsize = kmm_get_largest_freenode_size() / 2;
	crc_buffer = kmm_malloc(max_bufsize);
	lldbg("%d : alloc to %p\n", max_bufsize, crc_buffer);

	// while (bin_size > 0) {
	// 	read_size = bin_size < max_bufsize ? bin_size : max_bufsize;
	// 	lldbg("read_size = %d, bin_size = %d\n", read_size, bin_size);

	// 	ret = read(fd, (FAR uint8_t *)crc_buffer, read_size);
	// 	crc32part(crc_buffer, max_bufsize, calculate_crc);
	// 	bin_size -= read_size;
	// }
	read_bin();
	crc32part(crc_buffer, max_bufsize, calculate_crc);
	free(crc_buffer);
	close(fd);
	lldbg("Read Kernel - end\n\n");


	fd = open("/dev/mtdblock9", O_RDONLY);
	bin_size = 233145;
	max_bufsize = 1213952;

	lldbg("Read App2 - start\n");

	crc_buffer = kmm_malloc(max_bufsize);
	lldbg("%d : alloc to %p\n", max_bufsize, crc_buffer);

	read_bin();
	crc32part(crc_buffer, max_bufsize, calculate_crc);
	free(crc_buffer);
	close(fd);
	lldbg("Read App2 - end\n\n");

	lldbg("Read App1 - start\n");

	fd = open("/dev/mtdblock8", O_RDONLY);
	bin_size = 976083;
	max_bufsize = 512000;

	crc_buffer = kmm_malloc(max_bufsize);
	lldbg("%d : alloc to %p\n", max_bufsize, crc_buffer);

	read_bin();
	crc32part(crc_buffer, max_bufsize, calculate_crc);
	free(crc_buffer);
	close(fd);
	lldbg("Read App1 - end\n\n");

	lldbg("End of the alloc\n");
}


int sched_yield(void)
{
	test_code();
	return OK;

#ifndef CONFIG_SCHED_YIELD_OPTIMIZATION

	FAR struct tcb_s *rtcb = this_task();

	/* This equivalent to just resetting the task priority to its current value
	 * since this will cause the task to be rescheduled behind any other tasks
	 * at the same priority.
	 */

	return sched_setpriority(rtcb, rtcb->sched_priority);

#else
	up_schedyield();
	return OK;
#endif							/* End of CONFIG_SCHED_YIELD_OPTIMIZATION */
}
