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
#include <string.h>

#include <security/security_api.h>
int trt_crypto_random(int *random_number)
{
    security_handle hnd;
    security_error res = security_init(&hnd);



    if (res != 0) {
        printf("Fail\n  ! security_init\n");
        return -1;
    }

    security_data random;
    res = auth_generate_random(hnd, sizeof(int), &random);
    if (res != 0) {
        printf("Fail\n gen random number\n");
        security_deinit(hnd);
        return -1;
    }

    memcpy((void *)random_number, (void *)random.data, sizeof(int));

    if (*random_number < 0) {
        *random_number = *random_number * -1;
    }


    security_free_data(&random);
    security_deinit(hnd);

    return 0;
}


/****************************************************************************
 * hello_main
 ****************************************************************************/

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");

	int random_number[4];

	if((argc == 2) && (strncmp(argv[1], "1", 2) == 0)) {
		for(int i = 0 ; i < 4 ; i++) {
			trt_crypto_random(&random_number[i]);
			// printf("%d random_gen : %x\n", i, random_number[i]);
		}
        printf("generated Rand num [1st] = %x%x%x%x\n", random_number[0], random_number[1], random_number[2], random_number[3]);

		for(int i = 0 ; i < 4 ; i++) {
			trt_crypto_random(&random_number[i]);
			// printf("%d random_gen : %x\n", i, random_number[i]);
		}
        printf("generated Rand num [2nd] = %x%x%x%x\n", random_number[0], random_number[1], random_number[2], random_number[3]);

	}

	return 0;
}
