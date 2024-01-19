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
#include <debug.h>


#include <stdlib.h>
#include <tinyara/i2c.h>
/****************************************************************************
 * hello_main
 ****************************************************************************/
#define I2C_MAX_BUFFER_SIZE		100
#define MPU9250_ADDRESS 0x68  // Device address when ADO = 1
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71

void i2c_demo(void) {
    FAR struct i2c_dev_s *i2c;
    struct i2c_msg_s msg[2];
    int ret;
    uint8_t regaddr;
    uint8_t read_data;
	char wbuf[100] = {0};
	char rbuf[10] = {0};

	//Change i2c port here
	i2c = up_i2cinitialize(1);
    I2C_SETFREQUENCY(i2c,200000);

    I2C_SETADDRESS(i2c, MPU9250_ADDRESS, 7);

    // strncpy(wbuf, "THREE", 5);
    // regaddr = WHO_AM_I_MPU9250;
    // msg[0].addr = MPU9250_ADDRESS;
    // msg[0].flags = 0;
    // msg[0].buffer = wbuf;
    // msg[0].length = 100;

    // msg[1].addr = MPU9250_ADDRESS;
    // msg[1].flags = I2C_M_READ;
    // msg[1].buffer = rbuf;
    // msg[1].length = 1;

	for (int i = 0; i < I2C_MAX_BUFFER_SIZE; i++) {
        wbuf[i] = i;
    }
    lldbg("I2C test start \n");
	ret = I2C_WRITEREAD(i2c, wbuf, 100, rbuf, 5);
	if(ret != 0) {
        lldbg("I2C write error ret = %d! \n", ret);
    }
	lldbg("I2C read result: \n");
    for (int i = 0; i < 5; i++)
	    lldbg("%x\n", rbuf[i]);
    //Printf should receive 4c 49 56 49 44
    if (strcmp(rbuf, "LIVID"))
        lldbg("receive failed\n");
    else
        lldbg("receive passed\n");
    return 0;
}
int count = 0;
#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{	
	count++;
	if (count == 2) {
        i2c_demo();
    }
}