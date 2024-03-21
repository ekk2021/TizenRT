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
#include <security/security_common.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/
int cnt = 0;

char input[] = { 0xd0, 0x2a, 0x26, 0xd6, 0x46, 0xb3, 0x91, 0xf3, 0x99, 0xad, 0x62, 0x0a, 0x63, 0x1f, 0x43, 0x88};
char out[16];

#define AES128_KEY "ss/2" // ex. "ss/9"
#define AES_BLOCK_SIZE 16

int test_aes_decrypt(char *input, unsigned int input_size, char *output, unsigned int output_size)
{
	if ((input_size <= 0) || (input_size % AES_BLOCK_SIZE)) {
		printf("Invalid AES key input size\n"); printf("input size : %d\n", input_size);
		return -1;
	}
	
	if (input == NULL) {
		printf("AES input is null\n");
		return -1;
	}
	
	if (output_size < input_size) {
		printf("Invalid AES key output size\n"); printf("output size : %d\n", output_size);
		return -1;
	}
	
	if (output == NULL) {
		printf("AES output is null\n");
		return -1;
	}

	security_handle hnd;
	security_data encrypt_data;
	security_data decrypt_data;
	security_aes_param aes_param;
	security_error ret;
	char *tmp = NULL;

	ret = security_init(&hnd);
	if (ret != SECURITY_OK)
	{
		printf("security_crypto : security_init failed. ret : %d\n", ret);
		return -1;
	}
	
	encrypt_data.data = input;
	encrypt_data.length = input_size;
	decrypt_data.length = output_size;
	aes_param.mode = AES_CBC_NOPAD;
	aes_param.iv = NULL;
	aes_param.iv_len = 0;

	ret = crypto_aes_decryption(hnd, &aes_param, AES128_KEY, &encrypt_data, &decrypt_data);

	if (ret != SECURITY_OK)
	{
		printf("security_crypto : crypto_aes_decryption failed. ret : %d\n", ret);
		return -1;
	}
	tmp = decrypt_data.data;

	for (int i = 0; i < output_size; i++)
	{
		output[i] = tmp[i];
	}
	return 1;

}

uint8_t *buf;
// uint8_t buf[1728256];
uint8_t head_buf[256];
int count = 0;

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	uint32_t read_size;

	printf("Hello, World!!\n");

	if(count == 0) {
		count++;
		return;
	}

	buf = (uint8_t *)kmm_malloc(1728504);
	if (buf == NULL) {
		printf("!!! Error to allocate buf\n");
	 	return;
	}	

	read_size = amebasmart_flash_read((u32)0x75f100,  (void *)buf, 1728256);
	printf("1 - read size = %d\n", read_size);

	read_size = amebasmart_flash_read((u32)0x905000,  (void *)buf, 256);
	printf("11 - read size = %d\n", read_size);


	read_size = amebasmart_flash_read((u32)0x905000,  (void *)buf, 256);
	printf("11 - read size = %d\n", read_size);

	read_size = amebasmart_flash_read((u32)0x905100,  (void *)buf, 41984);
	printf("111 - read size = %d\n", read_size);

	read_size = amebasmart_flash_read((u32)0x90f500,  (void *)buf, 256);
	printf("111 - read size = %d\n", read_size);

	kmm_free(buf);




	read_size = amebasmart_flash_read((u32)0xdde000,  (void *)head_buf, 256);
	printf("2 - read size = %d\n", read_size);


	buf = (uint8_t *)kmm_malloc(1213952);
	if (buf == NULL) {
		printf("!!! Error to allocate buf\n");
	 	return;
	}	
	read_size = amebasmart_flash_read((u32)0xdde100,  (void *)buf, 232704);
	printf("22 - read size = %d\n", read_size);

	read_size = amebasmart_flash_read((u32)0xe16e00  ,  (void *)buf, 256);
	printf("222 - read size = %d\n", read_size);

	buf = (uint8_t *)kmm_malloc(1728504);
	if (buf == NULL) {
		printf("!!! Error to allocate buf\n");
	 	return;
	}	

	read_size = amebasmart_flash_read((u32)0x75f100,  (void *)buf, 1728256);
	printf("1 - read size = %d\n", read_size);

	read_size = amebasmart_flash_read((u32)0x905000,  (void *)buf, 256);
	printf("11 - read size = %d\n", read_size);


	read_size = amebasmart_flash_read((u32)0x905000,  (void *)buf, 256);
	printf("11 - read size = %d\n", read_size);

	read_size = amebasmart_flash_read((u32)0x905100,  (void *)buf, 41984);
	printf("111 - read size = %d\n", read_size);

	read_size = amebasmart_flash_read((u32)0x90f500,  (void *)buf, 256);
	printf("111 - read size = %d\n", read_size);

	kmm_free(buf);

	// read_size = amebasmart_flash_read((u32)0x92c000   ,  (void *)buf, 256);
	// printf("333 - read size = %d\n", read_size);

	return;

#if 0
	int a = 0;
	if (cnt >= 1)
	{
		// test_aes_decrypt(input, 16, out, 128);
		test_aes_decrypt(input, 16, out, 16);	
	}
	cnt++;
	return 0;
#endif	
}