/****************************************************************************
 *
 * Copyright 2019 Samsung Electronics All Rights Reserved.
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
 * os/binfmt/libelf/libelf_sections.c
 *
 *   Copyright (C) 2012 Gregory Nutt. All rights reserved.
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

#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
#include <debug.h>

#include <tinyara/kmalloc.h>
#include <tinyara/binfmt/elf.h>

#include <string.h>
#include <tinyara/binfmt/binfmt.h>
#include "binary_manager/binary_manager_internal.h"

#include "libelf.h"

#ifdef CONFIG_APP_BINARY_SEPARATION
/* The list for a common binary and user binaries(CONFIG_NUM_APPS) */
static bin_addr_info_t g_bin_addr_list[CONFIG_NUM_APPS + 1];
#endif
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: elf_sectname
 *
 * Description:
 *   Get the symbol name in loadinfo->iobuffer[].
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

static inline int elf_sectname(FAR struct elf_loadinfo_s *loadinfo, FAR const Elf32_Shdr *shdr)
{
	FAR Elf32_Shdr *shstr;
	FAR uint8_t *buffer;
	off_t offset;
	size_t readlen;
	size_t bytesread;
	int shstrndx;
	int ret;

	/* Get the section header table index of the entry associated with the
	 * section name string table. If the file has no section name string table,
	 * this member holds the value SH_UNDEF.
	 */

	shstrndx = loadinfo->ehdr.e_shstrndx;
	if (shstrndx == SHN_UNDEF) {
		berr("No section header string table\n");
		return -EINVAL;
	}

	/* Get the section name string table section header */

	shstr = &loadinfo->shdr[shstrndx];

	/* Get the file offset to the string that is the name of the section. This
	 * is the sum of:
	 *
	 *   shstr->sh_offset: The file offset to the first byte of the section
	 *     header string table data.
	 *   shdr->sh_name: The offset to the name of the section in the section
	 *     name table
	 */

	offset = shstr->sh_offset + shdr->sh_name;

	/* Loop until we get the entire section name into memory */

	bytesread = 0;

	for (;;) {
		/* Get the number of bytes to read */

		readlen = loadinfo->buflen - bytesread;
		if (offset + readlen > loadinfo->filelen) {
			if (loadinfo->filelen <= offset) {
				berr("At end of file\n");
				return -EINVAL;
			}

			readlen = loadinfo->filelen - offset;
		}

		/* Read that number of bytes into the array */

		buffer = &loadinfo->iobuffer[bytesread];
		ret = elf_read(loadinfo, buffer, readlen, offset + bytesread);
		if (ret < 0) {
			berr("Failed to read section name\n");
			return ret;
		}

		bytesread += readlen;

		/* Did we read the NUL terminator? */

		if (memchr(buffer, '\0', readlen) != NULL) {
			/* Yes, the buffer contains a NUL terminator. */

			return OK;
		}

		/* No.. then we have to read more */

		ret = elf_reallocbuffer(loadinfo, CONFIG_ELF_BUFFERINCR);
		if (ret < 0) {
			berr("elf_reallocbuffer failed: %d\n", ret);
			return ret;
		}
	}

	/* We will not get here */

	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
#ifdef CONFIG_BINFMT_SECTION_UNIFIED_MEMORY
void *elf_find_start_section_addr(struct binary_s *binp)
{
	int text_addr = (int)binp->sections[BIN_TEXT];
	int ro_addr = (int)binp->sections[BIN_RO];
	int data_addr = (int)binp->sections[BIN_DATA];

	if (text_addr <= ro_addr) {
		if (text_addr <= data_addr) {
			return (void *)text_addr;
		} else {
			return (void *)data_addr;
		}
	} else if (ro_addr <= data_addr) {
		return (void *)ro_addr;
	} else {
		return (void *)data_addr;
	}
}
#endif

#ifdef CONFIG_APP_BINARY_SEPARATION
void *elf_find_text_section_addr(int bin_idx)
{
	if (bin_idx >= 0 && bin_idx <= CONFIG_NUM_APPS) {
		return (void *)g_bin_addr_list[bin_idx].text_addr;
	}
	return NULL;
}

void elf_save_bin_section_addr(struct binary_s *bin)
{
	if (bin != NULL) {
		uint8_t bin_idx = bin->binary_idx;

		/* Save binary section address information */

		g_bin_addr_list[bin_idx].text_addr = bin->sections[BIN_TEXT];
		g_bin_addr_list[bin_idx].text_size = bin->sizes[BIN_TEXT];
#ifdef CONFIG_SAVE_BIN_SECTION_ADDR
		binfo("[%s] text_addr : %x\n", bin->bin_name, g_bin_addr_list[bin_idx].text_addr);
#ifdef CONFIG_OPTIMIZE_APP_RELOAD_TIME
		g_bin_addr_list[bin_idx].rodata_addr = bin->sections[BIN_RO];
		g_bin_addr_list[bin_idx].rodata_size = bin->sizes[BIN_RO];
		binfo("   rodata_addr : %x\n", g_bin_addr_list[bin_idx].rodata_addr);
#endif

#if defined(CONFIG_OPTIMIZE_APP_RELOAD_TIME) || defined(CONFIG_MEM_LEAK_CHECKER)
		g_bin_addr_list[bin_idx].data_addr = bin->sections[BIN_DATA];
		g_bin_addr_list[bin_idx].bss_addr = bin->sections[BIN_BSS];
		g_bin_addr_list[bin_idx].data_size = bin->sizes[BIN_DATA];
		g_bin_addr_list[bin_idx].bss_size = bin->sizes[BIN_BSS];
		binfo("   data_addr   : %x\n", g_bin_addr_list[bin_idx].data_addr);
		binfo("   bss_addr    : %x\n", g_bin_addr_list[bin_idx].bss_addr);
#endif
#endif
	} else {
		berr("ERROR : Failed to save bin section addresses\n");
	}
}

void elf_delete_bin_section_addr(uint8_t bin_idx)
{
	/* Clear binary section address information */

	memset(&g_bin_addr_list[bin_idx], 0, sizeof(bin_addr_info_t));
}

void elf_show_all_bin_section_addr(void)
{
	int bin_idx;
	lldbg_noarg("===========================================================\n");
	lldbg_noarg("Loading location information\n");
	lldbg_noarg("===========================================================\n");	
	for (bin_idx = 0; bin_idx <= CONFIG_NUM_APPS; bin_idx++) {
		if (g_bin_addr_list[bin_idx].text_addr != 0) {
			lldbg("[%s] Text Addr : %p, Text Size : %u\n", BIN_NAME(bin_idx), g_bin_addr_list[bin_idx].text_addr, g_bin_addr_list[bin_idx].text_size);
		}
	}
}
#endif /* CONFIG_APP_BINARY_SEPARATION */

/****************************************************************************
 * Name: elf_loadshdrs
 *
 * Description:
 *   Loads section headers into memory.
 *
 * Returned Value:
 *   0 (OK) is returned on success and a negated errno is returned on
 *   failure.
 *
 ****************************************************************************/

int elf_loadshdrs(FAR struct elf_loadinfo_s *loadinfo)
{
	size_t shdrsize;
	int ret;

	DEBUGASSERT(loadinfo->shdr == NULL);

	/* Verify that there are sections */

	if (loadinfo->ehdr.e_shnum < 1) {
		berr("No sections(?)\n");
		return -EINVAL;
	}

	/* Get the total size of the section header table */

	shdrsize = (size_t) loadinfo->ehdr.e_shentsize * (size_t) loadinfo->ehdr.e_shnum;
	if (loadinfo->ehdr.e_shoff + shdrsize > loadinfo->filelen) {
		berr("Insufficent space in file for section header table\n");
		return -ESPIPE;
	}

	/* Allocate memory to hold a working copy of the sector header table */

	loadinfo->shdr = (FAR FAR Elf32_Shdr *)kmm_malloc(shdrsize);
	if (!loadinfo->shdr) {
		berr("Failed to allocate the section header table. Size: %ld\n", (long)shdrsize);
		return -ENOMEM;
	}

	/* Read the section header table into memory */

	ret = elf_read(loadinfo, (FAR uint8_t *)loadinfo->shdr, shdrsize, loadinfo->ehdr.e_shoff);
	if (ret < 0) {
		berr("Failed to read section header table: %d\n", ret);
	}

	return ret;
}

/****************************************************************************
 * Name: elf_findsection
 *
 * Description:
 *   A section by its name.
 *
 * Input Parameters:
 *   loadinfo - Load state information
 *   sectname - Name of the section to find
 *
 * Returned Value:
 *   On success, the index to the section is returned; A negated errno value
 *   is returned on failure.
 *
 ****************************************************************************/

int elf_findsection(FAR struct elf_loadinfo_s *loadinfo, FAR const char *sectname)
{
	FAR const Elf32_Shdr *shdr;
	int ret;
	int i;

	/* Search through the shdr[] array in loadinfo for a section named 'sectname' */

	for (i = 0; i < loadinfo->ehdr.e_shnum; i++) {
		/* Get the name of this section */

		shdr = &loadinfo->shdr[i];
		ret = elf_sectname(loadinfo, shdr);
		if (ret < 0) {
			berr("elf_sectname failed: %d\n", ret);
			return ret;
		}

		/* Check if the name of this section is 'sectname' */

		binfo("%d. Comparing \"%s\" and .\"%s\"\n", i, loadinfo->iobuffer, sectname);

		if (strcmp((FAR const char *)loadinfo->iobuffer, sectname) == 0) {
			/* We found it... return the index */

			return i;
		}
	}

	/* We failed to find a section with this name. */

	return -ENOENT;
}

/****************************************************************************
 * Name: get_bin_addr_list
 *
 * Description:
 *   Returns the pointer to the bin info address list
 *
 * Returned Value:
 *   Pointer to the bin info address list
 ****************************************************************************/

bin_addr_info_t *get_bin_addr_list()
{
	return g_bin_addr_list;
}
