/****************************************************************************
 * scripts/script.ld
 *
 *   Copyright (C) 2016, 2019 Gregory Nutt. All rights reserved.
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

/* The Crystal NUC-H7xx (Nucleo-H753ZI) uses an STM32H753ZI with 2 MB flash.
 *
 *   User Flash memory:
 *     Bank 1: 0x0800:0000 – 0x080F:FFFF (8 sectors, 128 KB each)
 *     Bank 2: 0x0810:0000 – 0x081F:FFFF (8 sectors, 128 KB each)
 *
 *   Bootloader occupies the first 128 KB sector (Bank 1, sector 0).
 *   Firmware starts at 0x0802:0000 and has 1920 KB available.
 *
 *   SRAM layout:
 *     DTCM-RAM : 128 KB at 0x2000:0000 (2 × 64 KB, zero-wait)
 *     AXI SRAM :  512 KB at 0x2400:0000 (D1 domain)
 *     SRAM1    :  128 KB at 0x3000:0000 (D2 domain)
 *     SRAM2    :  128 KB at 0x3002:0000 (D2 domain)
 *     SRAM3    :   32 KB at 0x3004:0000 (D2 domain)
 *     SRAM4    :   64 KB at 0x3800:0000 (D3 domain, BDMA)
 *     Backup   :    4 KB at 0x3880:0000
 */

MEMORY
{
	ITCM_RAM  (rwx) : ORIGIN = 0x00000000, LENGTH =   64K
	FLASH      (rx) : ORIGIN = 0x08020000, LENGTH = 1920K

	DTCM1_RAM (rwx) : ORIGIN = 0x20000000, LENGTH =   64K
	DTCM2_RAM (rwx) : ORIGIN = 0x20010000, LENGTH =   64K
	AXI_SRAM  (rwx) : ORIGIN = 0x24000000, LENGTH =  512K
	SRAM1     (rwx) : ORIGIN = 0x30000000, LENGTH =  128K
	SRAM2     (rwx) : ORIGIN = 0x30020000, LENGTH =  128K
	SRAM3     (rwx) : ORIGIN = 0x30040000, LENGTH =   32K
	SRAM4     (rwx) : ORIGIN = 0x38000000, LENGTH =   64K
	BKPRAM    (rwx) : ORIGIN = 0x38800000, LENGTH =    4K
}

OUTPUT_ARCH(arm)
EXTERN(_vectors)
ENTRY(_stext)

EXTERN(abort)
EXTERN(_bootdelay_signature)
EXTERN(board_get_manifest)

SECTIONS
{
	.text : {
		_stext = ABSOLUTE(.);
		*(.vectors)
		. = ALIGN(32);
		_bootdelay_signature = ABSOLUTE(.);
		FILL(0xffecc2925d7d05c5)
		. += 8;
		*(.text .text.*)
		*(.fixup)
		*(.gnu.warning)
		*(.rodata .rodata.*)
		*(.gnu.linkonce.t.*)
		*(.glue_7)
		*(.glue_7t)
		*(.got)
		*(.gcc_except_table)
		*(.gnu.linkonce.r.*)
		_etext = ABSOLUTE(.);

	} > FLASH

	.init_section : {
		_sinit = ABSOLUTE(.);
		KEEP(*(.init_array .init_array.*))
		_einit = ABSOLUTE(.);
	} > FLASH

	.ARM.extab : {
		*(.ARM.extab*)
	} > FLASH

	__exidx_start = ABSOLUTE(.);
	.ARM.exidx : {
		*(.ARM.exidx*)
	} > FLASH
	__exidx_end = ABSOLUTE(.);

	_eronly = ABSOLUTE(.);

	.data : {
		_sdata = ABSOLUTE(.);
		*(.data .data.*)
		*(.gnu.linkonce.d.*)
		CONSTRUCTORS
		_edata = ABSOLUTE(.);

		/* Pad to STM32H7 flash write size (256 bits = 32 bytes) */
		. = ALIGN(16);
		FILL(0xffff)
		. += 16;
	} > AXI_SRAM AT > FLASH  = 0xffff

	.bss : {
		_sbss = ABSOLUTE(.);
		*(.bss .bss.*)
		*(.gnu.linkonce.b.*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = ABSOLUTE(.);
	} > AXI_SRAM

	/* D3 domain section for BDMA data */

	.sram4_reserve (NOLOAD) :
	{
		*(.sram4)
		. = ALIGN(4);
		_sram4_heap_start = ABSOLUTE(.);
	} > SRAM4

	.stab 0 : { *(.stab) }
	.stabstr 0 : { *(.stabstr) }
	.stab.excl 0 : { *(.stab.excl) }
	.stab.exclstr 0 : { *(.stab.exclstr) }
	.stab.index 0 : { *(.stab.index) }
	.stab.indexstr 0 : { *(.stab.indexstr) }
	.comment 0 : { *(.comment) }
	.debug_abbrev 0 : { *(.debug_abbrev) }
	.debug_info 0 : { *(.debug_info) }
	.debug_line 0 : { *(.debug_line) }
	.debug_pubnames 0 : { *(.debug_pubnames) }
	.debug_aranges 0 : { *(.debug_aranges) }
}
