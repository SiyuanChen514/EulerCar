/**
  * @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
  * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
  * following conditions are met:
  * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
  * disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
  * following disclaimer in the documentation and/or other materials provided with the distribution.
  * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
  * products derived from this software without specific prior written permission.
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
  * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
  * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  * @file    flash.lds
  * @author  MCU Application Driver Team
  * @brief   RISCV flash link script
  */

OUTPUT_ARCH( "riscv" )
ENTRY(_start)
SRAM_START   = 0x4000000;
SRAM_END     = 0x4000000 + 16K;

RAM_CODE_START         = 0x2000000;
RAM_CODE_SIZE          = 0;

RAM_RESERVE_DATA_START = SRAM_START + RAM_CODE_SIZE;
RAM_RESERVE_DATA_SIZE  = 0;

RAM_START        = RAM_RESERVE_DATA_START + RAM_RESERVE_DATA_SIZE;
RAM_END          = SRAM_START + 16K;

STL_TEST_START   = RAM_START;
RAM_BUF_SIZE     = 0x20;
RAM_PNT_SIZE     = 0x10;
CLASSB_SIZE      = 0x28;
CLASSB_INV_SIZE  = 0x28;
STL_MEM_CHK_SIZE = RAM_BUF_SIZE + RAM_PNT_SIZE + CLASSB_SIZE + CLASSB_INV_SIZE;
STACK_TEST_SIZE  = 0x10;

RAM_DATA_START   = STL_TEST_START + STL_MEM_CHK_SIZE;
RAM_DATA_SIZE    = 10K - RAM_CODE_SIZE - RAM_RESERVE_DATA_SIZE - STL_MEM_CHK_SIZE - STACK_TEST_SIZE;

STACK_TEST_START = RAM_DATA_START + RAM_DATA_SIZE;

STATCK_START     = STACK_TEST_START + STACK_TEST_SIZE;
STACK_SIZE       = 5k;
NMI_STACK_SIZE   = 1k;
INIT_STACK_SIZE  = 1k;
FLASH_START      = 0x3000000;
FLASH_SIZE       = 160K;

MEMORY
{
    /* ram for code */
    RAM_CODE(xr) : ORIGIN = RAM_CODE_START, LENGTH = RAM_CODE_SIZE

    /* ram for reserved data */
    RAM_RESERVE_DATA(rw) : ORIGIN = RAM_RESERVE_DATA_START, LENGTH = RAM_RESERVE_DATA_SIZE

    /*ram for STL var */
    RAM_BUF(xrw)    : ORIGIN = STL_TEST_START, LENGTH = RAM_BUF_SIZE
    RAM_PNT(xrw)    : ORIGIN = STL_TEST_START + RAM_BUF_SIZE, LENGTH = RAM_PNT_SIZE
    CLASSB(xrw)     : ORIGIN = STL_TEST_START + RAM_BUF_SIZE + RAM_PNT_SIZE, LENGTH = CLASSB_SIZE
    CLASSB_INV(xrw) : ORIGIN = STL_TEST_START + RAM_BUF_SIZE + RAM_PNT_SIZE + CLASSB_SIZE, LENGTH = CLASSB_INV_SIZE

    /* ram for common bss and data */
    RAM_DATA(xrw) : ORIGIN = RAM_DATA_START, LENGTH = RAM_DATA_SIZE

    /* ram for stack test array (size: 1word) */
    STACK_TEST(xrw) : ORIGIN = RAM_DATA_START + RAM_DATA_SIZE, LENGTH = STACK_TEST_SIZE

    /* ram for stack */
    RAM_STACK(xrw) : ORIGIN = STATCK_START, LENGTH = STACK_SIZE + NMI_STACK_SIZE

    /*magic number */
    FLASH_MAGIC(rw) : ORIGIN = FLASH_START, LENGTH = 4

    /* ram for target */
    FLASH_CODE(rx) : ORIGIN = FLASH_START + 4, LENGTH = FLASH_SIZE
}


SECTIONS
{
    .data.magic : ALIGN(4)
    {
        KEEP(*(.data.magic))
    } > FLASH_MAGIC

    /* The startup code goes first into FLASH_CODE */
    .text.entry : ALIGN(4)
    {
        KEEP(*(.text.entry))
    } > FLASH_CODE

    STACK_TEST (NOLOAD):
    {
        *(STACK_BOTTOM)
    } > STACK_TEST

    /* Stack in SRAM at Highest addresses */
    .stacks (NOLOAD) :
    {
        . = ALIGN(4);
        __SYSTEM_STACK_BEGIN__ = ORIGIN(RAM_STACK);
        KEEP(*(.stacks))
        __SYSTEM_STACK_END__ = ORIGIN(RAM_STACK) + STACK_SIZE;
        . = ALIGN(0x20);
        __INTERRUPT_STACK_BEGIN__ = __SYSTEM_STACK_END__;
        . = ALIGN(0x20);
        __NMI_STACK_BEGIN__ = __SYSTEM_STACK_END__;
        . += NMI_STACK_SIZE;
        . = ALIGN(0x20);
        __NMI_STACK_END__ = __NMI_STACK_BEGIN__ + NMI_STACK_SIZE ;
    } > RAM_STACK
    __stack_top = __SYSTEM_STACK_END__;
    __init_stack_top = __SYSTEM_STACK_BEGIN__ + INIT_STACK_SIZE;
    __irq_stack_top = __SYSTEM_STACK_END__;

    .text.sram : ALIGN(4)
    {
        __sram_code_load = LOADADDR(.text.sram);
        __sram_code_start = .;
        *(.text.sram)
        . = ALIGN(4);
        __sram_code_end = .;
    } > RAM_CODE AT > FLASH_CODE

    .reserved.data : ALIGN(4)
    {
        __reserved_code_load_addr = LOADADDR(.reserved.data);
        __reserved_code_start_addr = .;
        *(.reserved.data*)
        . = ALIGN(4);
        __reserved_code_end_addr = .;
    } > RAM_RESERVE_DATA AT > FLASH_CODE

    .text : ALIGN(4)
    {
        __start_addr = .;
        *(.text*)
        *(.ram.text*)
        . = ALIGN(4);
        __rodata_start = .;
        *(.rodata*)
        . = ALIGN(4);
        __rodata_end = .;
        *(.got*)
        __text_end = .;
    } > FLASH_CODE

    /* data section */
    .data : ALIGN(4)
    {
        __data_load = LOADADDR(.data);
        __data_start = .;
        *(.data*)
        . = ALIGN(4);
        __data_end = .;
    } > RAM_DATA AT> FLASH_CODE
    __data_size = __data_end - __data_start;

    /* bss section */
    .bss (NOLOAD) : ALIGN(4)
    {
        __bss_begin__ = .;
        *(.bss*)
        *(COMMON)
        . = ALIGN(4);
        __bss_end__ = .;
    }  > RAM_DATA
    __bss_size__ = __bss_end__ - __bss_begin__;
    __global_pointer$ = __data_start + ((__data_size + __bss_size__) / 2);

    /* CHECKSUM section in FLASH end */
    CHECKSUM :
    {
        __checksum_addr = .;
        *(CHECKSUM)
        __checksum_end = .;
    } > FLASH_CODE

    /* Runtime ram buffer */
    RAM_BUF (NOLOAD):
    {
        *(RUN_TIME_RAM_BUF)
    } > RAM_BUF

    /* Ram test pointer */
    RAM_PNT (NOLOAD):
    {
        *(RUN_TIME_RAM_PNT)
    } > RAM_PNT

    /* STL classB var */
    CLASSB (NOLOAD):
    {
        *(CLASS_B_RAM)
    } > CLASSB

    /* STL classB var inverse */
    CLASSB_INV (NOLOAD):
    {
        *(CLASS_B_RAM_REV)
    } > CLASSB_INV
}
