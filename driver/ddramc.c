/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support
 * ----------------------------------------------------------------------------
 * Copyright (c) 2007, Stelian Pop <stelian.pop@leadtechdesign.com>
 * Copyright (c) 2007 Lead Tech Design <www.leadtechdesign.com>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaiimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "common.h"
#include "hardware.h"
#include "arch/at91_pmc.h"
#include "arch/at91_ddrsdrc.h"
#include "arch/at91_ccfg.h"
#include "debug.h"
#include "ddramc.h"
#include "timer.h"

/* write DDRC register */
static void write_ddramc(unsigned int address,
			unsigned int offset,
			const unsigned int value)
{
	writel(value, (address + offset));
}

/* read DDRC registers */
static unsigned int read_ddramc(unsigned int address, unsigned int offset)
{
	return readl(address + offset);
}

static int ddramc_decodtype_is_seq(unsigned int ddramc_cr)
{
#if defined(AT91SAM9X5) || defined(AT91SAM9N12) || defined(AT91SAMA5D3X)
	if (ddramc_cr & AT91C_DDRC2_DECOD_INTERLEAVED)
		return 0;
#endif
	return 1;
}

int ddram_initialize(unsigned int base_address,
			unsigned int ram_address,
			struct ddramc_register *ddramc_config)
{
	unsigned int ba_offset;
	unsigned int cr = 0;

	/* compute BA[] offset according to CR configuration */
	ba_offset = (ddramc_config->cr & AT91C_DDRC2_NC) + 9;
	if (ddramc_decodtype_is_seq(ddramc_config->cr))
		ba_offset += ((ddramc_config->cr & AT91C_DDRC2_NR) >> 2) + 11;

	ba_offset += (ddramc_config->mdr & AT91C_DDRC2_DBW) ? 1 : 2;

	dbg_log(3, " ba_offset = %x ...\n\r", ba_offset);

	/*
	 * Step 1: Program the memory device type into the Memory Device Register
	 */
	write_ddramc(base_address, HDDRSDRC2_MDR, ddramc_config->mdr);

	/* 
	 * Step 2: Program the feature of DDR2-SDRAM device into 
	 * the Timing Register, and into the Configuration Register
	 */
	write_ddramc(base_address, HDDRSDRC2_CR, ddramc_config->cr);

	write_ddramc(base_address, HDDRSDRC2_T0PR, ddramc_config->t0pr);
	write_ddramc(base_address, HDDRSDRC2_T1PR, ddramc_config->t1pr);
	write_ddramc(base_address, HDDRSDRC2_T2PR, ddramc_config->t2pr);

	/*
	 * Step 3: An NOP command is issued to the DDR2-SDRAM
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD);
	*((unsigned volatile int *)ram_address) = 0;
	/* Now, clocks which drive the DDR2-SDRAM device are enabled */

	/* A minimum pause wait 200 us is provided to precede any signal toggle.
	(6 core cycles per iteration, core is at 396MHz: min 13340 loops) */
	udelay(200);

	/*
	 * Step 4:  An NOP command is issued to the DDR2-SDRAM
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD);
	*((unsigned volatile int *)ram_address) = 0;
	/* Now, CKE is driven high */
	/* wait 400 ns min */
	udelay(1);

	/*
	 * Step 5: An all banks precharge command is issued to the DDR2-SDRAM.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_PRCGALL_CMD);
	*((unsigned volatile int *)ram_address) = 0;

	/* wait 2 cycles min (of tCK) = 15 ns min */
	udelay(1);

	/*
	 * Step 6: An Extended Mode Register set(EMRS2) cycle is issued to chose between commercial or high
	 * temperature operations.
	 * Perform a write access to DDR2-SDRAM to acknowledge this command.
	 * The write address must be chosen so that BA[1] is set to 1 and BA[0] is set to 0.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
	*((unsigned int *)(ram_address + (0x2 << ba_offset))) = 0;

	/* wait 2 cycles min (of tCK) = 15 ns min */
	udelay(1);

	/*
	 * Step 7: An Extended Mode Register set(EMRS3) cycle is issued
	 * to set the Extended Mode Register to "0".
	 * Perform a write access to DDR2-SDRAM to acknowledge this command.
	 * The write address must be chosen so that BA[1] is set to 1 and BA[0] is set to 1.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
	*((unsigned int *)(ram_address + (0x3 << ba_offset))) = 0;

	/* wait 2 cycles min (of tCK) = 15 ns min */
	udelay(1);

	/*
	 * Step 8: An Extened Mode Register set(EMRS1) cycle is issued to enable DLL,
	 * and to program D.I.C(Output Driver Impedance Control)
	 * Perform a write access to DDR2-SDRAM to acknowledge this command.
	 * The write address must be chosen so that BA[1] is set to 0 and BA[0] is set to 1.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
	*((unsigned int *)(ram_address + (0x1 << ba_offset))) = 0;

	/* An additional 200 cycles of clock are required for locking DLL */
	udelay(1);

	/*
	 * Step 9: Program DLL field into the Configuration Register to high(Enable DLL reset)
	 */
	cr = read_ddramc(base_address, HDDRSDRC2_CR);
	write_ddramc(base_address, HDDRSDRC2_CR, cr | AT91C_DDRC2_DLL_RESET_ENABLED);

	/*
	 * Step 10: A Mode Register set(MRS) cycle is issied to reset DLL.
	 * Perform a write access to DDR2-SDRAM to acknowledge this command.
	 * The write address must be chosen so that BA[1:0] bits are set to 0.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_LMR_CMD);
	*((unsigned int *)(ram_address + (0x0 << ba_offset))) = 0;

	/* wait 2 cycles min (of tCK) = 15 ns min */
	udelay(1);

	/*
	 * Step 11: An all banks precharge command is issued to the DDR2-SDRAM.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_PRCGALL_CMD);
	*(((unsigned volatile int *)ram_address)) = 0;

	/* wait 400 ns min (not needed on certain DDR2 devices) */
	udelay(1);

	/*
	 * Step 12: Two auto-refresh (CBR) cycles are provided.
	 * Program the auto refresh command (CBR) into the Mode Register.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_RFSH_CMD);
	*(((unsigned volatile int *)ram_address)) = 0;

	/* wait TRFC cycles min (135 ns min) extended to 400 ns */
	udelay(1);

	/* Set 2nd CBR */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_RFSH_CMD);
	*(((unsigned volatile int *)ram_address)) = 0;

	/* wait TRFC cycles min (135 ns min) extended to 400 ns */
	udelay(1);

	/*
	 * Step 13: Program DLL field into the Configuration Register to low(Disable DLL reset).
	 */
	cr = read_ddramc(base_address, HDDRSDRC2_CR);
	write_ddramc(base_address, HDDRSDRC2_CR, cr & (~AT91C_DDRC2_DLL_RESET_ENABLED));

	/*
	 * Step 14: A Mode Register set (MRS) cycle is issued to program
	 * the parameters of the DDR2-SDRAM devices, in particular CAS latency,
	 * burst length and to disable DDL reset.
	 * Perform a write access to DDR2-SDRAM to acknowledge this command.
	 * The write address must be chosen so that BA[1:0] bits are set to 0.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_LMR_CMD);
	*((unsigned int *)(ram_address + (0x0 << ba_offset))) = 0;

	/* wait 2 cycles min (of tCK) = 15 ns min */
	udelay(1);

	/*
	 * Step 15: Program OCD field into the Configuration Register
	 * to high (OCD calibration default).
	 */
	cr = read_ddramc(base_address, HDDRSDRC2_CR);
	write_ddramc(base_address, HDDRSDRC2_CR, cr | AT91C_DDRC2_OCD_DEFAULT);

	/* wait 2 cycles min (of tCK) = 15 ns min */
	udelay(1);

	/*
	 * Step 16: An Extended Mode Register set (EMRS1) cycle is issued to OCD default value.
	 * Perform a write access to DDR2-SDRAM to acknowledge this command.
	 * The write address must be chosen so that BA[1] is set to 0 and BA[0] is set to 1.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
	*((unsigned int *)(ram_address + (0x1 << ba_offset))) = 0;

	/* wait 2 cycles min (of tCK) = 15 ns min */
	udelay(1);

	/*
	 * Step 17: Program OCD field into the Configuration Register
	 * to low (OCD calibration mode exit).
	 */
	cr = read_ddramc(base_address, HDDRSDRC2_CR);
	write_ddramc(base_address, HDDRSDRC2_CR, cr & (~AT91C_DDRC2_OCD_DEFAULT));

	/* wait 2 cycles min (of tCK) = 15 ns min */
	udelay(1);

	/*
	 * Step 18: An Extended Mode Register set (EMRS1) cycle is issued to enable OCD exit.
	 * Perform a write access to DDR2-SDRAM to acknowledge this command.
	 * The write address must be chosen so that BA[1] is set to 0 and BA[0] is set to 1.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
	*((unsigned int *)(ram_address + (0x1 << ba_offset))) = 0;

	/* wait 2 cycles min (of tCK) = 15 ns min */
	udelay(1);

	/*
	 * Step 19: A Nornal mode command is provided.
	 */
	write_ddramc(base_address, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NORMAL_CMD);
	*(((unsigned volatile int *)ram_address)) = 0;

	/*
	 * Step 20: Perform a write access to any DDR2-SDRAM address
	 */
	*(((unsigned volatile int *)ram_address)) = 0;

	/*
	 * Step 21: Write the refresh rate into the count field in the Refresh Timer register.
	 */
	write_ddramc(base_address, HDDRSDRC2_RTR, ddramc_config->rtr);

	/*
	 * Now we are ready to work on the DDRSDR
	 *  wait for end of calibration
	 */
	udelay(10);

	return 0;
}

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR DDR2/SDRAM Controller
// *****************************************************************************
#ifndef __ASSEMBLY__
typedef volatile unsigned int AT91_REG;// Hardware register definition
#define AT91_CAST(a) (a)
#else
#define AT91_CAST(a)
#endif

#ifndef __ASSEMBLY__
#undef HDDRSDRC2_MR
#undef HDDRSDRC2_RTR
#undef HDDRSDRC2_CR
#undef HDDRSDRC2_T0PR
#undef HDDRSDRC2_T1PR
#undef HDDRSDRC2_T2PR
#undef HDDRSDRC2_LPR
#undef HDDRSDRC2_MDR
#undef HDDRSDRC2_DLL
#undef HDDRSDRC2_HS
#undef HDDRSDRC2_WPSR

typedef struct _AT91S_HDDRSDRC2 {
	AT91_REG	 HDDRSDRC2_MR; 	// Mode Register
	AT91_REG	 HDDRSDRC2_RTR; 	// Refresh Timer Register
	AT91_REG	 HDDRSDRC2_CR; 	// Configuration Register
	AT91_REG	 HDDRSDRC2_T0PR; 	// Timing0 Register
	AT91_REG	 HDDRSDRC2_T1PR; 	// Timing1 Register
	AT91_REG	 HDDRSDRC2_T2PR; 	// Timing2 Register
	AT91_REG	 Reserved0[1]; 	// 
	AT91_REG	 HDDRSDRC2_LPR; 	// Low-power Register
	AT91_REG	 HDDRSDRC2_MDR; 	// Memory Device Register
	AT91_REG	 HDDRSDRC2_DLL; 	// DLL Information Register
	AT91_REG	 HDDRSDRC2_VER; 	// DLL Version Register
	AT91_REG	 HDDRSDRC2_HS; 	// High Speed Register
	AT91_REG	 HDDRSDRC2_DELAY1; 	// Pad delay1 Register
	AT91_REG	 HDDRSDRC2_DELAY2; 	// Pad delay2 Register
	AT91_REG	 HDDRSDRC2_DELAY3; 	// Pad delay3 Register
	AT91_REG	 HDDRSDRC2_DELAY4; 	// Pad delay4 Register
	AT91_REG	 HDDRSDRC2_DELAY5; 	// Pad delay5 Register
	AT91_REG	 HDDRSDRC2_DELAY6; 	// Pad delay6 Register
	AT91_REG	 HDDRSDRC2_DELAY7; 	// Pad delay7 Register
	AT91_REG	 HDDRSDRC2_DELAY8; 	// Pad delay8 Register
	AT91_REG	 Reserved1[37]; 	// 
	AT91_REG	 HDDRSDRC2_WPMR; 	// Write Protect Mode Register
	AT91_REG	 HDDRSDRC2_WPSR; 	// Write Protect Status Register
	AT91_REG	 Reserved2[4]; 	// 
	AT91_REG	 HDDRSDRC2_VERSION; 	// Version Register
} AT91S_HDDRSDRC2, *AT91PS_HDDRSDRC2;
#else
#define HDDRSDRC2_MR    (AT91_CAST(AT91_REG *) 	0x00000000) // (HDDRSDRC2_MR) Mode Register
#define HDDRSDRC2_RTR   (AT91_CAST(AT91_REG *) 	0x00000004) // (HDDRSDRC2_RTR) Refresh Timer Register
#define HDDRSDRC2_CR    (AT91_CAST(AT91_REG *) 	0x00000008) // (HDDRSDRC2_CR) Configuration Register
#define HDDRSDRC2_T0PR  (AT91_CAST(AT91_REG *) 	0x0000000C) // (HDDRSDRC2_T0PR) Timing0 Register
#define HDDRSDRC2_T1PR  (AT91_CAST(AT91_REG *) 	0x00000010) // (HDDRSDRC2_T1PR) Timing1 Register
#define HDDRSDRC2_T2PR  (AT91_CAST(AT91_REG *) 	0x00000014) // (HDDRSDRC2_T2PR) Timing2 Register
#define HDDRSDRC2_LPR   (AT91_CAST(AT91_REG *) 	0x0000001C) // (HDDRSDRC2_LPR) Low-power Register
#define HDDRSDRC2_MDR   (AT91_CAST(AT91_REG *) 	0x00000020) // (HDDRSDRC2_MDR) Memory Device Register
#define HDDRSDRC2_DLL   (AT91_CAST(AT91_REG *) 	0x00000024) // (HDDRSDRC2_DLL) DLL Information Register
#define HDDRSDRC2_DLL_VER (AT91_CAST(AT91_REG *) 	0x00000028) // (HDDRSDRC2_DLL_VER) DLL Version Register
#define HDDRSDRC2_HS    (AT91_CAST(AT91_REG *) 	0x0000002C) // (HDDRSDRC2_HS) High Speed Register
#define HDDRSDRC2_DELAY1 (AT91_CAST(AT91_REG *) 	0x00000030) // (HDDRSDRC2_DELAY1) Pad delay1 Register
#define HDDRSDRC2_DELAY2 (AT91_CAST(AT91_REG *) 	0x00000034) // (HDDRSDRC2_DELAY2) Pad delay2 Register
#define HDDRSDRC2_DELAY3 (AT91_CAST(AT91_REG *) 	0x00000038) // (HDDRSDRC2_DELAY3) Pad delay3 Register
#define HDDRSDRC2_DELAY4 (AT91_CAST(AT91_REG *) 	0x0000003C) // (HDDRSDRC2_DELAY4) Pad delay4 Register
#define HDDRSDRC2_DELAY5 (AT91_CAST(AT91_REG *) 	0x00000040) // (HDDRSDRC2_DELAY5) Pad delay5 Register
#define HDDRSDRC2_DELAY6 (AT91_CAST(AT91_REG *) 	0x00000044) // (HDDRSDRC2_DELAY6) Pad delay6 Register
#define HDDRSDRC2_DELAY7 (AT91_CAST(AT91_REG *) 	0x00000048) // (HDDRSDRC2_DELAY7) Pad delay7 Register
#define HDDRSDRC2_DELAY8 (AT91_CAST(AT91_REG *) 	0x0000004C) // (HDDRSDRC2_DELAY8) Pad delay8 Register
#define HDDRSDRC2_WPMR  (AT91_CAST(AT91_REG *) 	0x000000E4) // (HDDRSDRC2_WPMR) Write Protect Mode Register
#define HDDRSDRC2_WPSR  (AT91_CAST(AT91_REG *) 	0x000000E8) // (HDDRSDRC2_WPSR) Write Protect Status Register
#define HDDRSDRC2_VERSION (AT91_CAST(AT91_REG *) 	0x000000FC) // (HDDRSDRC2_VERSION) Version Register

#endif

#define AT91C_BASE_DDR2CP1   (AT91_CAST(AT91PS_HDDRSDRC2) 	0xFFFFE400) // (DDR2CP1) Base Address
// EBI_SDRAM
#define AT91C_EBI_SDRAM	 (0x20000000) // SDRAM on EBI Chip Select 1 base address
#define AT91C_EBI_SDRAM_SIZE	 (0x10000000) // SDRAM on EBI Chip Select 1 size in byte (262144 Kbytes)
// EBI_SDRAM_16BIT
#define AT91C_EBI_SDRAM_16BIT	 (0x20000000) // SDRAM on EBI Chip Select 1 base address
#define AT91C_EBI_SDRAM_16BIT_SIZE	 (0x02000000) // SDRAM on EBI Chip Select 1 size in byte (32768 Kbytes)
// EBI_SDRAM_32BIT
#define AT91C_EBI_SDRAM_32BIT	 (0x20000000) // SDRAM on EBI Chip Select 1 base address
#define AT91C_EBI_SDRAM_32BIT_SIZE	 (0x04000000) // SDRAM on EBI Chip Select 1 size in byte (65536 Kbytes)

// *****************************************************************************
//              SOFTWARE API DEFINITION  FOR AHB CCFG Interface
// *****************************************************************************
#ifndef __ASSEMBLY__
#undef CCFG_UDPHS
#undef CCFG_EBICSA
#undef CCFG_MATRIXVERSION
typedef struct _AT91S_CCFG {
	AT91_REG	 CCFG_TCMR; 	//  TCM configuration
	AT91_REG	 CCFG_UDPHS; 	//  USB HS configuration
	AT91_REG	 CCFG_VIDEO; 	//  Video Mode configuration
	AT91_REG	 Reserved0[3]; 	// 
	AT91_REG	 CCFG_EBICSA; 	//  EBI Chip Select Assignement Register
	AT91_REG	 Reserved1[52]; 	// 
	AT91_REG	 CCFG_MATRIXVERSION; 	//  Version Register
} AT91S_CCFG, *AT91PS_CCFG;
#else
#define CCFG_TCMR       (AT91_CAST(AT91_REG *) 	0x00000000) // (CCFG_TCMR)  TCM configuration
#define CCFG_UDPHS      (AT91_CAST(AT91_REG *) 	0x00000004) // (CCFG_UDPHS)  USB HS configuration
#define CCFG_VIDEO      (AT91_CAST(AT91_REG *) 	0x00000008) // (CCFG_VIDEO)  Video Mode configuration
#define CCFG_EBICSA     (AT91_CAST(AT91_REG *) 	0x00000018) // (CCFG_EBICSA)  EBI Chip Select Assignement Register
#define CCFG_MATRIXVERSION (AT91_CAST(AT91_REG *) 	0x000000EC) // (CCFG_MATRIXVERSION)  Version Register
#endif
#define AT91C_BASE_CCFG      (AT91_CAST(AT91PS_CCFG) 	0xFFFFEB10) // (CCFG) Base Address

#define WRITE(peripheral, register, value)  (peripheral->register = value)
#define READ(peripheral, register)          (peripheral->register)

//------------------------------------------------------------------------------
/// Configure DDR on EBI bank 1
//------------------------------------------------------------------------------
void BOARD_ConfigureDdramCp1(unsigned char busWidth)
{    
    AT91PS_HDDRSDRC2 pDdrc = AT91C_BASE_DDR2CP1;
    volatile unsigned int *pDdr = (unsigned int *) AT91C_EBI_SDRAM;
    int i;
    volatile unsigned int cr = 0;
    unsigned short ddrc_dbw = 0;

    // Configure EBI
    AT91C_BASE_CCFG->CCFG_EBICSA |= AT91C_EBI_CS1A_SDRAMC;
    
    switch (busWidth) {
        case 16:
        default:
            ddrc_dbw = AT91C_DDRC2_DBW_16_BITS;
            break;

        case 32:
            ddrc_dbw = AT91C_DDRC2_DBW_32_BITS;
            break;

    }

    // Enable DDR2 clock x2 in PMC
    writel(AT91C_PMC_DDR, AT91C_BASE_PMC + PMC_SCER);

    // Disable anticipated read
    WRITE(pDdrc, HDDRSDRC2_HS, (READ(pDdrc, HDDRSDRC2_HS) | AT91C_DDRC2_NO_ANT));

    // Step 1: Program the memory device type
    WRITE(pDdrc, HDDRSDRC2_MDR, ddrc_dbw   |
                                AT91C_DDRC2_MD_DDR2_SDRAM);     // DDR2
    
    // Step 2:                            
    // 1. Program the features of DDR2-SDRAM device into the Configuration Register.                    
    // 2. Program the features of DDR2-SDRAM device into the Timing Register HDDRSDRC2_T0PR.            
    // 3. Program the features of DDR2-SDRAM device into the Timing Register HDDRSDRC2_T1PR.            
    // 4. Program the features of DDR2-SDRAM device into the Timing Register HDDRSDRC2_T2PR.            

    WRITE(pDdrc, HDDRSDRC2_CR, AT91C_DDRC2_NC_DDR10_SDR9  |     // 10 column bits (1K)
                               AT91C_DDRC2_NR_14          |     // 14 row bits    (8K)
                               AT91C_DDRC2_CAS_3          |     // CAS Latency 3
                               AT91C_DDRC2_DLL_RESET_DISABLED
                               ); // DLL not reset

    // assume timings for 7.5ns min clock period
    WRITE(pDdrc, HDDRSDRC2_T0PR, AT91C_DDRC2_TRAS_6       |     //  6 * 7.5 = 45 ns
                                 AT91C_DDRC2_TRCD_2       |     //  2 * 7.5 = 15 ns
                                 AT91C_DDRC2_TWR_2        |     //  2 * 7.5 = 15 ns
                                 AT91C_DDRC2_TRC_8        |     //  8 * 7.5 = 75 ns
                                 AT91C_DDRC2_TRP_2        |     //  2 * 7.5 = 15 ns
                                 AT91C_DDRC2_TRRD_1       |     //  1 * 7.5 = 7.5 ns
                                 AT91C_DDRC2_TWTR_1       |     //  1 clock cycle
                                 AT91C_DDRC2_TMRD_2);           //  2 clock cycles

    WRITE(pDdrc, HDDRSDRC2_T1PR, AT91C_DDRC2_TXP_2  |           //  2 * 7.5 = 15 ns
                                 200 << 16          |           // 200 clock cycles, TXSRD: Exit self refresh delay to Read command
                                 16 << 8            |           // 16 * 7.5 = 120 ns TXSNR: Exit self refresh delay to non read command
                                 AT91C_DDRC2_TRFC_14 << 0);     // 14 * 7.5 = 105 ns (must be 105 ns for 512M DDR)

    WRITE(pDdrc, HDDRSDRC2_T2PR, AT91C_DDRC2_TRTP_1   |         //  1 * 7.5 = 7.5 ns
                                 AT91C_DDRC2_TRPA_0   |         
                                 AT91C_DDRC2_TXARDS_7 |         //  7 clock cycles
                                 AT91C_DDRC2_TXARD_2);          //  2 clock cycles

    // Step 3: An NOP command is issued to the DDR2-SDRAM to enable clock.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD);
    *pDdr = 0;

    // Initialization Step 3 (must wait 200 us) (6 core cycles per iteration, core is at 396MHz: min 13200 loops)
    for (i = 0; i < 13300; i++) {
        asm("    nop");
    }
    // Step 4:  An NOP command is issued to the DDR2-SDRAM 
    // NOP command -> allow to enable cke
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NOP_CMD);
    *pDdr = 0;
    
    // wait 400 ns min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }

    // Step 5: An all banks precharge command is issued to the DDR2-SDRAM.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_PRCGALL_CMD);
    *pDdr = 0;
    
    // wait 400 ns min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }

    // Step 6: An Extended Mode Register set (EMRS2) cycle is  issued to chose between commercialor high  temperature operations.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
    *((unsigned int *)((unsigned char *)pDdr + 0x4000000)) = 0;
    
    // wait 2 cycles min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }
    
    // Step 7: An Extended Mode Register set (EMRS3) cycle is issued to set all registers to 0.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
    *((unsigned int *)((unsigned char *)pDdr + 0x6000000)) = 0;

    // wait 2 cycles min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }
     
    // Step 8:  An Extended Mode Register set (EMRS1) cycle is issued to enable DLL.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
    *((unsigned int *)((unsigned char *)pDdr + 0x2000000)) = 0;

    // wait 200 cycles min
    for (i = 0; i < 10000; i++) {
        asm("    nop");
    }
    
    // Step 9:  Program DLL field into the Configuration Register.
    cr = READ(pDdrc, HDDRSDRC2_CR);
    WRITE(pDdrc, HDDRSDRC2_CR, cr | AT91C_DDRC2_DLL_RESET_ENABLED);
    
    // Step 10: A Mode Register set (MRS) cycle is issued to reset DLL.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_LMR_CMD);
    *(pDdr) = 0;

    // wait 2 cycles min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }
    
    // Step 11: An all banks precharge command is issued to the DDR2-SDRAM.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_PRCGALL_CMD);
    *(pDdr) = 0;

    // wait 400 ns min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }

    // Step 12: Two auto-refresh (CBR) cycles are provided. Program the auto refresh command (CBR) into the Mode Register.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_RFSH_CMD);
    *(pDdr) = 0;

    // wait 10 cycles min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }
    // Set 2nd CBR
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_RFSH_CMD);
    *(pDdr) = 0;

    // wait 10 cycles min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }
    
    // Step 13: Program DLL field into the Configuration Register to low(Disable DLL reset).
    cr = READ(pDdrc, HDDRSDRC2_CR);
    WRITE(pDdrc, HDDRSDRC2_CR, cr & (~AT91C_DDRC2_DLL_RESET_ENABLED));
    
    // Step 14: A Mode Register set (MRS) cycle is issued to program the parameters of the DDR2-SDRAM devices.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_LMR_CMD);
    *(pDdr) = 0;

    // Step 15: Program OCD field into the Configuration Register to high (OCD calibration default).
    cr = READ(pDdrc, HDDRSDRC2_CR);
    WRITE(pDdrc, HDDRSDRC2_CR, cr | AT91C_DDRC2_OCD_DEFAULT);
    
    // Step 16: An Extended Mode Register set (EMRS1) cycle is issued to OCD default value.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
    *((unsigned int *)((unsigned char *)pDdr + 0x2000000)) = 0;

    // wait 2 cycles min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }
    
    // Step 17: Program OCD field into the Configuration Register to low (OCD calibration mode exit).
    cr = READ(pDdrc, HDDRSDRC2_CR);
    WRITE(pDdrc, HDDRSDRC2_CR, cr & (~AT91C_DDRC2_OCD_EXIT));
    
    // Step 18: An Extended Mode Register set (EMRS1) cycle is issued to enable OCD exit.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_EXT_LMR_CMD);
    *((unsigned int *)((unsigned char *)pDdr + 0x6000000)) = 0;

    // wait 2 cycles min
    for (i = 0; i < 100; i++) {
        asm("    nop");
    }
    
    // Step 19,20: A mode Normal command is provided. Program the Normal mode into Mode Register.
    WRITE(pDdrc, HDDRSDRC2_MR, AT91C_DDRC2_MODE_NORMAL_CMD);
    *(pDdr) = 0;

    // Step 21: Write the refresh rate into the count field in the Refresh Timer register. The DDR2-SDRAM device requires a
    // refresh every 15.625 s or 7.81 s. With a 100MHz frequency, the refresh timer count register must to be set with
    // (15.625 /100 MHz) = 1562 i.e. 0x061A or (7.81 /100MHz) = 781 i.e. 0x030d.
  
    // Set Refresh timer
    WRITE(pDdrc, HDDRSDRC2_RTR, 0x00000411);

    // OK now we are ready to work on the DDRSDR

    // wait for end of calibration
    for (i = 0; i < 500; i++) {
        asm("    nop");
    }
}

