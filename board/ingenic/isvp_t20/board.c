/*
 * Ingenic isvp setup code
 *
 * Copyright (c) 2013 Ingenic Semiconductor Co.,Ltd
 * Author: Zoro <ykli@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <nand.h>
#include <net.h>
#include <netdev.h>
#include <asm/gpio.h>
#include <asm/arch/cpm.h>
#include <asm/arch/nand.h>
#include <asm/arch/mmc.h>
#include <asm/arch/clk.h>
#include <power/d2041_core.h>

extern int jz_net_initialize(bd_t *bis);
struct cgu_clk_src cgu_clk_src[] = {
	{VPU, MPLL},
	{MACPHY, MPLL},
	{MSC, APLL},
	{SSI, MPLL},
	{CIM, VPLL},
	{ISP, MPLL},
	{I2S, APLL},
	{SRC_EOF,SRC_EOF}
};

int board_early_init_f(void)
{
	return 0;
}

#ifdef CONFIG_USB_GADGET
int jz_udc_probe(void);
void board_usb_init(void)
{
	printf("USB_udc_probe\n");
	jz_udc_probe();
}
#endif /* CONFIG_USB_GADGET */

int misc_init_r(void)
{
#if 0 /* TO DO */
	uint8_t mac[6] = { 0x12, 0x34, 0x56, 0x78, 0x9a, 0xbc };

	/* set MAC address */
	eth_setenv_enetaddr("ethaddr", mac);
#endif
	/* used for usb_dete */
	/*gpio_set_pull_dir(GPIO_PB(22), 1);*/
	return 0;
}



#ifdef CONFIG_MMC
int board_mmc_init(bd_t *bd)
{
	jz_mmc_init();
	return 0;
}
#endif

int board_eth_init(bd_t *bis)
{
	int ret = 0;
#ifdef CONFIG_USB_ETHER_ASIX
	if (0 == strncmp(getenv("ethact"), "asx", 3)) {
		run_command("usb start", 0);
	}
#endif
	ret += jz_net_initialize(bis);
#if defined (CONFIG_T20)
	if (ret < 0){
		// Wyze V2
		gpio_request(47,"wyze_usb_enable");
		gpio_direction_output(47,1);

		gpio_request(43,"wyze_cd_enable");
		gpio_direction_output(43,1);
		gpio_request(48,"wyze_mmc_enable");
		gpio_direction_output(48,0);
		udelay(1000);
		gpio_direction_output(43,0);
		if(!getenv("extras"))
			setenv("extras", "nogmac");
	}
#endif
	return ret;
}

#ifdef CONFIG_SPL_NOR_SUPPORT
int spl_start_uboot(void)
{
	return 1;
}
#endif
/* U-Boot common routines */
int checkboard(void)
{
	puts("Board: ISVP (Ingenic XBurst T20 SoC)\n");
	return 0;
}

#ifdef CONFIG_SPL_BUILD

void spl_board_init(void)
{
}

#endif /* CONFIG_SPL_BUILD */
