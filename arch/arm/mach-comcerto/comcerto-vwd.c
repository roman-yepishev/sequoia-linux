/*
 *  linux/arch/arm/mach-comcerto/comcerto-vwd.c
 *
 *  Copyright (C) 2011 Mindspeed Technologies, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>


//static int comcerto_wifi_rx_dummy_hdlr(struct sk_buff *skb);
//static int (*vwd_rx_hdlr)(struct sk_buff *) = comcerto_wifi_rx_dummy_hdlr;
static int (*vwd_rx_hdlr)(struct sk_buff *);

static int comcerto_wifi_rx_dummy_hdlr(struct sk_buff *skb)
{
	return -1;
}

int comcerto_wifi_rx_fastpath_register(int (*hdlr)(struct sk_buff *skb))
{
	printk(KERN_INFO "%s:%d VWD Tx function registered\n", __func__, __LINE__ );
	vwd_rx_hdlr = hdlr;

	return 0;
}

void comcerto_wifi_rx_fastpath_unregister(void)
{
	printk(KERN_INFO "%s:%d VWD Tx function unregistered\n", __func__, __LINE__ );
	vwd_rx_hdlr = comcerto_wifi_rx_dummy_hdlr;

	return;
}

int comcerto_wifi_rx_fastpath(struct sk_buff *skb)
{
	return vwd_rx_hdlr(skb);

}

EXPORT_SYMBOL(comcerto_wifi_rx_fastpath_register);
EXPORT_SYMBOL(comcerto_wifi_rx_fastpath_unregister);
EXPORT_SYMBOL(comcerto_wifi_rx_fastpath);
