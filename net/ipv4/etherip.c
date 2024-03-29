/*
 * etherip.c: Ethernet over IPv4 tunnel driver (according to RFC3378)
 *
 * This driver could be used to tunnel Ethernet packets through IPv4
 * networks. This is especially usefull together with the bridging
 * code in Linux.
 *
 * This code was written with an eye on the IPIP driver in linux from
 * Sam Lantinga. Thanks for the great work.
 *
 *      This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      version 2 (no later version) as published by the
 *      Free Software Foundation.
 *
 */

#include <linux/version.h>
#include <linux/capability.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/ip.h>
#include <linux/if_tunnel.h>
#include <linux/list.h>
#include <linux/string.h>
#include <linux/netfilter_ipv4.h>
#include <net/ip.h>
#include <net/protocol.h>
#include <net/route.h>
#include <net/ip_tunnels.h>
#include <net/xfrm.h>
#include <net/inet_ecn.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Joerg Roedel <joro@8bytes.org>");
MODULE_DESCRIPTION("Ethernet over IPv4 tunnel driver");

#ifndef IPPROTO_ETHERIP
#define IPPROTO_ETHERIP 97
#endif


#define __IPTUNNEL_XMIT(stats1, stats2) do {				\
	int err;							\
	int pkt_len = skb->len - skb_transport_offset(skb);		\
									\
	skb->ip_summed = CHECKSUM_NONE;					\
	ip_select_ident(skb, NULL);				\
									\
	err = ip_local_out(skb);					\
	if (likely(net_xmit_eval(err) == 0)) {				\
		(stats1)->tx_bytes += pkt_len;				\
		(stats1)->tx_packets++;					\
	} else {							\
		(stats2)->tx_errors++;					\
		(stats2)->tx_aborted_errors++;				\
	}								\
} while (0)

/*
 * These 2 defines are taken from ipip.c - if it's good enough for them
 * it's good enough for me.
 */
#define HASH_SIZE        16
#define HASH(addr)       ((addr^(addr>>4))&0xF)

//#define ETHERIP_HEADER   ((u16)0x0300) /* There seems to be some difference here, This should ideally be 0x3000
//					http://lists.freebsd.org/pipermail/freebsd-bugs/2008-June/031385.html */

#define ETHERIP_HEADER     ((u16)0x3000)

#define ETHERIP_HLEN     2
#define ETHERIP_MAX_MTU  (65535 - 20 - ETHERIP_HLEN)

#define BANNER1 "etherip: Ethernet over IPv4 tunneling driver\n"
#if 0
struct pcpu_tstats {
	unsigned long rx_packets;
	unsigned long rx_bytes;
	unsigned long tx_packets;
	unsigned long tx_bytes;
};
#endif
struct etherip_tunnel {
	struct list_head list;
	struct net_device *dev;
	struct net_device_stats stats;
	struct ip_tunnel_parm parms;
	unsigned int recursion;
};

static struct net_device *etherip_tunnel_dev;
static struct list_head tunnels[HASH_SIZE];

static DEFINE_RWLOCK(etherip_lock);

static void etherip_tunnel_setup(struct net_device *dev);

static struct net_device_stats *etherip_get_stats(struct net_device *dev)
{
	struct pcpu_sw_netstats sum = { 0 };
	int i;

	for_each_possible_cpu(i) {
		const struct pcpu_sw_netstats *tstats = per_cpu_ptr(dev->tstats, i);
		
		sum.rx_packets += tstats->rx_packets;
		sum.rx_bytes   += tstats->rx_bytes;
		sum.tx_packets += tstats->tx_packets;
		sum.tx_bytes   += tstats->tx_bytes;
	}
	dev->stats.rx_packets = sum.rx_packets;
	dev->stats.rx_bytes   = sum.rx_bytes;
	dev->stats.tx_packets = sum.tx_packets;
	dev->stats.tx_bytes   = sum.tx_bytes;
	return &dev->stats;
}

/* add a tunnel to the hash */
static void etherip_tunnel_add(struct etherip_tunnel *tun)
{
	unsigned h = HASH(tun->parms.iph.daddr);
	list_add_tail(&tun->list, &tunnels[h]);
}

/* delete a tunnel from the hash*/
static void etherip_tunnel_del(struct etherip_tunnel *tun)
{
	list_del(&tun->list);
}

/* find a tunnel in the hash by parameters from userspace */
static struct etherip_tunnel* etherip_tunnel_find(struct ip_tunnel_parm *p)
{
	struct etherip_tunnel *ret;
	unsigned h = HASH(p->iph.daddr);

	list_for_each_entry(ret, &tunnels[h], list)
		if (ret->parms.iph.daddr == p->iph.daddr)
			return ret;

	return NULL;
}

/* find a tunnel by its destination address */
static struct etherip_tunnel* etherip_tunnel_locate(u32 remote)
{
	struct etherip_tunnel *ret;
	unsigned h = HASH(remote);

	list_for_each_entry(ret, &tunnels[h], list)
		if (ret->parms.iph.daddr == remote)
			return ret;

	return NULL;
}

static int etherip_change_mtu(struct net_device *dev, int new_mtu)
{
	if (new_mtu < 68 || new_mtu > ETHERIP_MAX_MTU)
		return -EINVAL;
	dev->mtu = new_mtu;

	return 0;
}

/* netdevice hard_start_xmit function
 * it gets an Ethernet packet in skb and encapsulates it in another IP
 * packet */
static int etherip_tunnel_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct etherip_tunnel *tunnel = netdev_priv(dev);
	struct rtable *rt;
	struct iphdr *iph;
	struct net_device *tdev;
	int max_headroom;
	struct pcpu_sw_netstats *tstats;
	struct flowi4 fl4;

	if (tunnel->recursion++) {
		tunnel->stats.collisions++;
		goto tx_error;
	}


	rt = ip_route_output_ports(dev_net(dev), &fl4, NULL,
				   tunnel->parms.iph.daddr,
				   tunnel->parms.iph.saddr,
				   0, 0, IPPROTO_ETHERIP,
				   RT_TOS(tunnel->parms.iph.tos),
				   tunnel->parms.link);
	if (IS_ERR(rt)) {
		dev->stats.tx_carrier_errors++;
		goto tx_error_icmp;
	}

	tdev = rt->dst.dev;
	if (tdev == dev) {
		ip_rt_put(rt);
		tunnel->stats.collisions++;
		goto tx_error;
	}

	max_headroom = (LL_RESERVED_SPACE(tdev)+sizeof(struct iphdr)
			+ ETHERIP_HLEN);

	if (skb_headroom(skb) < max_headroom || skb_shared(skb) ||
	    (skb_cloned(skb) && !skb_clone_writable(skb, 0))) {
		struct sk_buff *skn = skb_realloc_headroom(skb, max_headroom);
		if (!skn) {
			ip_rt_put(rt);
			dev->stats.tx_dropped++;
			dev_kfree_skb(skb);
			tunnel->recursion--;
			tunnel->stats.tx_dropped++;
			return 0;
		}
		if (skb->sk)
			skb_set_owner_w(skn, skb->sk);
		dev_kfree_skb(skb);
		skb = skn;
	}

	skb->transport_header = skb->mac_header;
	skb_push(skb, sizeof(struct iphdr)+ETHERIP_HLEN);
	skb_reset_network_header(skb);
	memset(&(IPCB(skb)->opt), 0, sizeof(IPCB(skb)->opt));
	IPCB(skb)->flags &= ~(IPSKB_XFRM_TUNNEL_SIZE | IPSKB_XFRM_TRANSFORMED |
			IPSKB_REROUTED);

	skb_dst_drop(skb);
	skb_dst_set(skb, &rt->dst);

	/* Build the IP header for the outgoing packet
	 *
	 * Note: This driver never sets the DF flag on outgoing packets
	 *       to ensure that the tunnel provides the full Ethernet MTU.
	 *       This behavior guarantees that protocols can be
	 *       encapsulated within the Ethernet packet which do not
	 *       know the concept of a path MTU
	 */
	iph = ip_hdr(skb);
	iph->version = 4;
	iph->ihl = sizeof(struct iphdr)>>2;
	iph->frag_off = 0;
	iph->protocol = IPPROTO_ETHERIP;
	iph->tos = tunnel->parms.iph.tos & INET_ECN_MASK;
	iph->daddr = fl4.daddr;
	iph->saddr = fl4.saddr;
	iph->ttl = tunnel->parms.iph.ttl;
	if (iph->ttl == 0)
		iph->ttl = 64;

	/* add the 16bit etherip header after the ip header */
	((u16*)(iph+1))[0]=htons(ETHERIP_HEADER);
	nf_reset(skb);
	tstats = this_cpu_ptr(dev->tstats);
	__IPTUNNEL_XMIT(tstats, &dev->stats);
	tunnel->dev->trans_start = jiffies;
	tunnel->recursion--;

	return NETDEV_TX_OK;

tx_error_icmp:
	dst_link_failure(skb);

tx_error:
	tunnel->stats.tx_errors++;
	dev_kfree_skb(skb);
	tunnel->recursion--;
	return NETDEV_TX_OK;
}

/* checks parameters the driver gets from userspace */
static int etherip_param_check(struct ip_tunnel_parm *p)
{
	if (p->iph.version != 4 ||
	    p->iph.protocol != IPPROTO_ETHERIP ||
	    p->iph.ihl != 5 ||
	    p->iph.daddr == INADDR_ANY ||
	    IN_MULTICAST(p->iph.daddr))
		return -EINVAL;

	return 0;
}

/* central ioctl function for all netdevices this driver manages
 * it allows to create, delete, modify a tunnel and fetch tunnel
 * information */
static int etherip_tunnel_ioctl(struct net_device *dev, struct ifreq *ifr,
		int cmd)
{
	int err = 0;
	struct ip_tunnel_parm p;
	struct net_device *tmp_dev;
	char *dev_name;
	struct etherip_tunnel *t;


	switch (cmd) {
	case SIOCGETTUNNEL:
		t = netdev_priv(dev);
		if (copy_to_user(ifr->ifr_ifru.ifru_data, &t->parms,
				sizeof(t->parms)))
			err = -EFAULT;
		break;
	case SIOCADDTUNNEL:
		err = -EINVAL;
		if (dev != etherip_tunnel_dev)
			goto out;

	case SIOCCHGTUNNEL:
		err = -EPERM;
		if (!capable(CAP_NET_ADMIN))
			goto out;

		err = -EFAULT;
		if (copy_from_user(&p, ifr->ifr_ifru.ifru_data,
					sizeof(p)))
			goto out;
		p.i_flags = p.o_flags = 0;

		if ((err = etherip_param_check(&p)) < 0)
			goto out;

		t = etherip_tunnel_find(&p);

		err = -EEXIST;
		if (t != NULL && t->dev != dev)
			goto out;

		if (cmd == SIOCADDTUNNEL) {

			p.name[IFNAMSIZ-1] = 0;
			dev_name = p.name;
			if (dev_name[0] == 0)
				dev_name = "ethip%d";

			err = -ENOMEM;
			tmp_dev = alloc_netdev(
					sizeof(struct etherip_tunnel),
					dev_name, NET_NAME_UNKNOWN,
					etherip_tunnel_setup);

			if (tmp_dev == NULL)
				goto out;

			if (strchr(tmp_dev->name, '%')) {
				err = dev_alloc_name(tmp_dev, tmp_dev->name);
				if (err < 0)
					goto add_err;
			}

			t = netdev_priv(tmp_dev);
			t->dev = tmp_dev;
			strncpy(p.name, tmp_dev->name, IFNAMSIZ);
			memcpy(&(t->parms), &p, sizeof(p));

			err = -EFAULT;
			if (copy_to_user(ifr->ifr_ifru.ifru_data, &p,
						sizeof(p)))
				goto add_err;

			err = -ENOMEM;
			tmp_dev->tstats = alloc_percpu(struct pcpu_sw_netstats);
			if (!tmp_dev->tstats)
				goto add_err;

			err = register_netdevice(tmp_dev);
			if (err < 0)
				goto add_err;

#ifdef  COMCERTO_ETHERIPV4
			t->dev->iflink  = t->parms.link; // MSPD Added
#endif
			dev_hold(tmp_dev);

			write_lock_bh(&etherip_lock);
			etherip_tunnel_add(t);
			write_unlock_bh(&etherip_lock);

		} else {
			err = -EINVAL;
			if ((t = netdev_priv(dev)) == NULL)
				goto out;
			if (dev == etherip_tunnel_dev)
				goto out;
			write_lock_bh(&etherip_lock);
			memcpy(&(t->parms), &p, sizeof(p));
			write_unlock_bh(&etherip_lock);
		}

		err = 0;
		break;
add_err:
		free_percpu(tmp_dev->tstats);
		free_netdev(tmp_dev);
		goto out;

	case SIOCDELTUNNEL:
		err = -EPERM;
		if (!capable(CAP_NET_ADMIN))
			goto out;

		err = -EFAULT;
		if (copy_from_user(&p, ifr->ifr_ifru.ifru_data,
					sizeof(p)))
			goto out;

		err = -EINVAL;
		if (dev == etherip_tunnel_dev) {
			t = etherip_tunnel_find(&p);
			if (t == NULL) {
				goto out;
			}
		} else
			t = netdev_priv(dev);

		write_lock_bh(&etherip_lock);
		etherip_tunnel_del(t);
		write_unlock_bh(&etherip_lock);

#ifdef COMCERTO_ETHERIPV4
		dev_put(t->dev);
#endif
		unregister_netdevice(t->dev);
		err = 0;

		break;
#ifdef COMCERTO_ETHERIPV4
	// MSPD added
	/* Returns Okay for EtherIP-over-IPv4 */
	case SIOCISETHIPV4TUNNEL:
		err = 0;
		break;
#endif
	default:
		err = -EINVAL;
	}

out:
	return err;
}

static const struct net_device_ops etherip_netdev_ops = {
	.ndo_start_xmit = etherip_tunnel_xmit,
	.ndo_do_ioctl   = etherip_tunnel_ioctl,
	.ndo_change_mtu = etherip_change_mtu,
	.ndo_get_stats  = etherip_get_stats,
};

static void free_etheripdev(struct net_device *dev)
{
	free_percpu(dev->tstats);
	free_netdev(dev);
}

/* device init function - called via register_netdevice
 * The tunnel is registered as an Ethernet device. This allows
 * the tunnel to be added to a bridge */
static void etherip_tunnel_setup(struct net_device *dev)
{
	ether_setup(dev);
	dev->netdev_ops      = &etherip_netdev_ops;
	dev->destructor      = free_etheripdev;
	dev->mtu             = ETH_DATA_LEN;
	dev->hard_header_len = LL_MAX_HEADER + sizeof(struct iphdr) + ETHERIP_HLEN;
#ifdef COMCERTO_ETHERIPV4
	dev->tx_queue_len	= 0; /* Being a virtual device set queue len to zero.
                                        Packets should only be queued on real interfaces */
	dev->flags	     |= IFF_POINTOPOINT;//MSPD Added
#endif
	random_ether_addr(dev->dev_addr);
}

/* receive function for EtherIP packets
 * Does some basic checks on the MAC addresses and
 * interface modes */
static int etherip_rcv(struct sk_buff *skb)
{
	struct iphdr *iph;
	struct etherip_tunnel *tunnel;
	struct net_device *dev;

	iph = ip_hdr(skb);

	read_lock_bh(&etherip_lock);
	tunnel = etherip_tunnel_locate(iph->saddr);
	if (tunnel == NULL)
		goto drop;

	dev = tunnel->dev;
	secpath_reset(skb);
	skb_pull(skb, (skb_network_header(skb)-skb->data) +
			sizeof(struct iphdr)+ETHERIP_HLEN);

	skb->dev = dev;
	skb->pkt_type = PACKET_HOST;
	skb->protocol = eth_type_trans(skb, tunnel->dev);
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	skb_dst_drop(skb);

	/* do some checks */
	if (skb->pkt_type == PACKET_HOST || skb->pkt_type == PACKET_BROADCAST)
		goto accept;

	if (skb->pkt_type == PACKET_MULTICAST &&
			(dev->mc.count > 0 || dev->flags & IFF_ALLMULTI))
		goto accept;

	if (skb->pkt_type == PACKET_OTHERHOST && dev->flags & IFF_PROMISC)
		goto accept;

drop:
	read_unlock_bh(&etherip_lock);
	kfree_skb(skb);
	return 0;

accept:
	tunnel->dev->last_rx = jiffies;
	tunnel->stats.rx_packets++;
	tunnel->stats.rx_bytes += skb->len;
	nf_reset(skb);
	netif_rx(skb);
	read_unlock_bh(&etherip_lock);
	return 0;

}

static struct net_protocol etherip_protocol = {
	.handler      = etherip_rcv,
	.err_handler  = 0,
	.no_policy    = 0,
	.netns_ok     = 1,
};

/* module init function
 * initializes the EtherIP protocol (97) and registers the initial
 * device */
static int __init etherip_init(void)
{
	int err, i;
	struct etherip_tunnel *p;

	printk(KERN_INFO BANNER1);

	for (i = 0; i < HASH_SIZE; ++i)
		INIT_LIST_HEAD(&tunnels[i]);

	printk(KERN_INFO"etherip  ----- is functional");

	if (inet_add_protocol(&etherip_protocol, IPPROTO_ETHERIP)) {
		printk(KERN_ERR "etherip: can't add protocol\n");
		return -EBUSY;
	}

	etherip_tunnel_dev = alloc_netdev(sizeof(struct etherip_tunnel),
			"ethip0", NET_NAME_UNKNOWN,
			etherip_tunnel_setup);

	if (!etherip_tunnel_dev) {
		err = -ENOMEM;
		goto err2;
	}

	err = -ENOMEM;
	etherip_tunnel_dev->tstats = alloc_percpu(struct pcpu_sw_netstats);
	if (!etherip_tunnel_dev->tstats)
		goto err1;

	p = netdev_priv(etherip_tunnel_dev);
	p->dev = etherip_tunnel_dev;
	/* set some params for iproute2 */
	strcpy(p->parms.name, "ethip0");
	p->parms.iph.protocol = IPPROTO_ETHERIP;

	if ((err = register_netdev(etherip_tunnel_dev)))
		goto err1;

out:
	return err;
err1:
	free_percpu(etherip_tunnel_dev->tstats);
	free_netdev(etherip_tunnel_dev);
err2:
	inet_del_protocol(&etherip_protocol, IPPROTO_ETHERIP);
	goto out;
}

/* destroy all tunnels */
static void __exit etherip_destroy_tunnels(void)
{
	int i;
	struct list_head *ptr;
	struct etherip_tunnel *tun;

	for (i = 0; i < HASH_SIZE; ++i) {
		list_for_each(ptr, &tunnels[i]) {
			tun = list_entry(ptr, struct etherip_tunnel, list);
			ptr = ptr->prev;
			etherip_tunnel_del(tun);
			dev_put(tun->dev);
			unregister_netdevice(tun->dev);
		}
	}
}

/* module cleanup function */
static void __exit etherip_exit(void)
{
	rtnl_lock();
	etherip_destroy_tunnels();
	unregister_netdevice(etherip_tunnel_dev);
	rtnl_unlock();
	if (inet_del_protocol(&etherip_protocol, IPPROTO_ETHERIP))
		printk(KERN_ERR "etherip: can't remove protocol\n");
}

module_init(etherip_init);
module_exit(etherip_exit);
