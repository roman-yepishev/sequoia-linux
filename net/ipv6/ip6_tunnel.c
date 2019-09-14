/*
 *	IPv6 tunneling device
 *	Linux INET6 implementation
 *
 *	Authors:
 *	Ville Nuorvala		<vnuorval@tcs.hut.fi>
 *	Yasuyuki Kozakai	<kozakai@linux-ipv6.org>
 *
 *      Based on:
 *      linux/net/ipv6/sit.c and linux/net/ipv4/ipip.c
 *
 *      RFC 2473
 *
 *	This program is free software; you can redistribute it and/or
 *      modify it under the terms of the GNU General Public License
 *      as published by the Free Software Foundation; either version
 *      2 of the License, or (at your option) any later version.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/capability.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/sockios.h>
#include <linux/icmp.h>
#include <linux/if.h>
#include <linux/in.h>
#include <linux/ip.h>
#include <linux/net.h>
#include <linux/in6.h>
#include <linux/netdevice.h>
#include <linux/if_arp.h>
#include <linux/icmpv6.h>
#include <linux/init.h>
#include <linux/route.h>
#include <linux/rtnetlink.h>
#include <linux/netfilter_ipv6.h>
#include <linux/slab.h>
#include <linux/hash.h>
#include <linux/etherdevice.h>

#include <asm/uaccess.h>
#include <linux/atomic.h>

#include <net/icmp.h>
#include <net/ip.h>
#include <net/ip_tunnels.h>
#include <net/ipv6.h>
#include <net/ip6_route.h>
#include <net/addrconf.h>
#include <net/ip6_tunnel.h>
#include <net/xfrm.h>
#include <net/dsfield.h>
#include <net/inet_ecn.h>
#include <net/net_namespace.h>
#include <net/netns/generic.h>

MODULE_AUTHOR("Ville Nuorvala");
MODULE_DESCRIPTION("IPv6 tunneling device");
MODULE_LICENSE("GPL");
MODULE_ALIAS_RTNL_LINK("ip6tnl");
MODULE_ALIAS_NETDEV("ip6tnl0");

#ifdef IP6_TNL_DEBUG
#define IP6_TNL_TRACE(x...) pr_debug("%s:" x "\n", __func__)
#else
#define IP6_TNL_TRACE(x...) do {;} while(0)
#endif

#define HASH_SIZE_SHIFT  5
#define HASH_SIZE (1 << HASH_SIZE_SHIFT)

static bool log_ecn_error = true;
module_param(log_ecn_error, bool, 0644);
MODULE_PARM_DESC(log_ecn_error, "Log packets received with corrupted ECN");

static u32 HASH(const struct in6_addr *addr1, const struct in6_addr *addr2)
{
	u32 hash = ipv6_addr_hash(addr1) ^ ipv6_addr_hash(addr2);

	return hash_32(hash, HASH_SIZE_SHIFT);
}


#define for_each_ip6_tunnel_rcu(start) \
	for (t = rcu_dereference(start); t; t = rcu_dereference(t->next))

static int ip6_tnl_dev_init(struct net_device *dev);
static void ip6_tnl_dev_setup(struct net_device *dev);
static struct rtnl_link_ops ip6_link_ops __read_mostly;

static int ip6_tnl_net_id __read_mostly;
struct ip6_tnl_net {
	/* the IPv6 tunnel fallback device */
	struct net_device *fb_tnl_dev;
	/* lists for storing tunnels in use */
	struct ip6_tnl __rcu *tnls_r_l[HASH_SIZE];
	struct ip6_tnl __rcu *tnls_wc[1];
	struct ip6_tnl __rcu **tnls[2];
};

static struct net_device_stats *ip6_get_stats(struct net_device *dev)
{
	struct pcpu_sw_netstats tmp, sum = { 0 };
	int i;

	for_each_possible_cpu(i) {
		unsigned int start;
		const struct pcpu_sw_netstats *tstats =
						   per_cpu_ptr(dev->tstats, i);

		do {
			start = u64_stats_fetch_begin_irq(&tstats->syncp);
			tmp.rx_packets = tstats->rx_packets;
			tmp.rx_bytes = tstats->rx_bytes;
			tmp.tx_packets = tstats->tx_packets;
			tmp.tx_bytes =  tstats->tx_bytes;
		} while (u64_stats_fetch_retry_irq(&tstats->syncp, start));

		sum.rx_packets += tmp.rx_packets;
		sum.rx_bytes   += tmp.rx_bytes;
		sum.tx_packets += tmp.tx_packets;
		sum.tx_bytes   += tmp.tx_bytes;
	}
	dev->stats.rx_packets = sum.rx_packets;
	dev->stats.rx_bytes   = sum.rx_bytes;
	dev->stats.tx_packets = sum.tx_packets;
	dev->stats.tx_bytes   = sum.tx_bytes;
	return &dev->stats;
}

/*
 * Locking : hash tables are protected by RCU and RTNL
 */


static struct kmem_cache *mr_kmem __read_mostly;
int mr_kmem_alloced = 0;

static inline size_t  ip6_4rd_nlmsg_size(void)
{
	return NLMSG_ALIGN(sizeof(struct ip6_4rd_map_msg));
}

static int ip6_4rd_fill_node( struct sk_buff *skb, struct ip6_tnl_4rd_map_rule *mr,
			u32 pid, u32 seq,int type, unsigned int flags, int reset, unsigned int ifindex)
{
	struct ip6_4rd_map_msg *mr_msg;
	struct nlmsghdr *nlh;

	nlh = nlmsg_put(skb, pid , seq, type, sizeof(*mr_msg), flags);
	if (nlh == NULL)
		return -EMSGSIZE;

	mr_msg = nlmsg_data(nlh);
	if(reset)
	{
		memset(mr_msg,0,sizeof(*mr_msg));
		mr_msg->reset = 1;
		mr_msg->ifindex = ifindex;
		
	}
	else
	{
		//	memcpy(mr_msg,mr, sizeof(*mr_msg));
		memset(mr_msg,0,sizeof(*mr_msg));
		mr_msg->prefix = mr->prefix;
		mr_msg->prefixlen = mr->prefixlen ;
		mr_msg->relay_prefix = mr->relay_prefix;
		mr_msg->relay_suffix = mr->relay_suffix;
		mr_msg->relay_prefixlen = mr->relay_prefixlen ;
		mr_msg->relay_suffixlen = mr->relay_suffixlen ;
		mr_msg->psid_offsetlen = mr->psid_offsetlen ;
		mr_msg->eabit_len = mr->eabit_len ;
		mr_msg->entry_num = mr->entry_num ;
		mr_msg->ifindex = ifindex;
	}
	return nlmsg_end(skb, nlh);

}


static int inet6_dump4rd_mrule(struct sk_buff *skb, struct netlink_callback *cb)
{
	struct net *net = sock_net(skb->sk);
	unsigned int h, s_h;
	int s_idx, s_ip_idx;
	int idx, ip_idx;
	struct ip6_tnl_4rd_map_rule *mr ;
	int err = 0;


	struct ip6_tnl *t;
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);

	s_h = cb->args[0];
	s_idx = idx = cb->args[1];
	s_ip_idx = ip_idx = cb->args[2];

	rcu_read_lock();

	for (h = s_h; h < HASH_SIZE ; h++, s_idx = 0) {
		idx = 0;
		for_each_ip6_tunnel_rcu(ip6n->tnls_r_l[h])
		{
			if (idx < s_idx)
				goto cont_tunnel;
			if (idx > s_idx)
				s_ip_idx = 0;
			ip_idx = 0;
			read_lock(&t->ip4rd.map_lock);
			list_for_each_entry (mr, &t->ip4rd.map_list, mr_list){
				if (ip_idx < s_ip_idx)
					goto cont_mr;

				err = ip6_4rd_fill_node(skb, mr,NETLINK_CB(cb->skb).portid,
						cb->nlh->nlmsg_seq,RTM_NEW4RD, NLM_F_MULTI , 0, t->dev->ifindex);
				if (err < 0) {
					WARN_ON(err == -EMSGSIZE);
					kfree_skb(skb);
					read_unlock(&t->ip4rd.map_lock);
					goto out;
				}
cont_mr:
				ip_idx++;
			}
			read_unlock(&t->ip4rd.map_lock);
cont_tunnel:
			idx++;	
		}
	}
out:
	rcu_read_unlock();

	cb->args[0] = h;
	cb->args[1] = idx;
	cb->args[2] = ip_idx;
	
	return skb->len;
}


void ip6_4rd_notify(int event, struct ip6_tnl_4rd_map_rule *mr ,struct net_device *dev, int reset)
{
	struct sk_buff *skb;
	struct net *net = dev_net(dev);
	int err;

	err = -ENOBUFS;

	skb = nlmsg_new(ip6_4rd_nlmsg_size(), gfp_any());
	if (skb == NULL)
		goto errout;

	err = ip6_4rd_fill_node(skb, mr,0,0,event,  0, reset, dev->ifindex);
	if (err < 0) {
		/* -EMSGSIZE implies BUG in rt6_nlmsg_size() */
		WARN_ON(err == -EMSGSIZE);
		kfree_skb(skb);
		goto errout;
	}
	rtnl_notify(skb, net, 0, RTNLGRP_IPV6_IFADDR,
		    NULL, gfp_any());
	return;
errout:
	if (err < 0)
		rtnl_set_sk_err(net, RTNLGRP_IPV6_IFADDR, err);
}

static inline void
ip6_tnl_4rd_mr_destroy(char *f, struct ip6_tnl_4rd_map_rule *mr)
{
	list_del(&mr->mr_list);
	kmem_cache_free(mr_kmem, mr);
	--mr_kmem_alloced;
}

static int
ip6_tnl_4rd_mr_create(struct ip6_tnl_4rd *ip4rd, struct ip6_tnl_4rd_parm *parm, struct net_device *dev)
{
	struct ip6_tnl_4rd_map_rule *mr ;
	int err = 0;

       write_lock_bh(&parm->map_lock);
       list_for_each_entry (mr, &parm->map_list, mr_list){
               if( mr->entry_num == ip4rd->entry_num ){
                       printk(KERN_DEBUG "ip6_tnl_4rd_mr_create: map rule found update");
                       mr->prefix = ip4rd->prefix ;
                       mr->relay_prefix = ip4rd->relay_prefix;
                       mr->relay_suffix = ip4rd->relay_suffix;
                       mr->prefixlen = ip4rd->prefixlen ;
                       mr->relay_prefixlen = ip4rd->relay_prefixlen ;
                       mr->relay_suffixlen = ip4rd->relay_suffixlen ;
                       mr->psid_offsetlen = ip4rd->psid_offsetlen ;
                       mr->eabit_len = ip4rd->eabit_len ;
                       mr->entry_num = ip4rd->entry_num ;
                       goto out;
               }
       }

       mr = kmem_cache_alloc(mr_kmem, GFP_KERNEL);

       if (!mr) {
               printk(KERN_INFO "ip6_tnl_4rd_mr_create: kmem_cache_alloc fail");
               err = -1 ;
               goto out;
       }

       mr->prefix = ip4rd->prefix ;
       mr->relay_prefix = ip4rd->relay_prefix;
       mr->relay_suffix = ip4rd->relay_suffix;
       mr->prefixlen = ip4rd->prefixlen ;
       mr->relay_prefixlen = ip4rd->relay_prefixlen ;
       mr->relay_suffixlen = ip4rd->relay_suffixlen ;
       mr->psid_offsetlen = ip4rd->psid_offsetlen ;
       mr->eabit_len = ip4rd->eabit_len ;
       mr->entry_num = ip4rd->entry_num ;

       ++mr_kmem_alloced;
       list_add_tail(&mr->mr_list, &parm->map_list);

out:
	ip6_4rd_notify(RTM_NEW4RD,mr, dev,0); /* modified by MSPD */
	write_unlock_bh(&parm->map_lock);
	return err;
}

static void
ip6_tnl_4rd_mr_delete_all(struct ip6_tnl_4rd_parm *parm, struct net_device *dev)
{
	struct ip6_tnl_4rd_map_rule *mr, *mr_rule;

	write_lock_bh(&parm->map_lock);
	list_for_each_entry_safe (mr, mr_rule, &parm->map_list, mr_list){
		ip6_tnl_4rd_mr_destroy("all", mr);
	}
	ip6_4rd_notify(RTM_DEL4RD,mr, dev ,1);
	write_unlock_bh(&parm->map_lock);

}

static int
ip6_tnl_4rd_mr_delete(__u16 entry_num , struct ip6_tnl_4rd_parm *parm, struct net_device *dev)
{
	struct ip6_tnl_4rd_map_rule *mr, *mr_rule;
	int err = -1 ;

	write_lock_bh(&parm->map_lock);
	list_for_each_entry_safe (mr, mr_rule, &parm->map_list, mr_list){
		if( mr->entry_num == entry_num ){
			printk(KERN_DEBUG "ip6_tnl_4rd_mr_delete: map rule found delete");
			ip6_tnl_4rd_mr_destroy("one", mr);
			err = 0 ;
			break;
		}
	}
	ip6_4rd_notify(RTM_DEL4RD,mr,dev,0);
	write_unlock_bh(&parm->map_lock);
	return err ;
}

static void
ip6_tnl_4rd_mr_show(struct ip6_tnl_4rd_parm *parm)
{
	struct ip6_tnl_4rd_map_rule *mr;

       printk(KERN_DEBUG "-- 4rd mapping rule list\n");
       printk(KERN_DEBUG "-- entry num = %d \n",mr_kmem_alloced);

	read_lock(&parm->map_lock);
	list_for_each_entry(mr, &parm->map_list, mr_list){
		printk(KERN_DEBUG "%03d : %03d.%03d.%03d.%03d/%02d %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x/%03d %02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x:%02x%02x/%03d eabit:%03d offset:%03d \n",
			mr->entry_num,
			(ntohl(mr->prefix) >> 24) & 0xff,
			(ntohl(mr->prefix) >> 16) & 0xff,
			(ntohl(mr->prefix) >>  8) & 0xff,
			ntohl(mr->prefix) & 0xff,
			mr->prefixlen,
			mr->relay_prefix.s6_addr[0],
			mr->relay_prefix.s6_addr[1],
			mr->relay_prefix.s6_addr[2],
			mr->relay_prefix.s6_addr[3],
			mr->relay_prefix.s6_addr[4],
			mr->relay_prefix.s6_addr[5],
			mr->relay_prefix.s6_addr[6],
			mr->relay_prefix.s6_addr[7],
			mr->relay_prefix.s6_addr[8],
			mr->relay_prefix.s6_addr[9],
			mr->relay_prefix.s6_addr[10],
			mr->relay_prefix.s6_addr[11],
			mr->relay_prefix.s6_addr[12],
			mr->relay_prefix.s6_addr[13],
			mr->relay_prefix.s6_addr[14],
			mr->relay_prefix.s6_addr[15],
			mr->relay_prefixlen,
			mr->relay_suffix.s6_addr[0],
			mr->relay_suffix.s6_addr[1],
			mr->relay_suffix.s6_addr[2],
			mr->relay_suffix.s6_addr[3],
			mr->relay_suffix.s6_addr[4],
			mr->relay_suffix.s6_addr[5],
			mr->relay_suffix.s6_addr[6],
			mr->relay_suffix.s6_addr[7],
			mr->relay_suffix.s6_addr[8],
			mr->relay_suffix.s6_addr[9],
			mr->relay_suffix.s6_addr[10],
			mr->relay_suffix.s6_addr[11],
			mr->relay_suffix.s6_addr[12],
			mr->relay_suffix.s6_addr[13],
			mr->relay_suffix.s6_addr[14],
			mr->relay_suffix.s6_addr[15],
			mr->relay_suffixlen,
			mr->eabit_len,
			mr->psid_offsetlen );
	}
	read_unlock(&parm->map_lock);
}

static int
ip6_tnl_4rd_modify_daddr(struct in6_addr *daddr6, __be32 daddr4, __be16 dport4,
		struct ip6_tnl_4rd_map_rule *mr)
{
       int i, pbw0, pbi0, pbi1;
       __u32 daddr[4];
       __u32 port_set_id = 0;
       __u32 mask;
       __u32 da = ntohl(daddr4);
       __u16 dp = ntohs(dport4);
       __u32 diaddr[4];
       int port_set_id_len = ( mr->eabit_len ) - ( 32 - mr->prefixlen ) ;

       if ( port_set_id_len < 0) {
               printk(KERN_DEBUG "ip6_tnl_4rd_modify_daddr: PSID length ERROR %d\n", port_set_id_len);
               return -1;
       }

       if ( port_set_id_len > 0) {
               mask = 0xffffffff >> (32 - port_set_id_len);
               port_set_id = ( dp >> (16 - mr->psid_offsetlen - port_set_id_len ) & mask ) ;
       }

       for (i = 0; i < 4; ++i)
               daddr[i] = ntohl(mr->relay_prefix.s6_addr32[i])
                       | ntohl(mr->relay_suffix.s6_addr32[i]);

       if( mr->prefixlen < 32 ) {
               pbw0 = mr->relay_prefixlen >> 5;
               pbi0 = mr->relay_prefixlen & 0x1f;
               daddr[pbw0] |= (da << mr->prefixlen) >> pbi0;
               pbi1 = pbi0 - mr->prefixlen;
               if (pbi1 > 0)
                       daddr[pbw0+1] |= da << (32 - pbi1);
	}
       if ( port_set_id_len > 0) {
	       pbw0 = (mr->relay_prefixlen + 32 - mr->prefixlen) >> 5;
	       pbi0 = (mr->relay_prefixlen + 32 - mr->prefixlen) & 0x1f;
	       daddr[pbw0] |= (port_set_id << (32 - port_set_id_len)) >> pbi0;
	       pbi1 = pbi0 - (32 - port_set_id_len);
	       if (pbi1 > 0)
		       daddr[pbw0+1] |= port_set_id << (32 - pbi1);
       }

       memset(diaddr, 0, sizeof(diaddr));

       diaddr[2] = ( da >> 8 ) ;
       diaddr[3] = ( da << 24 ) ;
       diaddr[3] |= ( port_set_id << 8 ) ;

       for (i = 0; i < 4; ++i)
               daddr[i] = daddr[i] | diaddr[i] ;

       for (i = 0; i < 4; ++i)
               daddr6->s6_addr32[i] = htonl(daddr[i]);

       /* DBG */
       printk(KERN_DEBUG "ip6_tnl_4rd_modify_daddr: %08x %08x %08x %08x  PSID:%04x\n",
               daddr[0], daddr[1], daddr[2], daddr[3], port_set_id);

       return 0;
}

/**
 * ip6_tnl_4rd_rcv_helper - 
 *   @skb: received socket buffer
 *   @t: tunnel device
 **/

static int
ip6_tnl_4rd_rcv_helper(struct sk_buff *skb, struct ip6_tnl *t)
{
       int err = 0;
       struct iphdr *iph;

       iph = ip_hdr(skb);

       switch (iph->protocol) {
       case IPPROTO_TCP:
       case IPPROTO_UDP:
       case IPPROTO_ICMP:
       case IPPROTO_GRE:
               break;
       default:
               err = -1;
               break;
       }

       return err;
}

static int
ip6_tnl_4rd_xmit_helper(struct sk_buff *skb, struct flowi6 *fl6,
		struct ip6_tnl *t)
{
       int err = 0;
       struct iphdr *iph, *icmpiph;
       __be16  *idp;
       struct tcphdr *tcph, *icmptcph;
       struct udphdr *udph, *icmpudph;
       struct icmphdr *icmph;
       struct gre_hdr *greh;
       __u32 mask;
       __be16 *sportp = NULL;
       __be32 daddr;
       __be16 dport;
       u8 *ptr;
       int no_dst_chg = 0;
       struct ip6_tnl_4rd_map_rule *mr,*mr_tmp;
       int mr_prefixlen ;
       int count ;

       iph = ip_hdr(skb);

       daddr = iph->daddr;
       idp = &iph->id;

       ptr = (u8 *)iph;
       ptr += iph->ihl * 4;
       switch (iph->protocol) {
       case IPPROTO_TCP:
               tcph = (struct tcphdr *)ptr;
               sportp = &tcph->source;
               dport = tcph->dest;
               break;
       case IPPROTO_UDP:
               udph = (struct udphdr *)ptr;
               sportp = &udph->source;
               dport = udph->dest;
               break;
       case IPPROTO_ICMP:
               icmph = (struct icmphdr *)ptr;
               switch (icmph->type) {
               case ICMP_DEST_UNREACH:
               case ICMP_SOURCE_QUENCH:
               case ICMP_REDIRECT:
               case ICMP_TIME_EXCEEDED:
               case ICMP_PARAMETERPROB:
                       ptr = (u8 *)icmph;
                       ptr += sizeof(struct icmphdr);
                       icmpiph = (struct iphdr*)ptr;
                       if (ntohs(iph->tot_len) < icmpiph->ihl * 4 + 12) {
                               err = -1;
                               goto out;
                       }
                       daddr = icmpiph->saddr;
                       ptr += icmpiph->ihl * 4;
                       switch (icmpiph->protocol) {
                       case IPPROTO_TCP:
                               icmptcph = (struct tcphdr *)ptr;
                               sportp = &icmptcph->dest;
                               dport = icmptcph->source;
                               break;
                       case IPPROTO_UDP:
                               icmpudph = (struct udphdr *)ptr;
                               sportp = &icmpudph->dest;
                               dport = icmpudph->source;
                               break;
                       default:
                               err = -1;
                               goto out;
                       }
                       break;
               default:
                       no_dst_chg = 1;
                       break;
               }
               break;
#if 0
	//FIXME this is a GRE related change, should be enabled after porting it
       case IPPROTO_GRE:
               greh = (struct gre_hdr *)ptr;
               if(greh->protocol != GRE_PROTOCOL_PPTP){
                       err = -1;
                       goto out;
               }
               no_dst_chg = 1;
               break;
#endif
       default:
               err = -1;
               goto out;
       }

       if ( no_dst_chg == 0 ){

               count = 0;
               mr_prefixlen = 0;

               read_lock(&t->ip4rd.map_lock);
               list_for_each_entry (mr, &t->ip4rd.map_list, mr_list){
                       mask = 0xffffffff << (32 - mr->prefixlen) ;
                       if( (htonl(daddr) & mask ) == htonl( mr->prefix) ) {
                               if ( mr->prefixlen >= mr_prefixlen ){
                                       mr_prefixlen = mr->prefixlen ;
                                       mr_tmp = mr;
                                       count++;
                               }
                       }
               }

               if (count){
                       err = ip6_tnl_4rd_modify_daddr(&fl6->daddr, daddr, dport, mr_tmp );
                       if (err){
                                       read_unlock(&t->ip4rd.map_lock);
                                       goto out;
                       }
               }
               read_unlock(&t->ip4rd.map_lock);

               if(sportp && idp){
                       *idp=*sportp;
               }
       }

       iph->check = 0;
       iph->check = ip_fast_csum((unsigned char *)iph, iph->ihl);

       /* XXX: */ //FIXME this feild is not present, this shold be fixed
       //skb->local_df = 1;

out:
	return err;
}

/**
 * ip6_tnl_4rd_update_parms - update 4rd parameters
 *   @t: tunnel to be updated
 *
 * Description:
 *   ip6_tnl_4rd_update_parms() updates 4rd parameters
 **/
static void
ip6_tnl_4rd_update_parms(struct ip6_tnl *t)
{
	int pbw0, pbi0, pbi1;
	__u32 d;

	t->ip4rd.port_set_id_len = t->ip4rd.relay_suffixlen
				- t->ip4rd.relay_prefixlen
				- (32 - t->ip4rd.prefixlen);
	pbw0 = (t->ip4rd.relay_suffixlen - t->ip4rd.port_set_id_len) >> 5;
	pbi0 = (t->ip4rd.relay_suffixlen - t->ip4rd.port_set_id_len) & 0x1f;
	d = (ntohl(t->parms.laddr.s6_addr32[pbw0]) << pbi0)
		>> (32 - t->ip4rd.port_set_id_len);
	pbi1 = pbi0 - (32 - t->ip4rd.port_set_id_len);

	if (pbi1 > 0)
		d |= ntohl(t->parms.laddr.s6_addr32[pbw0+1]) >> (32 - pbi1);
	t->ip4rd.port_set_id = d;

	/* local v4 address */
	t->ip4rd.laddr4 = t->ip4rd.prefix;
	pbw0 = t->ip4rd.relay_prefixlen >> 5;
	pbi0 = t->ip4rd.relay_prefixlen & 0x1f;
	d = (ntohl(t->parms.laddr.s6_addr32[pbw0]) << pbi0)
		>> t->ip4rd.prefixlen;
	pbi1 = pbi0 - t->ip4rd.prefixlen;
	if (pbi1 > 0)
		d |= ntohl(t->parms.laddr.s6_addr32[pbw0+1]) >> (32 - pbi1);
	t->ip4rd.laddr4 |= htonl(d);
	if (t->ip4rd.port_set_id_len < 0) {
		d = ntohl(t->ip4rd.laddr4);
		d &= 0xffffffff << -t->ip4rd.port_set_id_len;
		t->ip4rd.laddr4 = htonl(d);
	}

}



struct dst_entry *ip6_tnl_dst_check(struct ip6_tnl *t)
{
	struct dst_entry *dst = t->dst_cache;

	if (dst && dst->obsolete &&
	    dst->ops->check(dst, t->dst_cookie) == NULL) {
		t->dst_cache = NULL;
		dst_release(dst);
		return NULL;
	}

	return dst;
}
EXPORT_SYMBOL_GPL(ip6_tnl_dst_check);

void ip6_tnl_dst_reset(struct ip6_tnl *t)
{
	dst_release(t->dst_cache);
	t->dst_cache = NULL;
}
EXPORT_SYMBOL_GPL(ip6_tnl_dst_reset);

void ip6_tnl_dst_store(struct ip6_tnl *t, struct dst_entry *dst)
{
	struct rt6_info *rt = (struct rt6_info *) dst;
	t->dst_cookie = rt->rt6i_node ? rt->rt6i_node->fn_sernum : 0;
	dst_release(t->dst_cache);
	t->dst_cache = dst;
}
EXPORT_SYMBOL_GPL(ip6_tnl_dst_store);

/**
 * ip6_tnl_lookup - fetch tunnel matching the end-point addresses
 *   @remote: the address of the tunnel exit-point
 *   @local: the address of the tunnel entry-point
 *
 * Return:
 *   tunnel matching given end-points if found,
 *   else fallback tunnel if its device is up,
 *   else %NULL
 **/

static struct ip6_tnl *
ip6_tnl_lookup(struct net *net, const struct in6_addr *remote, const struct in6_addr *local)
{
	unsigned int hash = HASH(remote, local);
	struct ip6_tnl *t;
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);
	struct in6_addr any;

	for_each_ip6_tunnel_rcu(ip6n->tnls_r_l[hash]) {
		if (ipv6_addr_equal(local, &t->parms.laddr) &&
		    ipv6_addr_equal(remote, &t->parms.raddr) &&
		    (t->dev->flags & IFF_UP))
			return t;
                if (t->ip4rd.prefix &&
                   ipv6_addr_equal(local, &t->parms.laddr) &&
                /* ipv6_prefix_equal(remote, &t->ip4rd.relay_prefix,
                       t->ip4rd.relay_prefixlen) && 4RD D  */
                   (t->dev->flags & IFF_UP))
                       return t;
	}

	memset(&any, 0, sizeof(any));
	hash = HASH(&any, local);
	for_each_ip6_tunnel_rcu(ip6n->tnls_r_l[hash]) {
		if (ipv6_addr_equal(local, &t->parms.laddr) &&
		    (t->dev->flags & IFF_UP))
			return t;
	}

	hash = HASH(remote, &any);
	for_each_ip6_tunnel_rcu(ip6n->tnls_r_l[hash]) {
		if (ipv6_addr_equal(remote, &t->parms.raddr) &&
		    (t->dev->flags & IFF_UP))
			return t;
	}

	t = rcu_dereference(ip6n->tnls_wc[0]);
	if (t && (t->dev->flags & IFF_UP))
		return t;

	return NULL;
}

/**
 * ip6_tnl_bucket - get head of list matching given tunnel parameters
 *   @p: parameters containing tunnel end-points
 *
 * Description:
 *   ip6_tnl_bucket() returns the head of the list matching the
 *   &struct in6_addr entries laddr and raddr in @p.
 *
 * Return: head of IPv6 tunnel list
 **/

static struct ip6_tnl __rcu **
ip6_tnl_bucket(struct ip6_tnl_net *ip6n, const struct __ip6_tnl_parm *p)
{
	const struct in6_addr *remote = &p->raddr;
	const struct in6_addr *local = &p->laddr;
	unsigned int h = 0;
	int prio = 0;

	if (!ipv6_addr_any(remote) || !ipv6_addr_any(local)) {
		prio = 1;
		h = HASH(remote, local);
	}
	return &ip6n->tnls[prio][h];
}

/**
 * ip6_tnl_link - add tunnel to hash table
 *   @t: tunnel to be added
 **/

static void
ip6_tnl_link(struct ip6_tnl_net *ip6n, struct ip6_tnl *t)
{
	struct ip6_tnl __rcu **tp = ip6_tnl_bucket(ip6n, &t->parms);

	rcu_assign_pointer(t->next , rtnl_dereference(*tp));
	rcu_assign_pointer(*tp, t);
}

/**
 * ip6_tnl_unlink - remove tunnel from hash table
 *   @t: tunnel to be removed
 **/

static void
ip6_tnl_unlink(struct ip6_tnl_net *ip6n, struct ip6_tnl *t)
{
	struct ip6_tnl __rcu **tp;
	struct ip6_tnl *iter;

	for (tp = ip6_tnl_bucket(ip6n, &t->parms);
	     (iter = rtnl_dereference(*tp)) != NULL;
	     tp = &iter->next) {
		if (t == iter) {
			rcu_assign_pointer(*tp, t->next);
			break;
		}
	}
}

static void ip6_dev_free(struct net_device *dev)
{
	free_percpu(dev->tstats);
	free_netdev(dev);
}

static int ip6_tnl_create2(struct net_device *dev)
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct net *net = dev_net(dev);
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);
	int err;

	t = netdev_priv(dev);

	err = register_netdevice(dev);
	if (err < 0)
		goto out;

	strcpy(t->parms.name, dev->name);
	dev->rtnl_link_ops = &ip6_link_ops;

	rwlock_init(&t->ip4rd.map_lock);
	INIT_LIST_HEAD(&t->ip4rd.map_list); 

	dev_hold(dev);
	ip6_tnl_link(ip6n, t);
	return 0;

out:
	return err;
}

/**
 * ip6_tnl_create - create a new tunnel
 *   @p: tunnel parameters
 *   @pt: pointer to new tunnel
 *
 * Description:
 *   Create tunnel matching given parameters.
 *
 * Return:
 *   created tunnel or NULL
 **/

static struct ip6_tnl *ip6_tnl_create(struct net *net, struct __ip6_tnl_parm *p)
{
	struct net_device *dev;
	struct ip6_tnl *t;
	char name[IFNAMSIZ];
	int err;

	if (p->name[0])
		strlcpy(name, p->name, IFNAMSIZ);
	else
		sprintf(name, "ip6tnl%%d");

	dev = alloc_netdev(sizeof(*t), name, NET_NAME_UNKNOWN,
			   ip6_tnl_dev_setup);
	if (dev == NULL)
		goto failed;

	dev_net_set(dev, net);

	t = netdev_priv(dev);
	t->parms = *p;
	t->net = dev_net(dev);
	err = ip6_tnl_create2(dev);
	if (err < 0)
		goto failed_free;

	return t;

failed_free:
	ip6_dev_free(dev);
failed:
	return NULL;
}

/**
 * ip6_tnl_locate - find or create tunnel matching given parameters
 *   @p: tunnel parameters
 *   @create: != 0 if allowed to create new tunnel if no match found
 *
 * Description:
 *   ip6_tnl_locate() first tries to locate an existing tunnel
 *   based on @parms. If this is unsuccessful, but @create is set a new
 *   tunnel device is created and registered for use.
 *
 * Return:
 *   matching tunnel or NULL
 **/

static struct ip6_tnl *ip6_tnl_locate(struct net *net,
		struct __ip6_tnl_parm *p, int create)
{
	const struct in6_addr *remote = &p->raddr;
	const struct in6_addr *local = &p->laddr;
	struct ip6_tnl __rcu **tp;
	struct ip6_tnl *t;
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);

	for (tp = ip6_tnl_bucket(ip6n, p);
	     (t = rtnl_dereference(*tp)) != NULL;
	     tp = &t->next) {
		if (ipv6_addr_equal(local, &t->parms.laddr) &&
		    ipv6_addr_equal(remote, &t->parms.raddr)) {
			if (create)
				return NULL;

			return t;
		}
	}
	if (!create)
		return NULL;
	return ip6_tnl_create(net, p);
}

/**
 * ip6_tnl_dev_uninit - tunnel device uninitializer
 *   @dev: the device to be destroyed
 *
 * Description:
 *   ip6_tnl_dev_uninit() removes tunnel from its list
 **/

static void
ip6_tnl_dev_uninit(struct net_device *dev)
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct net *net = t->net;
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);

	if (dev == ip6n->fb_tnl_dev)
		RCU_INIT_POINTER(ip6n->tnls_wc[0], NULL);
	else
		ip6_tnl_unlink(ip6n, t);
	ip6_tnl_dst_reset(t);
	dev_put(dev);
}

/**
 * parse_tvl_tnl_enc_lim - handle encapsulation limit option
 *   @skb: received socket buffer
 *
 * Return:
 *   0 if none was found,
 *   else index to encapsulation limit
 **/

__u16 ip6_tnl_parse_tlv_enc_lim(struct sk_buff *skb, __u8 *raw)
{
	const struct ipv6hdr *ipv6h = (const struct ipv6hdr *) raw;
	__u8 nexthdr = ipv6h->nexthdr;
	__u16 off = sizeof(*ipv6h);

	while (ipv6_ext_hdr(nexthdr) && nexthdr != NEXTHDR_NONE) {
		__u16 optlen = 0;
		struct ipv6_opt_hdr *hdr;
		if (raw + off + sizeof(*hdr) > skb->data &&
		    !pskb_may_pull(skb, raw - skb->data + off + sizeof (*hdr)))
			break;

		hdr = (struct ipv6_opt_hdr *) (raw + off);
		if (nexthdr == NEXTHDR_FRAGMENT) {
			struct frag_hdr *frag_hdr = (struct frag_hdr *) hdr;
			if (frag_hdr->frag_off)
				break;
			optlen = 8;
		} else if (nexthdr == NEXTHDR_AUTH) {
			optlen = (hdr->hdrlen + 2) << 2;
		} else {
			optlen = ipv6_optlen(hdr);
		}
		if (nexthdr == NEXTHDR_DEST) {
			__u16 i = off + 2;
			while (1) {
				struct ipv6_tlv_tnl_enc_lim *tel;

				/* No more room for encapsulation limit */
				if (i + sizeof (*tel) > off + optlen)
					break;

				tel = (struct ipv6_tlv_tnl_enc_lim *) &raw[i];
				/* return index of option if found and valid */
				if (tel->type == IPV6_TLV_TNL_ENCAP_LIMIT &&
				    tel->length == 1)
					return i;
				/* else jump to next option */
				if (tel->type)
					i += tel->length + 2;
				else
					i++;
			}
		}
		nexthdr = hdr->nexthdr;
		off += optlen;
	}
	return 0;
}
EXPORT_SYMBOL(ip6_tnl_parse_tlv_enc_lim);

/**
 * ip6_tnl_err - tunnel error handler
 *
 * Description:
 *   ip6_tnl_err() should handle errors in the tunnel according
 *   to the specifications in RFC 2473.
 **/

static int
ip6_tnl_err(struct sk_buff *skb, __u8 ipproto, struct inet6_skb_parm *opt,
	    u8 *type, u8 *code, int *msg, __u32 *info, int offset)
{
	const struct ipv6hdr *ipv6h = (const struct ipv6hdr *) skb->data;
	struct ip6_tnl *t;
	int rel_msg = 0;
	u8 rel_type = ICMPV6_DEST_UNREACH;
	u8 rel_code = ICMPV6_ADDR_UNREACH;
	u8 tproto;
	__u32 rel_info = 0;
	__u16 len;
	int err = -ENOENT;

	/* If the packet doesn't contain the original IPv6 header we are
	   in trouble since we might need the source address for further
	   processing of the error. */

	rcu_read_lock();
	t = ip6_tnl_lookup(dev_net(skb->dev), &ipv6h->daddr, &ipv6h->saddr);
	if (t == NULL)
		goto out;

	tproto = ACCESS_ONCE(t->parms.proto);
	if (tproto != ipproto && tproto != 0)
		goto out;

	err = 0;

	switch (*type) {
		__u32 teli;
		struct ipv6_tlv_tnl_enc_lim *tel;
		__u32 mtu;
	case ICMPV6_DEST_UNREACH:
		net_warn_ratelimited("%s: Path to destination invalid or inactive!\n",
				     t->parms.name);
		rel_msg = 1;
		break;
	case ICMPV6_TIME_EXCEED:
		if ((*code) == ICMPV6_EXC_HOPLIMIT) {
			net_warn_ratelimited("%s: Too small hop limit or routing loop in tunnel!\n",
					     t->parms.name);
			rel_msg = 1;
		}
		break;
	case ICMPV6_PARAMPROB:
		teli = 0;
		if ((*code) == ICMPV6_HDR_FIELD)
			teli = ip6_tnl_parse_tlv_enc_lim(skb, skb->data);

		if (teli && teli == *info - 2) {
			tel = (struct ipv6_tlv_tnl_enc_lim *) &skb->data[teli];
			if (tel->encap_limit == 0) {
				net_warn_ratelimited("%s: Too small encapsulation limit or routing loop in tunnel!\n",
						     t->parms.name);
				rel_msg = 1;
			}
		} else {
			net_warn_ratelimited("%s: Recipient unable to parse tunneled packet!\n",
					     t->parms.name);
		}
		break;
	case ICMPV6_PKT_TOOBIG:
		mtu = *info - offset;
		if (mtu < IPV6_MIN_MTU)
			mtu = IPV6_MIN_MTU;
		t->dev->mtu = mtu;

		len = sizeof(*ipv6h) + ntohs(ipv6h->payload_len);
		if (len > mtu) {
			rel_type = ICMPV6_PKT_TOOBIG;
			rel_code = 0;
			rel_info = mtu;
			rel_msg = 1;
		}
		break;
	}

	*type = rel_type;
	*code = rel_code;
	*info = rel_info;
	*msg = rel_msg;

out:
	rcu_read_unlock();
	return err;
}

static int
ip4ip6_err(struct sk_buff *skb, struct inet6_skb_parm *opt,
	   u8 type, u8 code, int offset, __be32 info)
{
	int rel_msg = 0;
	u8 rel_type = type;
	u8 rel_code = code;
	__u32 rel_info = ntohl(info);
	int err;
	struct sk_buff *skb2;
	const struct iphdr *eiph;
	struct rtable *rt;
	struct flowi4 fl4;

	err = ip6_tnl_err(skb, IPPROTO_IPIP, opt, &rel_type, &rel_code,
			  &rel_msg, &rel_info, offset);
	if (err < 0)
		return err;

	if (rel_msg == 0)
		return 0;

	switch (rel_type) {
	case ICMPV6_DEST_UNREACH:
		if (rel_code != ICMPV6_ADDR_UNREACH)
			return 0;
		rel_type = ICMP_DEST_UNREACH;
		rel_code = ICMP_HOST_UNREACH;
		break;
	case ICMPV6_PKT_TOOBIG:
		if (rel_code != 0)
			return 0;
		rel_type = ICMP_DEST_UNREACH;
		rel_code = ICMP_FRAG_NEEDED;
		break;
	case NDISC_REDIRECT:
		rel_type = ICMP_REDIRECT;
		rel_code = ICMP_REDIR_HOST;
	default:
		return 0;
	}

	if (!pskb_may_pull(skb, offset + sizeof(struct iphdr)))
		return 0;

	skb2 = skb_clone(skb, GFP_ATOMIC);
	if (!skb2)
		return 0;

	skb_dst_drop(skb2);

	skb_pull(skb2, offset);
	skb_reset_network_header(skb2);
	eiph = ip_hdr(skb2);

	/* Try to guess incoming interface */
	rt = ip_route_output_ports(dev_net(skb->dev), &fl4, NULL,
				   eiph->saddr, 0,
				   0, 0,
				   IPPROTO_IPIP, RT_TOS(eiph->tos), 0);
	if (IS_ERR(rt))
		goto out;

	skb2->dev = rt->dst.dev;

	/* route "incoming" packet */
	if (rt->rt_flags & RTCF_LOCAL) {
		ip_rt_put(rt);
		rt = NULL;
		rt = ip_route_output_ports(dev_net(skb->dev), &fl4, NULL,
					   eiph->daddr, eiph->saddr,
					   0, 0,
					   IPPROTO_IPIP,
					   RT_TOS(eiph->tos), 0);
		if (IS_ERR(rt) ||
		    rt->dst.dev->type != ARPHRD_TUNNEL) {
			if (!IS_ERR(rt))
				ip_rt_put(rt);
			goto out;
		}
		skb_dst_set(skb2, &rt->dst);
	} else {
		ip_rt_put(rt);
		if (ip_route_input(skb2, eiph->daddr, eiph->saddr, eiph->tos,
				   skb2->dev) ||
		    skb_dst(skb2)->dev->type != ARPHRD_TUNNEL)
			goto out;
	}

	/* change mtu on this route */
	if (rel_type == ICMP_DEST_UNREACH && rel_code == ICMP_FRAG_NEEDED) {
		if (rel_info > dst_mtu(skb_dst(skb2)))
			goto out;

		skb_dst(skb2)->ops->update_pmtu(skb_dst(skb2), NULL, skb2, rel_info);
	}
	if (rel_type == ICMP_REDIRECT)
		skb_dst(skb2)->ops->redirect(skb_dst(skb2), NULL, skb2);

	icmp_send(skb2, rel_type, rel_code, htonl(rel_info));

out:
	kfree_skb(skb2);
	return 0;
}

static int
ip6ip6_err(struct sk_buff *skb, struct inet6_skb_parm *opt,
	   u8 type, u8 code, int offset, __be32 info)
{
	int rel_msg = 0;
	u8 rel_type = type;
	u8 rel_code = code;
	__u32 rel_info = ntohl(info);
	int err;

	err = ip6_tnl_err(skb, IPPROTO_IPV6, opt, &rel_type, &rel_code,
			  &rel_msg, &rel_info, offset);
	if (err < 0)
		return err;

	if (rel_msg && pskb_may_pull(skb, offset + sizeof(struct ipv6hdr))) {
		struct rt6_info *rt;
		struct sk_buff *skb2 = skb_clone(skb, GFP_ATOMIC);

		if (!skb2)
			return 0;

		skb_dst_drop(skb2);
		skb_pull(skb2, offset);
		skb_reset_network_header(skb2);

		/* Try to guess incoming interface */
		rt = rt6_lookup(dev_net(skb->dev), &ipv6_hdr(skb2)->saddr,
				NULL, 0, 0);

		if (rt && rt->dst.dev)
			skb2->dev = rt->dst.dev;

		icmpv6_send(skb2, rel_type, rel_code, rel_info);

		ip6_rt_put(rt);

		kfree_skb(skb2);
	}

	return 0;
}

static int ip4ip6_dscp_ecn_decapsulate(const struct ip6_tnl *t,
				       const struct ipv6hdr *ipv6h,
				       struct sk_buff *skb)
{
	__u8 dsfield = ipv6_get_dsfield(ipv6h) & ~INET_ECN_MASK;

	if (t->parms.flags & IP6_TNL_F_RCV_DSCP_COPY)
		ipv4_change_dsfield(ip_hdr(skb), INET_ECN_MASK, dsfield);

	return IP6_ECN_decapsulate(ipv6h, skb);
}

static int ip6ip6_dscp_ecn_decapsulate(const struct ip6_tnl *t,
				       const struct ipv6hdr *ipv6h,
				       struct sk_buff *skb)
{
	if (t->parms.flags & IP6_TNL_F_RCV_DSCP_COPY)
		ipv6_copy_dscp(ipv6_get_dsfield(ipv6h), ipv6_hdr(skb));

	return IP6_ECN_decapsulate(ipv6h, skb);
}

__u32 ip6_tnl_get_cap(struct ip6_tnl *t,
			     const struct in6_addr *laddr,
			     const struct in6_addr *raddr)
{
	struct __ip6_tnl_parm *p = &t->parms;
	int ltype = ipv6_addr_type(laddr);
	int rtype = ipv6_addr_type(raddr);
	__u32 flags = 0;

	if (ltype == IPV6_ADDR_ANY || rtype == IPV6_ADDR_ANY) {
		flags = IP6_TNL_F_CAP_PER_PACKET;
	} else if (ltype & (IPV6_ADDR_UNICAST|IPV6_ADDR_MULTICAST) &&
		   rtype & (IPV6_ADDR_UNICAST|IPV6_ADDR_MULTICAST) &&
		   !((ltype|rtype) & IPV6_ADDR_LOOPBACK) &&
		   (!((ltype|rtype) & IPV6_ADDR_LINKLOCAL) || p->link)) {
		if (ltype&IPV6_ADDR_UNICAST)
			flags |= IP6_TNL_F_CAP_XMIT;
		if (rtype&IPV6_ADDR_UNICAST)
			flags |= IP6_TNL_F_CAP_RCV;
	}
	return flags;
}
EXPORT_SYMBOL(ip6_tnl_get_cap);

/* called with rcu_read_lock() */
int ip6_tnl_rcv_ctl(struct ip6_tnl *t,
				  const struct in6_addr *laddr,
				  const struct in6_addr *raddr)
{
	struct __ip6_tnl_parm *p = &t->parms;
	int ret = 0;
	struct net *net = t->net;

	if ((p->flags & IP6_TNL_F_CAP_RCV) ||
	    ((p->flags & IP6_TNL_F_CAP_PER_PACKET) &&
	     (ip6_tnl_get_cap(t, laddr, raddr) & IP6_TNL_F_CAP_RCV))) {
		struct net_device *ldev = NULL;

		if (p->link)
			ldev = dev_get_by_index_rcu(net, p->link);

		if ((ipv6_addr_is_multicast(laddr) ||
		     likely(ipv6_chk_addr(net, laddr, ldev, 0))) &&
		    likely(!ipv6_chk_addr(net, raddr, NULL, 0)))
			ret = 1;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ip6_tnl_rcv_ctl);

/**
 * ip6_tnl_rcv - decapsulate IPv6 packet and retransmit it locally
 *   @skb: received socket buffer
 *   @protocol: ethernet protocol ID
 *   @dscp_ecn_decapsulate: the function to decapsulate DSCP code and ECN
 *
 * Return: 0
 **/

static int ip6_tnl_rcv(struct sk_buff *skb, __u16 protocol,
		       __u8 ipproto,
		       int (*dscp_ecn_decapsulate)(const struct ip6_tnl *t,
						   const struct ipv6hdr *ipv6h,
						   struct sk_buff *skb))
{
	struct ip6_tnl *t;
	const struct ipv6hdr *ipv6h = ipv6_hdr(skb);
	u8 tproto;
	int err;

	rcu_read_lock();
	t = ip6_tnl_lookup(dev_net(skb->dev), &ipv6h->saddr, &ipv6h->daddr);
	if (t != NULL) {
		struct pcpu_sw_netstats *tstats;

		tproto = ACCESS_ONCE(t->parms.proto);
		if (tproto != ipproto && tproto != 0) {
			rcu_read_unlock();
			goto discard;
		}

		if (!xfrm6_policy_check(NULL, XFRM_POLICY_IN, skb)) {
			rcu_read_unlock();
			goto discard;
		}

		if (!ip6_tnl_rcv_ctl(t, &ipv6h->daddr, &ipv6h->saddr)) {
			t->dev->stats.rx_dropped++;
			rcu_read_unlock();
			goto discard;
		}
		skb->mac_header = skb->network_header;
		skb_reset_network_header(skb);
		skb->protocol = htons(protocol);
		memset(skb->cb, 0, sizeof(struct inet6_skb_parm));

		__skb_tunnel_rx(skb, t->dev, t->net);

		err = dscp_ecn_decapsulate(t, ipv6h, skb);
		if (unlikely(err)) {
			if (log_ecn_error)
				net_info_ratelimited("non-ECT from %pI6 with dsfield=%#x\n",
						     &ipv6h->saddr,
						     ipv6_get_dsfield(ipv6h));
			if (err > 1) {
				++t->dev->stats.rx_frame_errors;
				++t->dev->stats.rx_errors;
				rcu_read_unlock();
				goto discard;
			}
		}

                if (ip6_tnl_4rd_rcv_helper(skb, t)) {
                        rcu_read_unlock();
                        goto discard;
                }

		tstats = this_cpu_ptr(t->dev->tstats);
		u64_stats_update_begin(&tstats->syncp);
		tstats->rx_packets++;
		tstats->rx_bytes += skb->len;
		u64_stats_update_end(&tstats->syncp);

		netif_rx(skb);

		rcu_read_unlock();
		return 0;
	}
	rcu_read_unlock();
	return 1;

discard:
	kfree_skb(skb);
	return 0;
}

static int ip4ip6_rcv(struct sk_buff *skb)
{
	return ip6_tnl_rcv(skb, ETH_P_IP, IPPROTO_IPIP,
			   ip4ip6_dscp_ecn_decapsulate);
}

static int ip6ip6_rcv(struct sk_buff *skb)
{
	return ip6_tnl_rcv(skb, ETH_P_IPV6, IPPROTO_IPV6,
			   ip6ip6_dscp_ecn_decapsulate);
}

struct ipv6_tel_txoption {
	struct ipv6_txoptions ops;
	__u8 dst_opt[8];
};

static void init_tel_txopt(struct ipv6_tel_txoption *opt, __u8 encap_limit)
{
	memset(opt, 0, sizeof(struct ipv6_tel_txoption));

	opt->dst_opt[2] = IPV6_TLV_TNL_ENCAP_LIMIT;
	opt->dst_opt[3] = 1;
	opt->dst_opt[4] = encap_limit;
	opt->dst_opt[5] = IPV6_TLV_PADN;
	opt->dst_opt[6] = 1;

	opt->ops.dst0opt = (struct ipv6_opt_hdr *) opt->dst_opt;
	opt->ops.opt_nflen = 8;
}

/**
 * ip6_tnl_addr_conflict - compare packet addresses to tunnel's own
 *   @t: the outgoing tunnel device
 *   @hdr: IPv6 header from the incoming packet
 *
 * Description:
 *   Avoid trivial tunneling loop by checking that tunnel exit-point
 *   doesn't match source of incoming packet.
 *
 * Return:
 *   1 if conflict,
 *   0 else
 **/

static inline bool
ip6_tnl_addr_conflict(const struct ip6_tnl *t, const struct ipv6hdr *hdr)
{
	return ipv6_addr_equal(&t->parms.raddr, &hdr->saddr);
}

int ip6_tnl_xmit_ctl(struct ip6_tnl *t,
		     const struct in6_addr *laddr,
		     const struct in6_addr *raddr)
{
	struct __ip6_tnl_parm *p = &t->parms;
	int ret = 0;
	struct net *net = t->net;

	if ((p->flags & IP6_TNL_F_CAP_XMIT) ||
	    ((p->flags & IP6_TNL_F_CAP_PER_PACKET) &&
	     (ip6_tnl_get_cap(t, laddr, raddr) & IP6_TNL_F_CAP_XMIT))) {
		struct net_device *ldev = NULL;

		rcu_read_lock();
		if (p->link)
			ldev = dev_get_by_index_rcu(net, p->link);

		if (unlikely(!ipv6_chk_addr(net, laddr, ldev, 0)))
			pr_warn("%s xmit: Local address not yet configured!\n",
				p->name);
		else if (!ipv6_addr_is_multicast(raddr) &&
			 unlikely(ipv6_chk_addr(net, raddr, NULL, 0)))
			pr_warn("%s xmit: Routing loop! Remote address found on this node!\n",
				p->name);
		else
			ret = 1;
		rcu_read_unlock();
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ip6_tnl_xmit_ctl);

/**
 * ip6_tnl_xmit2 - encapsulate packet and send
 *   @skb: the outgoing socket buffer
 *   @dev: the outgoing tunnel device
 *   @dsfield: dscp code for outer header
 *   @fl: flow of tunneled packet
 *   @encap_limit: encapsulation limit
 *   @pmtu: Path MTU is stored if packet is too big
 *
 * Description:
 *   Build new header and do some sanity checks on the packet before sending
 *   it.
 *
 * Return:
 *   0 on success
 *   -1 fail
 *   %-EMSGSIZE message too big. return mtu in this case.
 **/

static int ip6_tnl_xmit2(struct sk_buff *skb,
			 struct net_device *dev,
			 __u8 dsfield,
			 struct flowi6 *fl6,
			 int encap_limit,
			 __u32 *pmtu)
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct net *net = t->net;
	struct net_device_stats *stats = &t->dev->stats;
	struct ipv6hdr *ipv6h = ipv6_hdr(skb);
	struct ipv6_tel_txoption opt;
	struct dst_entry *dst = NULL, *ndst = NULL;
	struct net_device *tdev;
	int mtu;
	unsigned int max_headroom = sizeof(struct ipv6hdr);
	u8 proto;
	int err = -1;
	__u8 hop_limit;    

	/* NBMA tunnel */
	if (ipv6_addr_any(&t->parms.raddr)) {
		struct in6_addr *addr6;
		struct neighbour *neigh;
		int addr_type;

		if (!skb_dst(skb))
			goto tx_err_link_failure;

		neigh = dst_neigh_lookup(skb_dst(skb),
					 &ipv6_hdr(skb)->daddr);
		if (!neigh)
			goto tx_err_link_failure;

		addr6 = (struct in6_addr *)&neigh->primary_key;
		addr_type = ipv6_addr_type(addr6);

		if (addr_type == IPV6_ADDR_ANY)
			addr6 = &ipv6_hdr(skb)->daddr;

		memcpy(&fl6->daddr, addr6, sizeof(fl6->daddr));
		neigh_release(neigh);
	} else if (!fl6->flowi6_mark)
		dst = ip6_tnl_dst_check(t);

	if (!ip6_tnl_xmit_ctl(t, &fl6->saddr, &fl6->daddr))
		goto tx_err_link_failure;

	if (!dst) {
		ndst = ip6_route_output(net, NULL, fl6);

		if (ndst->error)
			goto tx_err_link_failure;
		ndst = xfrm_lookup(net, ndst, flowi6_to_flowi(fl6), NULL, 0);
		if (IS_ERR(ndst)) {
			err = PTR_ERR(ndst);
			ndst = NULL;
			goto tx_err_link_failure;
		}
		dst = ndst;
	}

	tdev = dst->dev;

	if (tdev == dev) {
		stats->collisions++;
		net_warn_ratelimited("%s: Local routing loop detected!\n",
				     t->parms.name);
		goto tx_err_dst_release;
	}
	mtu = dst_mtu(dst) - sizeof(*ipv6h);
	if (encap_limit >= 0) {
		max_headroom += 8;
		mtu -= 8;
	}
	if (mtu < IPV6_MIN_MTU)
		mtu = IPV6_MIN_MTU;
	if (skb_dst(skb))
		skb_dst(skb)->ops->update_pmtu(skb_dst(skb), NULL, skb, mtu);
        if (!t->ip4rd.prefix) {   /* 4rd support requires Post Fragmentation, so skb->len could be greater than MTU */ 
                if (skb->len > mtu) {
                        *pmtu = mtu;
                        err = -EMSGSIZE;
                        goto tx_err_dst_release;
                }
        }                         

	if (t->ip4rd.prefix) {
		struct iphdr *iph;
		iph = ip_hdr(skb);
		hop_limit = iph->ttl;
	}
	else {
		hop_limit = t->parms.hop_limit;
	}

	skb_scrub_packet(skb, !net_eq(t->net, dev_net(dev)));

	/*
	 * Okay, now see if we can stuff it in the buffer as-is.
	 */
	max_headroom += LL_RESERVED_SPACE(tdev);

	if (skb_headroom(skb) < max_headroom || skb_shared(skb) ||
	    (skb_cloned(skb) && !skb_clone_writable(skb, 0))) {
		struct sk_buff *new_skb;

		new_skb = skb_realloc_headroom(skb, max_headroom);
		if (!new_skb)
			goto tx_err_dst_release;

		if (skb->sk)
			skb_set_owner_w(new_skb, skb->sk);
		consume_skb(skb);
		skb = new_skb;
	}
	if (fl6->flowi6_mark) {
		skb_dst_set(skb, dst);
		ndst = NULL;
	} else {
		skb_dst_set_noref(skb, dst);
	}
	skb->transport_header = skb->network_header;

	proto = fl6->flowi6_proto;
	if (encap_limit >= 0) {
		init_tel_txopt(&opt, encap_limit);
		ipv6_push_nfrag_opts(skb, &opt.ops, &proto, NULL);
	}

	if (likely(!skb->encapsulation)) {
		skb_reset_inner_headers(skb);
		skb->encapsulation = 1;
	}

	skb_push(skb, sizeof(struct ipv6hdr));
	skb_reset_network_header(skb);
	ipv6h = ipv6_hdr(skb);
	ip6_flow_hdr(ipv6h, INET_ECN_encapsulate(0, dsfield),
		     ip6_make_flowlabel(net, skb, fl6->flowlabel, false));
	ipv6h->hop_limit = hop_limit;
	ipv6h->nexthdr = proto;
	ipv6h->saddr = fl6->saddr;
	ipv6h->daddr = fl6->daddr;
	ip6tunnel_xmit(skb, dev);
	if (ndst)
		ip6_tnl_dst_store(t, ndst);
	return 0;
tx_err_link_failure:
	stats->tx_carrier_errors++;
	dst_link_failure(skb);
tx_err_dst_release:
	dst_release(ndst);
	return err;
}

static inline int
ip4ip6_tnl_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ip6_tnl *t = netdev_priv(dev);
	const struct iphdr  *iph = ip_hdr(skb);
	int encap_limit = -1;
	struct flowi6 fl6;
	__u8 dsfield;
	__u32 mtu;
	u8 tproto;
	int err;

	tproto = ACCESS_ONCE(t->parms.proto);
	if (tproto != IPPROTO_IPIP && tproto != 0)
		return -1;

	if (!(t->parms.flags & IP6_TNL_F_IGN_ENCAP_LIMIT))
		encap_limit = t->parms.encap_limit;

	memcpy(&fl6, &t->fl.u.ip6, sizeof(fl6));
	fl6.flowi6_proto = IPPROTO_IPIP;

	dsfield = ipv4_get_dsfield(iph);

	if (t->parms.flags & IP6_TNL_F_USE_ORIG_TCLASS)
		fl6.flowlabel |= htonl((__u32)iph->tos << IPV6_TCLASS_SHIFT)
					  & IPV6_TCLASS_MASK;
	if (t->parms.flags & IP6_TNL_F_USE_ORIG_FWMARK)
		fl6.flowi6_mark = skb->mark;


        if (t->ip4rd.prefix && ip6_tnl_4rd_xmit_helper(skb, &fl6, t))
                return -1;

	err = ip6_tnl_xmit2(skb, dev, dsfield, &fl6, encap_limit, &mtu);
	if (err != 0) {
		/* XXX: send ICMP error even if DF is not set. */
		if (err == -EMSGSIZE)
			icmp_send(skb, ICMP_DEST_UNREACH, ICMP_FRAG_NEEDED,
				  htonl(mtu));
		return -1;
	}

	return 0;
}

static inline int
ip6ip6_tnl_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct ipv6hdr *ipv6h = ipv6_hdr(skb);
	int encap_limit = -1;
	__u16 offset;
	struct flowi6 fl6;
	__u8 dsfield;
	__u32 mtu;
	u8 tproto;
	int err;

	tproto = ACCESS_ONCE(t->parms.proto);
	if ((tproto != IPPROTO_IPV6 && tproto != 0) ||
	    ip6_tnl_addr_conflict(t, ipv6h))
		return -1;

	offset = ip6_tnl_parse_tlv_enc_lim(skb, skb_network_header(skb));
	if (offset > 0) {
		struct ipv6_tlv_tnl_enc_lim *tel;
		tel = (struct ipv6_tlv_tnl_enc_lim *)&skb_network_header(skb)[offset];
		if (tel->encap_limit == 0) {
			icmpv6_send(skb, ICMPV6_PARAMPROB,
				    ICMPV6_HDR_FIELD, offset + 2);
			return -1;
		}
		encap_limit = tel->encap_limit - 1;
	} else if (!(t->parms.flags & IP6_TNL_F_IGN_ENCAP_LIMIT))
		encap_limit = t->parms.encap_limit;

	memcpy(&fl6, &t->fl.u.ip6, sizeof(fl6));
	fl6.flowi6_proto = IPPROTO_IPV6;

	dsfield = ipv6_get_dsfield(ipv6h);
	if (t->parms.flags & IP6_TNL_F_USE_ORIG_TCLASS)
		fl6.flowlabel |= (*(__be32 *) ipv6h & IPV6_TCLASS_MASK);
	if (t->parms.flags & IP6_TNL_F_USE_ORIG_FLOWLABEL)
		fl6.flowlabel |= ip6_flowlabel(ipv6h);
	if (t->parms.flags & IP6_TNL_F_USE_ORIG_FWMARK)
		fl6.flowi6_mark = skb->mark;

	err = ip6_tnl_xmit2(skb, dev, dsfield, &fl6, encap_limit, &mtu);
	if (err != 0) {
		if (err == -EMSGSIZE)
			icmpv6_send(skb, ICMPV6_PKT_TOOBIG, 0, mtu);
		return -1;
	}

	return 0;
}

static netdev_tx_t
ip6_tnl_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct net_device_stats *stats = &t->dev->stats;
	int ret;

	switch (skb->protocol) {
	case htons(ETH_P_IP):
                if (t->ip4rd.prefix && ip_defrag(skb, IP_DEFRAG_IP6_TNL_4RD))
                        return NETDEV_TX_OK;
		ret = ip4ip6_tnl_xmit(skb, dev);
		break;
	case htons(ETH_P_IPV6):
		ret = ip6ip6_tnl_xmit(skb, dev);
		break;
	default:
		goto tx_err;
	}

	if (ret < 0)
		goto tx_err;

	return NETDEV_TX_OK;

tx_err:
	stats->tx_errors++;
	stats->tx_dropped++;
	kfree_skb(skb);
	return NETDEV_TX_OK;
}

static void ip6_tnl_link_config(struct ip6_tnl *t)
{
	struct net_device *dev = t->dev;
	struct __ip6_tnl_parm *p = &t->parms;
	struct flowi6 *fl6 = &t->fl.u.ip6;

	memcpy(dev->dev_addr, &p->laddr, sizeof(struct in6_addr));
	memcpy(dev->broadcast, &p->raddr, sizeof(struct in6_addr));

	/* Set up flowi template */
	fl6->saddr = p->laddr;
	fl6->daddr = p->raddr;
	fl6->flowi6_oif = p->link;
	fl6->flowlabel = 0;

	if (!(p->flags&IP6_TNL_F_USE_ORIG_TCLASS))
		fl6->flowlabel |= IPV6_TCLASS_MASK & p->flowinfo;
	if (!(p->flags&IP6_TNL_F_USE_ORIG_FLOWLABEL))
		fl6->flowlabel |= IPV6_FLOWLABEL_MASK & p->flowinfo;

	p->flags &= ~(IP6_TNL_F_CAP_XMIT|IP6_TNL_F_CAP_RCV|IP6_TNL_F_CAP_PER_PACKET);
	p->flags |= ip6_tnl_get_cap(t, &p->laddr, &p->raddr);

	if (p->flags&IP6_TNL_F_CAP_XMIT && p->flags&IP6_TNL_F_CAP_RCV)
		dev->flags |= IFF_POINTOPOINT;
	else
		dev->flags &= ~IFF_POINTOPOINT;

	dev->iflink = p->link;

	if (p->flags & IP6_TNL_F_CAP_XMIT) {
		int strict = (ipv6_addr_type(&p->raddr) &
			      (IPV6_ADDR_MULTICAST|IPV6_ADDR_LINKLOCAL));

		struct rt6_info *rt = rt6_lookup(t->net,
						 &p->raddr, &p->laddr,
						 p->link, strict);

		if (rt == NULL)
			return;

		if (rt->dst.dev) {
			dev->hard_header_len = rt->dst.dev->hard_header_len +
				sizeof(struct ipv6hdr);

			dev->mtu = rt->dst.dev->mtu - sizeof(struct ipv6hdr);
			if (!(t->parms.flags & IP6_TNL_F_IGN_ENCAP_LIMIT))
				dev->mtu -= 8;

			if (dev->mtu < IPV6_MIN_MTU)
				dev->mtu = IPV6_MIN_MTU;
		}
		ip6_rt_put(rt);
	}

        if (t->ip4rd.prefix) {
                p->flags |= IP6_TNL_F_CAP_XMIT;
                p->flags |= IP6_TNL_F_CAP_RCV;
        }
}

/**
 * ip6_tnl_change - update the tunnel parameters
 *   @t: tunnel to be changed
 *   @p: tunnel configuration parameters
 *
 * Description:
 *   ip6_tnl_change() updates the tunnel parameters
 **/

static int
ip6_tnl_change(struct ip6_tnl *t, const struct __ip6_tnl_parm *p)
{
	t->parms.laddr = p->laddr;
	t->parms.raddr = p->raddr;
	t->parms.flags = p->flags;
	t->parms.hop_limit = p->hop_limit;
	t->parms.encap_limit = p->encap_limit;
	t->parms.flowinfo = p->flowinfo;
	t->parms.link = p->link;
	t->parms.proto = p->proto;
        ip6_tnl_4rd_update_parms(t);
	ip6_tnl_dst_reset(t);
	ip6_tnl_link_config(t);
	return 0;
}

static int ip6_tnl_update(struct ip6_tnl *t, struct __ip6_tnl_parm *p)
{
	struct net *net = t->net;
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);
	int err;

	ip6_tnl_unlink(ip6n, t);
	synchronize_net();
	err = ip6_tnl_change(t, p);
	ip6_tnl_link(ip6n, t);
	netdev_state_change(t->dev);
	return err;
}

static int ip6_tnl0_update(struct ip6_tnl *t, struct __ip6_tnl_parm *p)
{
	/* for default tnl0 device allow to change only the proto */
	t->parms.proto = p->proto;
	netdev_state_change(t->dev);
	return 0;
}

static void
ip6_tnl_parm_from_user(struct __ip6_tnl_parm *p, const struct ip6_tnl_parm *u)
{
	p->laddr = u->laddr;
	p->raddr = u->raddr;
	p->flags = u->flags;
	p->hop_limit = u->hop_limit;
	p->encap_limit = u->encap_limit;
	p->flowinfo = u->flowinfo;
	p->link = u->link;
	p->proto = u->proto;
	memcpy(p->name, u->name, sizeof(u->name));
}

static void
ip6_tnl_parm_to_user(struct ip6_tnl_parm *u, const struct __ip6_tnl_parm *p)
{
	u->laddr = p->laddr;
	u->raddr = p->raddr;
	u->flags = p->flags;
	u->hop_limit = p->hop_limit;
	u->encap_limit = p->encap_limit;
	u->flowinfo = p->flowinfo;
	u->link = p->link;
	u->proto = p->proto;
	memcpy(u->name, p->name, sizeof(u->name));
}

/**
 * ip6_tnl_ioctl - configure ipv6 tunnels from userspace
 *   @dev: virtual device associated with tunnel
 *   @ifr: parameters passed from userspace
 *   @cmd: command to be performed
 *
 * Description:
 *   ip6_tnl_ioctl() is used for managing IPv6 tunnels
 *   from userspace.
 *
 *   The possible commands are the following:
 *     %SIOCGETTUNNEL: get tunnel parameters for device
 *     %SIOCADDTUNNEL: add tunnel matching given tunnel parameters
 *     %SIOCCHGTUNNEL: change tunnel parameters to those given
 *     %SIOCDELTUNNEL: delete tunnel
 *
 *   The fallback device "ip6tnl0", created during module
 *   initialization, can be used for creating other tunnel devices.
 *
 * Return:
 *   0 on success,
 *   %-EFAULT if unable to copy data to or from userspace,
 *   %-EPERM if current process hasn't %CAP_NET_ADMIN set
 *   %-EINVAL if passed tunnel parameters are invalid,
 *   %-EEXIST if changing a tunnel's parameters would cause a conflict
 *   %-ENODEV if attempting to change or delete a nonexisting device
 **/

static int
ip6_tnl_ioctl(struct net_device *dev, struct ifreq *ifr, int cmd)
{
	int err = 0;
	struct ip6_tnl_parm p;
	struct __ip6_tnl_parm p1;
	struct ip6_tnl *t = netdev_priv(dev);
	struct net *net = t->net;
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);
        struct ip6_tnl_4rd ip4rd, *ip4rdp;  
        struct ip6_tnl_4rd_map_rule *mr;   

	switch (cmd) {
	case SIOCGETTUNNEL:
		if (dev == ip6n->fb_tnl_dev) {
			if (copy_from_user(&p, ifr->ifr_ifru.ifru_data, sizeof(p))) {
				err = -EFAULT;
				break;
			}
			ip6_tnl_parm_from_user(&p1, &p);
			t = ip6_tnl_locate(net, &p1, 0);
			if (t == NULL)
				t = netdev_priv(dev);
		} else {
			memset(&p, 0, sizeof(p));
		}
		ip6_tnl_parm_to_user(&p, &t->parms);
		if (copy_to_user(ifr->ifr_ifru.ifru_data, &p, sizeof(p))) {
			err = -EFAULT;
		}
		break;
	case SIOCADDTUNNEL:
	case SIOCCHGTUNNEL:
		err = -EPERM;
		if (!ns_capable(net->user_ns, CAP_NET_ADMIN))
			break;
		err = -EFAULT;
		if (copy_from_user(&p, ifr->ifr_ifru.ifru_data, sizeof(p)))
			break;
		err = -EINVAL;
		if (p.proto != IPPROTO_IPV6 && p.proto != IPPROTO_IPIP &&
		    p.proto != 0)
			break;
		ip6_tnl_parm_from_user(&p1, &p);
		t = ip6_tnl_locate(net, &p1, cmd == SIOCADDTUNNEL);
		if (cmd == SIOCCHGTUNNEL) {
			if (t != NULL) {
				if (t->dev != dev) {
					err = -EEXIST;
					break;
				}
			} else
				t = netdev_priv(dev);
			if (dev == ip6n->fb_tnl_dev)
				err = ip6_tnl0_update(t, &p1);
			else
				err = ip6_tnl_update(t, &p1);
		}
		if (t) {
			err = 0;
			ip6_tnl_parm_to_user(&p, &t->parms);
			if (copy_to_user(ifr->ifr_ifru.ifru_data, &p, sizeof(p)))
				err = -EFAULT;

		} else
			err = (cmd == SIOCADDTUNNEL ? -ENOBUFS : -ENOENT);
		break;
	case SIOCDELTUNNEL:
		err = -EPERM;
		if (!ns_capable(net->user_ns, CAP_NET_ADMIN))
			break;

		if (dev == ip6n->fb_tnl_dev) {
			err = -EFAULT;
			if (copy_from_user(&p, ifr->ifr_ifru.ifru_data, sizeof(p)))
				break;
			err = -ENOENT;
			ip6_tnl_parm_from_user(&p1, &p);
			t = ip6_tnl_locate(net, &p1, 0);
			if (t == NULL)
				break;
			err = -EPERM;
			if (t->dev == ip6n->fb_tnl_dev)
				break;
			dev = t->dev;
		}
		err = 0;
		unregister_netdevice(dev);
		break;
	case SIOCADD4RD:
	case SIOCDEL4RD:
		err = -EPERM;
		if (!capable(CAP_NET_ADMIN))
			goto done;

		err = -EFAULT;
		if (copy_from_user(&ip4rd, ifr->ifr_ifru.ifru_data, sizeof(ip4rd)))
			goto done;

		t = netdev_priv(dev);

		if (cmd == SIOCADD4RD) {

			__be32 prefix;
			struct in6_addr relay_prefix, relay_suffix;

			err = -EINVAL;

			if (ip4rd.relay_suffixlen > 64)
				goto done;

			if (ip4rd.relay_suffixlen <= ip4rd.relay_prefixlen)
				goto done;

			prefix = ip4rd.prefix & htonl(0xffffffffUL << (32 - ip4rd.prefixlen));
			if (prefix != ip4rd.prefix)
				goto done;

			ipv6_addr_prefix(&relay_prefix, &ip4rd.relay_prefix, ip4rd.relay_prefixlen);
			if (!ipv6_addr_equal(&relay_prefix, &ip4rd.relay_prefix))
				goto done;

			ipv6_addr_prefix(&relay_suffix, &ip4rd.relay_suffix, ip4rd.relay_suffixlen);
			if (!ipv6_addr_equal(&relay_suffix, &ip4rd.relay_suffix))
				goto done;


			err = ip6_tnl_4rd_mr_create(&ip4rd, &t->ip4rd, t->dev); /* modified by MSPD */

			if ( ip4rd.entry_num == 0 ){

				t->ip4rd.prefix = prefix;
				t->ip4rd.relay_prefix = relay_prefix;
				t->ip4rd.relay_suffix = relay_suffix;
				t->ip4rd.prefixlen = ip4rd.prefixlen;
				t->ip4rd.relay_prefixlen = ip4rd.relay_prefixlen;
				t->ip4rd.relay_suffixlen = ip4rd.relay_suffixlen;
				t->ip4rd.psid_offsetlen = ip4rd.psid_offsetlen;

				ip6_tnl_4rd_update_parms(t);
				ip6_tnl_dst_reset(t);
				ip6_tnl_link_config(t);
			}
			/* DBG */
			ip6_tnl_4rd_mr_show(&t->ip4rd);
		}else if(cmd == SIOCDEL4RD){
			if ( ip4rd.entry_num == 0 ){
				ip6_tnl_4rd_mr_delete_all(&t->ip4rd, t->dev);
				t->ip4rd.prefix = 0;
				memset(&t->ip4rd.relay_prefix, 0, sizeof(t->ip4rd.relay_prefix));
				memset(&t->ip4rd.relay_suffix, 0, sizeof(t->ip4rd.relay_suffix));
				t->ip4rd.prefixlen = 0;
				t->ip4rd.relay_prefixlen = 0;
				t->ip4rd.relay_suffixlen = 0;
				t->ip4rd.psid_offsetlen = 0;
				t->ip4rd.laddr4 = 0;
				t->ip4rd.port_set_id = t->ip4rd.port_set_id_len = 0;

				ip6_tnl_dst_reset(t);
				ip6_tnl_link_config(t);
			}else{
				err = ip6_tnl_4rd_mr_delete( ip4rd.entry_num , &t->ip4rd, t->dev);  /* modified by MSPD */
			}
			/* DBG */
			ip6_tnl_4rd_mr_show(&t->ip4rd);
		}else{
			printk(KERN_ERR "=== ioctl_cmd 0x%x \n",cmd );
		}
		err = 0;
		break;
        case SIOCGET4RD:
                t = netdev_priv(dev);
                ip4rdp = (struct ip6_tnl_4r *)ifr->ifr_ifru.ifru_data;
 
                read_lock(&t->ip4rd.map_lock);
                list_for_each_entry (mr, &t->ip4rd.map_list, mr_list){
                        ip4rd.relay_prefix = mr->relay_prefix;
                        ip4rd.relay_suffix = mr->relay_suffix;
                        ip4rd.prefix = mr->prefix;
                        ip4rd.relay_prefixlen = mr->relay_prefixlen;
                        ip4rd.relay_suffixlen = mr->relay_suffixlen;
                        ip4rd.prefixlen = mr->prefixlen;
                        ip4rd.eabit_len = mr->eabit_len;
                        ip4rd.psid_offsetlen = mr->psid_offsetlen;
                        ip4rd.entry_num = mr->entry_num;
 
                        if (copy_to_user(ip4rdp, &ip4rd, sizeof(ip4rd))) {
                                read_unlock(&t->ip4rd.map_lock);
                                err = -EFAULT;
                        }
                        ip4rdp++;
                }
                read_unlock(&t->ip4rd.map_lock);
                break;
	default:
		err = -EINVAL;
	}
done:
	return err;
}

/**
 * ip6_tnl_change_mtu - change mtu manually for tunnel device
 *   @dev: virtual device associated with tunnel
 *   @new_mtu: the new mtu
 *
 * Return:
 *   0 on success,
 *   %-EINVAL if mtu too small
 **/

static int
ip6_tnl_change_mtu(struct net_device *dev, int new_mtu)
{
	struct ip6_tnl *tnl = netdev_priv(dev);

	if (tnl->parms.proto == IPPROTO_IPIP) {
		if (new_mtu < 68)
			return -EINVAL;
	} else {
		if (new_mtu < IPV6_MIN_MTU)
			return -EINVAL;
	}
	if (new_mtu > 0xFFF8 - dev->hard_header_len)
		return -EINVAL;
	dev->mtu = new_mtu;
	return 0;
}


static const struct net_device_ops ip6_tnl_netdev_ops = {
	.ndo_init	= ip6_tnl_dev_init,
	.ndo_uninit	= ip6_tnl_dev_uninit,
	.ndo_start_xmit = ip6_tnl_xmit,
	.ndo_do_ioctl	= ip6_tnl_ioctl,
	.ndo_change_mtu = ip6_tnl_change_mtu,
	.ndo_get_stats	= ip6_get_stats,
};


/**
 * ip6_tnl_dev_setup - setup virtual tunnel device
 *   @dev: virtual device associated with tunnel
 *
 * Description:
 *   Initialize function pointers and device parameters
 **/

static void ip6_tnl_dev_setup(struct net_device *dev)
{
	struct ip6_tnl *t;

	dev->netdev_ops = &ip6_tnl_netdev_ops;
	dev->destructor = ip6_dev_free;

	dev->type = ARPHRD_TUNNEL6;
	dev->hard_header_len = LL_MAX_HEADER + sizeof(struct ipv6hdr);
	dev->mtu = ETH_DATA_LEN - sizeof(struct ipv6hdr);
	t = netdev_priv(dev);
	if (!(t->parms.flags & IP6_TNL_F_IGN_ENCAP_LIMIT))
		dev->mtu -= 8;
	dev->flags |= IFF_NOARP;
	dev->addr_len = sizeof(struct in6_addr);
	netif_keep_dst(dev);
	/* This perm addr will be used as interface identifier by IPv6 */
	dev->addr_assign_type = NET_ADDR_RANDOM;
	eth_random_addr(dev->perm_addr);
}


/**
 * ip6_tnl_dev_init_gen - general initializer for all tunnel devices
 *   @dev: virtual device associated with tunnel
 **/

static inline int
ip6_tnl_dev_init_gen(struct net_device *dev)
{
	struct ip6_tnl *t = netdev_priv(dev);

	t->dev = dev;
	t->net = dev_net(dev);
	dev->tstats = netdev_alloc_pcpu_stats(struct pcpu_sw_netstats);
	if (!dev->tstats)
		return -ENOMEM;
	return 0;
}

/**
 * ip6_tnl_dev_init - initializer for all non fallback tunnel devices
 *   @dev: virtual device associated with tunnel
 **/

static int ip6_tnl_dev_init(struct net_device *dev)
{
	struct ip6_tnl *t = netdev_priv(dev);
	int err = ip6_tnl_dev_init_gen(dev);

	if (err)
		return err;
	ip6_tnl_link_config(t);
	return 0;
}

/**
 * ip6_fb_tnl_dev_init - initializer for fallback tunnel device
 *   @dev: fallback device
 *
 * Return: 0
 **/

static int __net_init ip6_fb_tnl_dev_init(struct net_device *dev)
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct net *net = dev_net(dev);
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);

	t->parms.proto = IPPROTO_IPV6;
	dev_hold(dev);

	rcu_assign_pointer(ip6n->tnls_wc[0], t);
	return 0;
}

static int ip6_tnl_validate(struct nlattr *tb[], struct nlattr *data[])
{
	u8 proto;

	if (!data || !data[IFLA_IPTUN_PROTO])
		return 0;

	proto = nla_get_u8(data[IFLA_IPTUN_PROTO]);
	if (proto != IPPROTO_IPV6 &&
	    proto != IPPROTO_IPIP &&
	    proto != 0)
		return -EINVAL;

	return 0;
}

static void ip6_tnl_netlink_parms(struct nlattr *data[],
				  struct __ip6_tnl_parm *parms)
{
	memset(parms, 0, sizeof(*parms));

	if (!data)
		return;

	if (data[IFLA_IPTUN_LINK])
		parms->link = nla_get_u32(data[IFLA_IPTUN_LINK]);

	if (data[IFLA_IPTUN_LOCAL])
		nla_memcpy(&parms->laddr, data[IFLA_IPTUN_LOCAL],
			   sizeof(struct in6_addr));

	if (data[IFLA_IPTUN_REMOTE])
		nla_memcpy(&parms->raddr, data[IFLA_IPTUN_REMOTE],
			   sizeof(struct in6_addr));

	if (data[IFLA_IPTUN_TTL])
		parms->hop_limit = nla_get_u8(data[IFLA_IPTUN_TTL]);

	if (data[IFLA_IPTUN_ENCAP_LIMIT])
		parms->encap_limit = nla_get_u8(data[IFLA_IPTUN_ENCAP_LIMIT]);

	if (data[IFLA_IPTUN_FLOWINFO])
		parms->flowinfo = nla_get_be32(data[IFLA_IPTUN_FLOWINFO]);

	if (data[IFLA_IPTUN_FLAGS])
		parms->flags = nla_get_u32(data[IFLA_IPTUN_FLAGS]);

	if (data[IFLA_IPTUN_PROTO])
		parms->proto = nla_get_u8(data[IFLA_IPTUN_PROTO]);
}

static int ip6_tnl_newlink(struct net *src_net, struct net_device *dev,
			   struct nlattr *tb[], struct nlattr *data[])
{
	struct net *net = dev_net(dev);
	struct ip6_tnl *nt;

	nt = netdev_priv(dev);
	ip6_tnl_netlink_parms(data, &nt->parms);

	if (ip6_tnl_locate(net, &nt->parms, 0))
		return -EEXIST;

	return ip6_tnl_create2(dev);
}

static int ip6_tnl_changelink(struct net_device *dev, struct nlattr *tb[],
			      struct nlattr *data[])
{
	struct ip6_tnl *t = netdev_priv(dev);
	struct __ip6_tnl_parm p;
	struct net *net = t->net;
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);

	if (dev == ip6n->fb_tnl_dev)
		return -EINVAL;

	ip6_tnl_netlink_parms(data, &p);

	t = ip6_tnl_locate(net, &p, 0);

	if (t) {
		if (t->dev != dev)
			return -EEXIST;
	} else
		t = netdev_priv(dev);

	return ip6_tnl_update(t, &p);
}

static void ip6_tnl_dellink(struct net_device *dev, struct list_head *head)
{
	struct net *net = dev_net(dev);
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);

	if (dev != ip6n->fb_tnl_dev)
		unregister_netdevice_queue(dev, head);
}

static size_t ip6_tnl_get_size(const struct net_device *dev)
{
	return
		/* IFLA_IPTUN_LINK */
		nla_total_size(4) +
		/* IFLA_IPTUN_LOCAL */
		nla_total_size(sizeof(struct in6_addr)) +
		/* IFLA_IPTUN_REMOTE */
		nla_total_size(sizeof(struct in6_addr)) +
		/* IFLA_IPTUN_TTL */
		nla_total_size(1) +
		/* IFLA_IPTUN_ENCAP_LIMIT */
		nla_total_size(1) +
		/* IFLA_IPTUN_FLOWINFO */
		nla_total_size(4) +
		/* IFLA_IPTUN_FLAGS */
		nla_total_size(4) +
		/* IFLA_IPTUN_PROTO */
		nla_total_size(1) +
		0;
}

static int ip6_tnl_fill_info(struct sk_buff *skb, const struct net_device *dev)
{
	struct ip6_tnl *tunnel = netdev_priv(dev);
	struct __ip6_tnl_parm *parm = &tunnel->parms;

	if (nla_put_u32(skb, IFLA_IPTUN_LINK, parm->link) ||
	    nla_put(skb, IFLA_IPTUN_LOCAL, sizeof(struct in6_addr),
		    &parm->laddr) ||
	    nla_put(skb, IFLA_IPTUN_REMOTE, sizeof(struct in6_addr),
		    &parm->raddr) ||
	    nla_put_u8(skb, IFLA_IPTUN_TTL, parm->hop_limit) ||
	    nla_put_u8(skb, IFLA_IPTUN_ENCAP_LIMIT, parm->encap_limit) ||
	    nla_put_be32(skb, IFLA_IPTUN_FLOWINFO, parm->flowinfo) ||
	    nla_put_u32(skb, IFLA_IPTUN_FLAGS, parm->flags) ||
	    nla_put_u8(skb, IFLA_IPTUN_PROTO, parm->proto))
		goto nla_put_failure;
	return 0;

nla_put_failure:
	return -EMSGSIZE;
}

static const struct nla_policy ip6_tnl_policy[IFLA_IPTUN_MAX + 1] = {
	[IFLA_IPTUN_LINK]		= { .type = NLA_U32 },
	[IFLA_IPTUN_LOCAL]		= { .len = sizeof(struct in6_addr) },
	[IFLA_IPTUN_REMOTE]		= { .len = sizeof(struct in6_addr) },
	[IFLA_IPTUN_TTL]		= { .type = NLA_U8 },
	[IFLA_IPTUN_ENCAP_LIMIT]	= { .type = NLA_U8 },
	[IFLA_IPTUN_FLOWINFO]		= { .type = NLA_U32 },
	[IFLA_IPTUN_FLAGS]		= { .type = NLA_U32 },
	[IFLA_IPTUN_PROTO]		= { .type = NLA_U8 },
};

static struct rtnl_link_ops ip6_link_ops __read_mostly = {
	.kind		= "ip6tnl",
	.maxtype	= IFLA_IPTUN_MAX,
	.policy		= ip6_tnl_policy,
	.priv_size	= sizeof(struct ip6_tnl),
	.setup		= ip6_tnl_dev_setup,
	.validate	= ip6_tnl_validate,
	.newlink	= ip6_tnl_newlink,
	.changelink	= ip6_tnl_changelink,
	.dellink	= ip6_tnl_dellink,
	.get_size	= ip6_tnl_get_size,
	.fill_info	= ip6_tnl_fill_info,
};

static struct xfrm6_tunnel ip4ip6_handler __read_mostly = {
	.handler	= ip4ip6_rcv,
	.err_handler	= ip4ip6_err,
	.priority	=	1,
};

static struct xfrm6_tunnel ip6ip6_handler __read_mostly = {
	.handler	= ip6ip6_rcv,
	.err_handler	= ip6ip6_err,
	.priority	=	1,
};

static void __net_exit ip6_tnl_destroy_tunnels(struct net *net)
{
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);
	struct net_device *dev, *aux;
	int h;
	struct ip6_tnl *t;
	LIST_HEAD(list);

	for_each_netdev_safe(net, dev, aux)
		if (dev->rtnl_link_ops == &ip6_link_ops)
			unregister_netdevice_queue(dev, &list);

	for (h = 0; h < HASH_SIZE; h++) {
		t = rtnl_dereference(ip6n->tnls_r_l[h]);
		while (t != NULL) {
			/* If dev is in the same netns, it has already
			 * been added to the list by the previous loop.
			 */
			if (!net_eq(dev_net(t->dev), net))
				unregister_netdevice_queue(t->dev, &list);
			t = rtnl_dereference(t->next);
		}
	}

	unregister_netdevice_many(&list);
}

static int __net_init ip6_tnl_init_net(struct net *net)
{
	struct ip6_tnl_net *ip6n = net_generic(net, ip6_tnl_net_id);
	struct ip6_tnl *t = NULL;
	int err;

	ip6n->tnls[0] = ip6n->tnls_wc;
	ip6n->tnls[1] = ip6n->tnls_r_l;

	err = -ENOMEM;
	ip6n->fb_tnl_dev = alloc_netdev(sizeof(struct ip6_tnl), "ip6tnl0",
					NET_NAME_UNKNOWN, ip6_tnl_dev_setup);

	if (!ip6n->fb_tnl_dev)
		goto err_alloc_dev;
	dev_net_set(ip6n->fb_tnl_dev, net);
	ip6n->fb_tnl_dev->rtnl_link_ops = &ip6_link_ops;
	/* FB netdevice is special: we have one, and only one per netns.
	 * Allowing to move it to another netns is clearly unsafe.
	 */
	ip6n->fb_tnl_dev->features |= NETIF_F_NETNS_LOCAL;

	err = ip6_fb_tnl_dev_init(ip6n->fb_tnl_dev);
	if (err < 0)
		goto err_register;

	err = register_netdev(ip6n->fb_tnl_dev);
	if (err < 0)
		goto err_register;

	t = netdev_priv(ip6n->fb_tnl_dev);

	strcpy(t->parms.name, ip6n->fb_tnl_dev->name);
	return 0;

err_register:
	ip6_dev_free(ip6n->fb_tnl_dev);
err_alloc_dev:
	return err;
}

static void __net_exit ip6_tnl_exit_net(struct net *net)
{
	rtnl_lock();
	ip6_tnl_destroy_tunnels(net);
	rtnl_unlock();
}

static struct pernet_operations ip6_tnl_net_ops = {
	.init = ip6_tnl_init_net,
	.exit = ip6_tnl_exit_net,
	.id   = &ip6_tnl_net_id,
	.size = sizeof(struct ip6_tnl_net),
};

/**
 * ip6_tunnel_init - register protocol and reserve needed resources
 *
 * Return: 0 on success
 **/

static int __init ip6_tunnel_init(void)
{
        int err = 0;                
 
        mr_kmem = kmem_cache_create("ip6_tnl_4rd_map_rule",
                sizeof(struct ip6_tnl_4rd_map_rule), 0, SLAB_HWCACHE_ALIGN,
                NULL);
        if (!mr_kmem)
	{
		err= -ENOMEM; 
                goto out_pernet;
	}

	err = register_pernet_device(&ip6_tnl_net_ops);
	if (err < 0)
		goto out_kmem;

	err = xfrm6_tunnel_register(&ip4ip6_handler, AF_INET);
	if (err < 0) {
		pr_err("%s: can't register ip4ip6\n", __func__);
		goto out_ip4ip6;
	}

	err = xfrm6_tunnel_register(&ip6ip6_handler, AF_INET6);
	if (err < 0) {
		pr_err("%s: can't register ip6ip6\n", __func__);
		goto out_ip6ip6;
	}

	err =__rtnl_register(PF_UNSPEC, RTM_GET4RD, NULL , inet6_dump4rd_mrule, NULL);
	if(err < 0)
		goto rtnl_link_failed;

	err = rtnl_link_register(&ip6_link_ops);
	if (err < 0)
		goto rtnl_link_failed;

	return 0;

rtnl_link_failed:
	xfrm6_tunnel_deregister(&ip6ip6_handler, AF_INET6);
out_ip6ip6:
	xfrm6_tunnel_deregister(&ip4ip6_handler, AF_INET);
out_ip4ip6:
	unregister_pernet_device(&ip6_tnl_net_ops);
out_kmem:
	kmem_cache_destroy(mr_kmem);
out_pernet:
	return err;
}

/**
 * ip6_tunnel_cleanup - free resources and unregister protocol
 **/

static void __exit ip6_tunnel_cleanup(void)
{
	rtnl_link_unregister(&ip6_link_ops);
	if (xfrm6_tunnel_deregister(&ip4ip6_handler, AF_INET))
		pr_info("%s: can't deregister ip4ip6\n", __func__);

	if (xfrm6_tunnel_deregister(&ip6ip6_handler, AF_INET6))
		pr_info("%s: can't deregister ip6ip6\n", __func__);

        kmem_cache_destroy(mr_kmem);     
 
	unregister_pernet_device(&ip6_tnl_net_ops);

        rtnl_unregister(PF_UNSPEC, RTM_GET4RD);
 
}

module_init(ip6_tunnel_init);
module_exit(ip6_tunnel_cleanup);
