
// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved

 This file is part of VL53LX and is dual licensed,
 either GPL-2.0+
 or 'BSD 3-clause "New" or "Revised" License' , at your option.
 ******************************************************************************
 */

/**
 * @file stmvl53lx_ipp_nl.c  vl53lx ipp proxy over netlink kernel side
 */
#include <linux/module.h>
#include <net/sock.h>
#include <linux/netlink.h>
#include <linux/skbuff.h>
#include <linux/version.h>

#include "stmvl53lx.h"

#include "vl53lx_platform_ipp.h"
#include "stmvl53lx_ipp.h"

#define IPP_DEBUG	1
#ifndef IPP_DEBUG
#	define _ipp_dump_work(...) (void)0
#else
#	define _ipp_dump_work(...) ipp_dump_work(__VA_ARGS__)
#endif

#define IPP_STATE_PENDING	1
#define IPP_STATE_COMPLETED	2
#define IPP_STATE_CANCELED	4

#define IPP_TIMEOUT_MS		100

/** the single netlink strut use by all instance
 * @note is NULL until set
 */
static struct sock *nl_sk;

static DEFINE_MUTEX(ipp_mutex);

/**
 * current registered daemon pid
 * @note default value 0 or later 1 is kind of invalid and will require
 * user space to connect before we can send any packet
 */
static int daemon_pid;

/**
 * next xfer_id (shared other all dev)
 * no direct us used  @@ref get_next_xfer_id (will get lock)
 * @note default to 0 what is "reserved"
 */
static int next_xfer_id;


#define ipp_err(fmt, ...) pr_err("STMVL53LX IPP Err in %s %d :" fmt "\n", \
		__func__, __LINE__, ##__VA_ARGS__)

#define ipp_warn(fmt, ...) pr_warn("STMVL53LX IPP wng in %s %d : "fmt"\n",\
		__func__, __LINE__, ##__VA_ARGS__)

#if 0
#	define ipp_dbg(fmt, ...) pr_info("IPP %s %d " fmt "\n",\
		__func__, __LINE__, ##__VA_ARGS__)
#else
#	define ipp_dbg(...) (void)0
#endif

/**
 * get and managed increment of next xfer_id
 * @note will get ipp_mutex
 * @return the xfer_id to be used
 */
static int get_next_xfer_id(void)
{
	mutex_lock(&ipp_mutex);
	next_xfer_id++;
	/*0 is reserved skip it*/
	if (next_xfer_id == 0)
		next_xfer_id = 1;
	mutex_unlock(&ipp_mutex);

	return next_xfer_id;
}

static int send_client_msg(void *msg_data, int msg_size)
{
	int rc;
	struct sk_buff *skb_out;
	struct nlmsghdr *nlh;
	void *nl_data;

	ipp_dbg("to send %d byte", msg_size);
	skb_out = nlmsg_new(msg_size, 0);
	if (!skb_out) {
		ipp_err("nlmsg_new fail\n");
		return -1;
	}

	nlh = nlmsg_put(skb_out, 0, 0, NLMSG_DONE, msg_size, 0);
	NETLINK_CB(skb_out).dst_group = 0; /* not in mcast group */

	nl_data = nlmsg_data(nlh); /*get data ptr from header*/
	memcpy(nl_data, msg_data, msg_size);

	/* FIXME do we real need to lock to send a data other nl_sk ? */
	mutex_lock(&ipp_mutex);
	rc = nlmsg_unicast(nl_sk, skb_out, daemon_pid);
	if (rc < 0)
		ipp_err("fail to send data size %d to pid %d\n",
				msg_size, daemon_pid);
	/* stat can be done here in else case */
	mutex_unlock(&ipp_mutex);

	return rc;
}

/*
 * ipp lock is held ping already handled
 */
int ipp_in_process(struct ipp_work_t *pwork)
{
	struct stmvl53lx_data *data;

	ipp_dbg("enter");
	_ipp_dump_work(pwork, IPP_WORK_MAX_PAYLOAD, STMVL53LX_CFG_MAX_DEV);

	/* work id check already done */
	data = stmvl53lx_dev_table[pwork->dev_id];
	ipp_dbg("to lock ");
	/* Release now useless ipp_mutex for below work since we may deadlock
	 * with irq path.
	 */
	mutex_unlock(&ipp_mutex);
	mutex_lock(&data->work_mutex);
	if (data->ipp.buzy == IPP_STATE_PENDING) {
		/* if  it was already handled ignore it */
		if (data->ipp.waited_xfer_id == pwork->xfer_id) {
			/* ok that is what we are expecting back */
			memcpy(&data->ipp.work_out, pwork, pwork->payload);
			data->ipp.buzy |= IPP_STATE_COMPLETED;
			ipp_dbg("to wake ipp waiter as buzy state %d",
					data->ipp.buzy);
			wake_up(&data->ipp.waitq);
			goto done_lock;
		}
	}
	/* either not waiting any more or not the expected id drop it */
	ipp_err("dev #%d ippp buzy %d xfer id %d  rcv id %d droping it",
			data->id, data->ipp.buzy, data->ipp.waited_xfer_id,
			pwork->xfer_id);
done_lock:
	mutex_unlock(&data->work_mutex);
	mutex_lock(&ipp_mutex);

	return 0;
}

int stmvl53lx_ipp_stop(struct stmvl53lx_data *data)
{
	int rc;

	rc = data->ipp.buzy;
	ipp_dbg("#%d to stop buzy %d", data->id, data->ipp.buzy);
	if (data->ipp.buzy) {
		/* set invalid wait id to discard canceled job when back */
		data->ipp.waited_xfer_id = 0;
		data->ipp.buzy |= IPP_STATE_CANCELED|IPP_STATE_COMPLETED;
		ipp_dbg("#%dto wake up worker", data->id);
		/* wake up worker or abort the thread */
		wake_up(&data->ipp.waitq);
	}

	return rc;
}

/*
 * ipp and dev lock are held
 * release and re-grabbed here
 */
int stmvl53lx_ipp_do(struct stmvl53lx_data *data,
		struct ipp_work_t *pin, struct ipp_work_t *pout)
{
	int xfer_id;
	int rc;
	bool has_timeout;

	ipp_dbg("enter");

	xfer_id = get_next_xfer_id();
	/* set xfer and device dependent part of the work */
	pin->dev_id = data->id;
	pin->xfer_id =  xfer_id;
	data->ipp.waited_xfer_id = xfer_id;
	/*  try to do it */
	rc = send_client_msg(pin, pin->payload);
	/* shall we retry if fail to send for some time or number of try ? */
	if (rc < 0) {
		rc = -1;
		ipp_err("fail to send msg %d", rc);
	} else if (data->ipp.buzy == 0) {
		/* send ok put the ipp on buzy state while locked */
		data->ipp.buzy = IPP_STATE_PENDING;
		/*  unlock now that state is marked buzy */
		mutex_unlock(&data->work_mutex);

		/* put task to wait for completion */
		ipp_dbg("to wait");
		has_timeout = !wait_event_timeout(data->ipp.waitq,
			(data->ipp.buzy != IPP_STATE_PENDING),
			msecs_to_jiffies(IPP_TIMEOUT_MS));

		/* relock the main lock */
		mutex_lock(&data->work_mutex);

		rc = (data->ipp.buzy & IPP_STATE_CANCELED) || has_timeout ?
			-1 : 0;
		if (rc) {
			ipp_dbg("waking up with from canceled/timeout ipp");
		} else {
			/* return status from the ipp itself */
			ipp_dbg("ip back with status %d", data->ipp.status);
			rc = data->ipp.status;
		}
		data->ipp.buzy = 0;/* buzy clear but locked so safe */
	} else {
		ipp_dbg("buzy still not zero %d", data->ipp.buzy);
		rc = -1;
	}


/* done_lock: */
	return rc;
}


static void stmvl53lx_nl_recv_msg(struct sk_buff *skb_in)
{
	int pid_chg = 0;
	int pid;
	struct nlmsghdr *nlh;
	struct ipp_work_t *pwork;

	ipp_dbg("Entering");

	nlh = (struct nlmsghdr *)skb_in->data;
	pid = nlh->nlmsg_pid; /*pid of sending process */

	pwork = nlmsg_data(nlh);
	if (pwork->payload < IPP_WORK_HDR_SIZE ||
			pwork->payload > IPP_WORK_MAX_PAYLOAD){
		/* invalid header size */
		ipp_err("invalid msg header size %d", pwork->payload);
		_ipp_dump_work(pwork, IPP_WORK_MAX_PAYLOAD,
			STMVL53LX_CFG_MAX_DEV);
		return;
	}

	mutex_lock(&ipp_mutex);

	if ((pwork->dev_id >= STMVL53LX_CFG_MAX_DEV) || (pwork->dev_id < 0)) {
		ipp_err("invalid dev id on msg %d", pwork->dev_id);
		_ipp_dump_work(pwork, IPP_WORK_MAX_PAYLOAD,
			STMVL53LX_CFG_MAX_DEV);
		goto done_locked;
	}

	if (pwork->process_no == stmvl53lx_ipp_ping) {
		/* in that case the payload must be exact status size only
		 * if not it is a badly format message or bad message
		 */
		if (pwork->payload !=  IPP_WORK_HDR_SIZE) {
			ipp_err("invalid ping msg size %d!=%zu ",
					pwork->payload, IPP_WORK_HDR_SIZE);
			_ipp_dump_work(pwork, IPP_WORK_MAX_PAYLOAD,
				STMVL53LX_CFG_MAX_DEV);
			goto done_locked;
		}
		/* if pid was not set or change resent all ongoing ipp */
		if (pid != daemon_pid)
			ipp_warn("pid chg %d => %d\n", daemon_pid, pid);
		else
			ipp_dbg("got ping fm pid %d\n", daemon_pid);
		daemon_pid = pid;
		pid_chg = 1;
	} else {
		ipp_in_process(pwork);
	}
 done_locked:
	mutex_unlock(&ipp_mutex);
}

int stmvl53lx_ipp_setup(struct stmvl53lx_data *data)
{
	int rc;

	mutex_lock(&ipp_mutex);

	data->ipp.buzy = 0;
	init_waitqueue_head(&data->ipp.waitq);
	ipp_dbg("now %d dev daemon pid is %d", STMVL53LX_CFG_MAX_DEV,
		daemon_pid);
	rc = 0;

	mutex_unlock(&ipp_mutex);

	return rc;
}

void stmvl53lx_ipp_cleanup(struct stmvl53lx_data *data)
{
	/* nothink to do */
}

#if !defined(OLD_NETLINK_API)
struct netlink_kernel_cfg cfg = {
	.input = stmvl53lx_nl_recv_msg
};
#endif

static int netlink_protocol_type = STMVL531_CFG_NETLINK_USER;

module_param(netlink_protocol_type, int, 0444);
MODULE_PARM_DESC(netlink_protocol_type,
	"select netlink protocol type for ipp communications");

int stmvl53lx_ipp_init(void)
{
	mutex_init(&ipp_mutex);
	daemon_pid = 1; /* pid  1 is safe should not be use for user space */

#if defined(OLD_NETLINK_API)
	nl_sk = netlink_kernel_create(&init_net,
			netlink_protocol_type,
			0,
			stmvl53lx_nl_recv_msg,
			NULL,
			THIS_MODULE);
#else
	nl_sk = netlink_kernel_create(&init_net,
			netlink_protocol_type,
			&cfg);
#endif

	return nl_sk ? 0 : -1;
}


void stmvl53lx_ipp_exit(void)
{
	if (nl_sk != NULL) {
		vl53lx_dbgmsg("releasing netlink socket");
		netlink_kernel_release(nl_sk);
		nl_sk = NULL;
	}
}


