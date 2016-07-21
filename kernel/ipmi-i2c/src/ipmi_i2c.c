/*
 * I2C IPMI driver with software slave backend
 *
 * Copyright 2016 Anton D. Kachalov <mouse@yandex-team.ru>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.
 */

#define pr_fmt(fmt)        "ipmi-i2c: " fmt

#include <linux/i2c.h>
#include <linux/ipmi_smi.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/of.h>

#define IPMB_SLAVE_ADDR		0x20

struct ipmb_msg {
	u8 slave;
	u8 netfn;
	u8 hdr_crc;
	u8 response;
	u8 seq : 6;
	u8 response_lun : 2;
	u8 cmd;
	union {
		struct {
			u8 cmd_crc;
			u8 data[0];
		} eq;
		u8 data[0];
	} r;
};

struct ipmi_smi_i2c {
	struct ipmi_device_id	ipmi_id;
	ipmi_smi_t		intf;

	/**
	 * We assume that there can only be one outstanding request, so
	 * keep the pending message in cur_msg. We protect this from concurrent
	 * updates through send & recv calls, (and consequently opal_msg, which
	 * is in-use when cur_msg is set) with msg_lock
	 */
	spinlock_t		msg_lock;
	int			buf_off;
	struct ipmi_smi_msg	*cur_msg;
	union {
		u8		raw[0];
		struct ipmb_msg	ipmb;
	} *msg;
	struct i2c_client	*client;
	struct i2c_client	*slave;
};

static u8 ipmi_crc(const u8 *buf, u32 len)
{
	u8 crc = 0;

	while (len--) {
		crc += *buf;
		buf++;
	}

	return (crc ^ 0xff) + 1;
}

static int ipmi_i2c_start_processing(void *send_info, ipmi_smi_t intf)
{
	struct ipmi_smi_i2c *smi = send_info;

	printk("%s:\n", __func__);
	smi->intf = intf;
	return 0;
}

static void send_error_reply(struct ipmi_smi_i2c *smi,
		struct ipmi_smi_msg *msg, u8 completion_code)
{
	msg->rsp[0] = msg->data[0] | 0x4;
	msg->rsp[1] = msg->data[1];
	msg->rsp[2] = completion_code;
	msg->rsp_size = 3;
	ipmi_smi_msg_received(smi->intf, msg);
}

static void ipmi_i2c_send(void *send_info, struct ipmi_smi_msg *msg)
{
	struct ipmi_smi_i2c *smi = send_info;
	struct ipmb_msg *ipmb_msg;
	unsigned long flags;
	int comp, rc;
	size_t size;

	printk("%s\n", __func__);
	/* ensure data_len will fit in the opal_ipmi_msg buffer... */
	if (msg->data_size > IPMI_MAX_MSG_LENGTH) {
		comp = IPMI_REQ_LEN_EXCEEDED_ERR;
		goto err;
	}

	/* ... and that we at least have netfn and cmd bytes */
	if (msg->data_size < 2) {
		comp = IPMI_REQ_LEN_INVALID_ERR;
		goto err;
	}

	spin_lock_irqsave(&smi->msg_lock, flags);

	if (smi->cur_msg) {
		comp = IPMI_NODE_BUSY_ERR;
		goto err_unlock;
	}

	/* format our data for I2C block transfer */
	ipmb_msg = &smi->msg->ipmb;
	ipmb_msg->netfn = msg->data[0];
	ipmb_msg->cmd = msg->data[1];
	ipmb_msg->seq++;
	/* workaround for Emerson firmware (left shift) */
	ipmb_msg->slave = smi->client->addr << 1;
	ipmb_msg->response = smi->slave->addr << 1;
	ipmb_msg->hdr_crc = ipmi_crc(smi->msg->raw, 2);
	ipmb_msg->r.eq.cmd_crc = ipmi_crc(smi->msg->raw + 3, 3);
	if (msg->data_size > 2)
		memcpy(ipmb_msg->r.eq.data, msg->data + 2, msg->data_size - 2);

	/* data_size already includes the netfn and cmd bytes and
	   smbus excludes slave address and command as first byte */
	size = sizeof(*ipmb_msg) + msg->data_size - 2 - 2;

	printk("%s: opal_ipmi_send(%02x, %02x, %u)\n", __func__,
			ipmb_msg->netfn, ipmb_msg->cmd, size);
	smi->buf_off = 0;
	rc = i2c_smbus_write_i2c_block_data(smi->client,
					    smi->msg->raw[1],
					    size,
					    smi->msg->raw + 2);
	printk("%s:  -> %d\n", __func__, rc);

	if (!rc) {
		smi->cur_msg = msg;
		spin_unlock_irqrestore(&smi->msg_lock, flags);
		return;
	}

	comp = IPMI_ERR_UNSPECIFIED;
err_unlock:
	spin_unlock_irqrestore(&smi->msg_lock, flags);
err:
	send_error_reply(smi, msg, comp);
}

static int ipmi_i2c_recv(struct ipmi_smi_i2c *smi)
{
	struct ipmb_msg *ipmb_msg;
	struct ipmi_smi_msg *msg;
	unsigned long flags;
	uint64_t size;
	int rc;

	printk("%s: opal_ipmi_recv(msg, sz)\n", __func__);

	spin_lock_irqsave(&smi->msg_lock, flags);

	if (!smi->cur_msg) {
		spin_unlock_irqrestore(&smi->msg_lock, flags);
		pr_warn("no current message?\n");
		return 0;
	}

	msg = smi->cur_msg;
	ipmb_msg = &smi->msg->ipmb;

	size = smi->buf_off;

	/* TODO: calculate & check checksum */
	rc = 0;

	printk("%s:   -> %d (size %lld)\n", __func__,
			rc, rc == 0 ? size : 0);
	if (rc) {
		/* If came via the poll, and response was not yet ready */
#define OPAL_EMPTY 0
		if (rc == OPAL_EMPTY) {
			spin_unlock_irqrestore(&smi->msg_lock, flags);
			return 0;
		}

		smi->cur_msg = NULL;
		spin_unlock_irqrestore(&smi->msg_lock, flags);
		send_error_reply(smi, msg, IPMI_ERR_UNSPECIFIED);
		return 0;
	}

	if (size < sizeof(*ipmb_msg)) {
		spin_unlock_irqrestore(&smi->msg_lock, flags);
		pr_warn("unexpected IPMI message size %lld\n", size);
		return 0;
	}

	msg->rsp[0] = ipmb_msg->netfn;
	msg->rsp[1] = ipmb_msg->cmd;
	printk("netfn %02x  cmd %02x code %02x  len %02x\n", msg->rsp[0], msg->rsp[1], ipmb_msg->r.data[0], size - sizeof(*ipmb_msg));
	if (size > sizeof(*ipmb_msg))
		memcpy(&msg->rsp[2], ipmb_msg->r.data, size - sizeof(*ipmb_msg));
	msg->rsp_size = 2 + size - sizeof(*ipmb_msg);

	smi->cur_msg = NULL;
	spin_unlock_irqrestore(&smi->msg_lock, flags);
	ipmi_smi_msg_received(smi->intf, msg);
	return 0;
}

static void ipmi_i2c_request_events(void *send_info)
{
	printk("%s\n", __func__);
}

static void ipmi_i2c_set_run_to_completion(void *send_info,
		bool run_to_completion)
{
	printk("%s\n", __func__);
}

static void ipmi_i2c_poll(void *send_info)
{
	struct ipmi_smi_i2c *smi = send_info;

	printk("%s\n", __func__);
	ipmi_i2c_recv(smi);
}

static struct ipmi_smi_handlers ipmi_i2c_smi_handlers = {
	.owner			= THIS_MODULE,
	.start_processing	= ipmi_i2c_start_processing,
	.sender			= ipmi_i2c_send,
	.request_events		= ipmi_i2c_request_events,
	.set_run_to_completion	= ipmi_i2c_set_run_to_completion,
	.poll			= ipmi_i2c_poll,
};

static int i2c_ipmb_slave_cb(struct i2c_client *client,
			     enum i2c_slave_event event, u8 *val)
{
	struct ipmi_smi_i2c *smi = i2c_get_clientdata(client);

	switch (event) {
	case I2C_SLAVE_WRITE_RECEIVED:
		smi->msg->raw[smi->buf_off++] = *val;
		break;

	case I2C_SLAVE_READ_PROCESSED:
		break;

	case I2C_SLAVE_READ_REQUESTED:
		break;

	case I2C_SLAVE_STOP:
		ipmi_i2c_recv(smi);
		break;

	case I2C_SLAVE_WRITE_REQUESTED:
		smi->buf_off = 1;
		break;

	default:
		break;
	}

	return 0;
}

static int ipmi_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ipmi_smi_i2c *ipmi;
	struct device *dev;
	int rc;

	struct i2c_board_info info = {
		I2C_BOARD_INFO("i2c-ipmb-slave", IPMB_SLAVE_ADDR)
	};

	dev = &client->dev;

	ipmi = devm_kzalloc(dev, sizeof(*ipmi), GFP_KERNEL);
	if (!ipmi)
		return -ENOMEM;

	spin_lock_init(&ipmi->msg_lock);

	ipmi->msg = devm_kmalloc(dev,
			sizeof(*ipmi->msg) + IPMI_MAX_MSG_LENGTH,
			GFP_KERNEL);
	if (!ipmi->msg) {
		rc = -ENOMEM;
		goto err_free;
	}

	ipmi->client = client;
	printk("i2c_new_device\n");
	ipmi->slave = i2c_new_device(to_i2c_adapter(client->dev.parent),
				     &info);
	ipmi->slave->flags |= I2C_CLIENT_SLAVE;

	printk("i2c_new_device: %p\n", ipmi->slave);
	rc = i2c_slave_register(ipmi->slave, i2c_ipmb_slave_cb);
	printk("i2c_slave_register: %d\n", rc);
	i2c_set_clientdata(client, ipmi);
	i2c_set_clientdata(ipmi->slave, ipmi);
	if (rc) {
		goto err_free;
	}

	/* todo: query actual ipmi_device_id */
	rc = ipmi_register_smi(&ipmi_i2c_smi_handlers, ipmi,
			&ipmi->ipmi_id, dev, IPMB_SLAVE_ADDR << 1);
	printk("ipmi_register_smi: %d\n", rc);
	if (rc) {
		dev_warn(dev, "IPMI SMI registration failed (%d)\n", rc);
		goto err_free_msg;
	}

	return 0;

err_free_msg:
	i2c_slave_unregister(ipmi->slave);
//	devm_kfree(dev, ipmi->opal_msg);
err_free:
	devm_kfree(dev, ipmi);
	return rc;
}

static int ipmi_i2c_remove(struct i2c_client *client)
{
	struct ipmi_smi_i2c *smi = i2c_get_clientdata(client);

	i2c_slave_unregister(smi->slave);
	ipmi_unregister_smi(smi->intf);

	return 0;
}

static const struct i2c_device_id i2c_ipmb_slave_id[] = {
	{ "i2c-ipmb-slave", 0 },
	{ },
};

static int i2c_ipmb_slave_probe(struct i2c_client *client,
			        const struct i2c_device_id *id)
{
	return 0;
}

static int i2c_ipmb_slave_remove(struct i2c_client *client)
{
	return 0;
}

static struct i2c_driver i2c_ipmb_slave_driver = {
	.driver.name	= "i2c-ipmb-slave",
	.probe		= i2c_ipmb_slave_probe,
	.remove		= i2c_ipmb_slave_remove,
	.id_table	= i2c_ipmb_slave_id,
};

static const struct i2c_device_id ipmi_i2c_ids[] = {
	{ "aspeed,i2c-ipmb", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, ipmi_i2c_ids);

static struct i2c_driver i2c_ipmi_driver = {
	.driver = {
		.name		= "ipmi-i2c",
	},
	.probe	= ipmi_i2c_probe,
	.remove	= ipmi_i2c_remove,
	.id_table = ipmi_i2c_ids,
};

static int __init ipmi_i2c_init(void)
{
	int rc;
	rc = i2c_add_driver(&i2c_ipmb_slave_driver);
	if (rc)
		goto err_out;

	rc = i2c_add_driver(&i2c_ipmi_driver);
	if (rc)
		i2c_del_driver(&i2c_ipmb_slave_driver);
err_out:
	return rc;
}

static void __exit ipmi_i2c_exit(void)
{
	i2c_del_driver(&i2c_ipmi_driver);
	i2c_del_driver(&i2c_ipmb_slave_driver);
}

postcore_initcall(ipmi_i2c_init);
module_exit(ipmi_i2c_exit);

MODULE_DESCRIPTION("I2C IPMI driver");
MODULE_AUTHOR("Anton D. Kachalov <mouse@yandex-team.ru>");
MODULE_LICENSE("GPL");
