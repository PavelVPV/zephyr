/*
 * Copyright (c) 2021 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/bluetooth.h>
#include "mesh_test.h"
#include "mesh/adv.h"
#include "mesh/net.h"
#include "mesh/transport.h"
#include "mesh/mesh.h"
#include "mesh/foundation.h"
#include "mesh/crypto.h"

#define LOG_MODULE_NAME test_relay_latency

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_INF);

#if defined(CONFIG_BT_EXT_ADV)

#define WAIT_TIME 60 /*seconds*/

#define ADV_INT_FAST_MS 20
#define AID_MASK                    ((uint8_t)(BIT_MASK(6)))
#define AID(data)                   ((data)[0] & AID_MASK)
#define UNSEG_HDR(akf, aid)         ((akf << 6) | (aid & AID_MASK))

#define IVI(pdu)           ((pdu)[0] >> 7)
#define NID(pdu)           ((pdu)[0] & 0x7f)
#define CTL(pdu)           ((pdu)[1] >> 7)
#define TTL(pdu)           ((pdu)[1] & 0x7f)
#define SEQ(pdu)           (sys_get_be24(&pdu[2]))
#define SRC(pdu)           (sys_get_be16(&(pdu)[5]))
#define DST(pdu)           (sys_get_be16(&(pdu)[7]))

#define MSG_PARAMS_DEFAULT \
	&(struct msg_params){ \
		.src = 0x0005, \
		.dst = 0x000a, \
		.seqnum = seqnum++, \
		.send_ttl = 3, \
		.app_key = test_app_key, \
		.net_cred = &net_cred, \
	}

static struct bt_le_ext_adv *adv;
static uint32_t seqnum;
static struct bt_mesh_net_cred net_cred;

struct msg_params {
	uint16_t src;
	uint16_t dst;
	uint32_t seqnum;
	uint8_t send_ttl;
	const uint8_t *app_key;
	struct bt_mesh_net_cred *net_cred;
};

static const char *bt_hex(const void *buf, size_t len)
{
	static const char hex[] = "0123456789abcdef";
	static char str[129];
	const uint8_t *b = buf;
	size_t i;

	len = MIN(len, (sizeof(str) - 1) / 2);

	for (i = 0; i < len; i++) {
		str[i * 2]     = hex[b[i] >> 4];
		str[i * 2 + 1] = hex[b[i] & 0xf];
	}

	str[i * 2] = '\0';

	return str;
}

static void msg_send(struct msg_params *params)
{
	/* Create an application message. */
	BT_MESH_MODEL_BUF_DEFINE(msg, 0x1004, 4);
	bt_mesh_model_msg_init(&msg, 0x1004);

	net_buf_simple_add_be32(&msg, k_uptime_get_32());

	struct bt_mesh_app_crypto_ctx app_crypto = {
		.src = params->src,
		.dst = params->dst,
		.seq_num = params->seqnum,
		.iv_index = 0,
	};

	/* Encrypt the application message. */
	ASSERT_OK(bt_mesh_app_encrypt(params->app_key, &app_crypto, &msg));

//	LOG_INF("encr app: %s", bt_hex(msg.data, msg.len));

	/* Create network message. */
	struct net_buf_simple *buf = NET_BUF_SIMPLE(BT_MESH_ADV_DATA_SIZE);
	net_buf_simple_init(buf, BT_MESH_NET_HDR_LEN);

	net_buf_simple_add_u8(buf, UNSEG_HDR(1, 0 /* aid */));
	net_buf_simple_add_mem(buf, msg.data, msg.len);

	/* Encode network header. */
	net_buf_simple_push_be16(buf, params->dst);
	net_buf_simple_push_be16(buf, params->src);
	net_buf_simple_push_be24(buf, params->seqnum);
	net_buf_simple_push_u8(buf, params->send_ttl);

	net_buf_simple_push_u8(buf, (params->net_cred->nid | (app_crypto.iv_index & 1) << 7));

//	LOG_INF("net hdr: %s", bt_hex(buf->data, BT_MESH_NET_HDR_LEN));

	/* Encrypt & obfuscate network header. */
	ASSERT_OK(bt_mesh_net_encrypt(params->net_cred->enc, buf, app_crypto.iv_index, false /* proxy */));

//	LOG_INF("encr net: %s", bt_hex(buf->data, buf->len));

	ASSERT_OK(bt_mesh_net_obfuscate(buf->data, app_crypto.iv_index, params->net_cred->privacy));

//	LOG_INF("obfusc net: %s", bt_hex(buf->data, buf->len));

	/* Start advertising. */
	struct bt_le_ext_adv_start_param start = {
		.num_events = 1,
	};
	uint16_t duration;
	uint16_t adv_int;
	struct bt_data ad;

	adv_int = ADV_INT_FAST_MS;
	duration = start.num_events * (adv_int + 10);

	ad.type = BT_DATA_MESH_MESSAGE;
	ad.data_len = buf->len;
	ad.data = buf->data;

//	LOG_WRN("tx len: %d", buf->len);

	ASSERT_OK(bt_le_ext_adv_set_data(adv, &ad, 1, NULL, 0));
	ASSERT_OK(bt_le_ext_adv_start(adv, &start));
}

static void adv_sent(struct bt_le_ext_adv *adv,
		     struct bt_le_ext_adv_sent_info *info)
{
	if (info->num_sent == 0 || info->num_sent > 3) {
		LOG_INF("%d", info->num_sent);
		ASSERT_FALSE(1);
	}

	msg_send(MSG_PARAMS_DEFAULT);
}

static void adv_init(void)
{
	struct bt_le_adv_param adv_param = {
		.id = BT_ID_DEFAULT,
		.interval_min = BT_MESH_ADV_SCAN_UNIT(ADV_INT_FAST_MS),
		.interval_max = BT_MESH_ADV_SCAN_UNIT(ADV_INT_FAST_MS),
	};

	static const struct bt_le_ext_adv_cb adv_cb = {
		.sent = adv_sent,
	};

	ASSERT_OK(bt_le_ext_adv_create(&adv_param, &adv_cb, &adv));
}

static void net_cred_create(const uint8_t *key, struct bt_mesh_net_cred *cred)
{
	uint8_t p = 0;

	ASSERT_OK(bt_mesh_k2(test_net_key, &p, 1, &net_cred.nid, net_cred.enc, net_cred.privacy));
}

static void test_tester_send_timestamped_msgs(void)
{
	bt_mesh_test_cfg_set(NULL, WAIT_TIME);
	ASSERT_OK(bt_enable(NULL));

	adv_init();
	net_cred_create(test_net_key, &net_cred);

	k_sleep(K_SECONDS(1));

	msg_send(MSG_PARAMS_DEFAULT);

	k_sleep(K_SECONDS(10));

	ASSERT_OK(bt_le_ext_adv_stop(adv));

	LOG_INF("messages sent: %u", seqnum);

	PASS();
}

static struct {
	uint32_t min;
	uint32_t max;
	uint32_t avg;
} latency;
static uint32_t dropped;
static uint32_t recvd;

static void mesh_msg_recv(struct net_buf_simple *in, struct bt_mesh_net_cred *net_cred)
{
	uint32_t iv_index;
	uint16_t src_addr;
	uint16_t dst_addr;
	uint8_t ttl;
	uint32_t seqnum;
	uint8_t aid;

//	LOG_INF("obfusc net: %s", bt_hex(in->data, in->len));

	NET_BUF_SIMPLE_DEFINE(buf, BT_MESH_NET_MAX_PDU_LEN);

	net_buf_simple_reset(&buf);
	net_buf_simple_add_mem(&buf, in->data, in->len);

	iv_index = IVI(in->data);

	ASSERT_OK(bt_mesh_net_obfuscate(buf.data, iv_index, net_cred->privacy));

//	LOG_INF("encr net: %s", bt_hex(buf.data, buf.len));

	src_addr = SRC(buf.data);

	ASSERT_OK(bt_mesh_net_decrypt(net_cred->enc, &buf, iv_index, false /* proxy */));

	ttl = TTL(buf.data);
	seqnum = SEQ(buf.data);
	dst_addr = DST(buf.data);

	if (ttl != 2) {
		dropped++;
		return;
	}

	recvd++;

	/* Transport layer. */
	net_buf_simple_pull(&buf, BT_MESH_NET_HDR_LEN);

	uint8_t hdr;

	hdr = net_buf_simple_pull_u8(&buf);
	aid = AID(&hdr);
	buf.len -= 4; //APP_MIC_LEN(0);


	NET_BUF_SIMPLE_DEFINE(sdu, BT_MESH_SDU_UNSEG_MAX);
	net_buf_simple_reset(&sdu);

	struct bt_mesh_app_crypto_ctx crypto = {
		.src = src_addr,
		.dst = dst_addr,
		.seq_num = seqnum,
		.iv_index = iv_index, //BT_MESH_NET_IVI_RX(rx),
	};

	ASSERT_OK(bt_mesh_app_decrypt(test_app_key, &crypto, &buf, &sdu));

	uint16_t opcode;
	uint32_t uptime;

	opcode = net_buf_simple_pull_be16(&sdu);
	ASSERT_EQUAL(0x1004, opcode);
	uptime = net_buf_simple_pull_be32(&sdu);

	uptime = k_uptime_get_32() - uptime;

	if (latency.min == 0) {
		latency.min = uptime;
	} else if (latency.min > uptime) {
		latency.min = uptime;
	}

	if (latency.max < uptime) {
		latency.max = uptime;
	}

	latency.avg = (latency.avg + uptime) / 2;

//	LOG_INF("uptime: %u", uptime);
}

static void bt_mesh_scan_cb(const bt_addr_le_t *addr, int8_t rssi, uint8_t adv_type,
			    struct net_buf_simple *buf)
{
//	LOG_WRN("%s:%d: len: %d", __func__, __LINE__, buf->len);

	if (adv_type != BT_GAP_ADV_TYPE_ADV_NONCONN_IND) {
		LOG_WRN("%s:%d", __func__, __LINE__);
		return;
	}

	while (buf->len > 1) {
		struct net_buf_simple_state state;
		uint8_t len, type;

		len = net_buf_simple_pull_u8(buf);
		/* Check for early termination */
		if (len == 0U) {
			LOG_WRN("%s:%d", __func__, __LINE__);
			return;
		}

		if (len > buf->len) {
			LOG_WRN("%s:%d", __func__, __LINE__);
			return;
		}

		net_buf_simple_save(buf, &state);

		type = net_buf_simple_pull_u8(buf);

		buf->len = len - 1;

		if (type != BT_DATA_MESH_MESSAGE) {
			LOG_WRN("%s:%d", __func__, __LINE__);
			return;
		}

		mesh_msg_recv(buf, &net_cred);

		net_buf_simple_restore(buf, &state);
		net_buf_simple_pull(buf, len);
	}
}

static void test_tester_recv_and_check_latency(void)
{
	bt_mesh_test_cfg_set(NULL, WAIT_TIME);

	ASSERT_OK(bt_enable(NULL));

	struct bt_le_scan_param scan_param = {
			.type       = BT_HCI_LE_SCAN_PASSIVE,
			.options    = BT_LE_SCAN_OPT_NONE,
			.interval   = BT_MESH_ADV_SCAN_UNIT(1000),
			.window     = BT_MESH_ADV_SCAN_UNIT(1000)
	};

	net_cred_create(test_net_key, &net_cred);

	ASSERT_OK(bt_le_scan_start(&scan_param, bt_mesh_scan_cb));

	k_sleep(K_SECONDS(11));

	ASSERT_OK(bt_le_scan_stop());

	LOG_INF("latency: min: %u, max: %u, avg: %u", latency.min, latency.max, latency.avg);
	LOG_INF("Dropped: %u, processed: %u", dropped, recvd);

	PASS();
}

static void test_dut_relay_only(void)
{
	const struct bt_mesh_test_cfg rx_cfg = {
		.addr = 0x0001,
		.dev_key = { 0x02 },
	};

	bt_mesh_test_cfg_set(&rx_cfg, WAIT_TIME);
	bt_mesh_test_setup();

	uint8_t status;
	uint8_t transmit;
	int err;

	err = bt_mesh_cfg_beacon_set(0, rx_cfg.addr, BT_MESH_FEATURE_DISABLED, &status);
	if (err || status) {
		FAIL("Beacon Set failed (err %, status %u", err, status);
		return;
	}

	err = bt_mesh_cfg_relay_set(0, rx_cfg.addr, BT_MESH_FEATURE_ENABLED,
				    BT_MESH_TRANSMIT(0, 10), &status, &transmit);
	if (err || !status || transmit) {
		FAIL("Relay Set failed (err %d, status %u, transmit %u)", err, status, transmit);
		return;
	}

	k_sleep(K_SECONDS(10));

	PASS();

}

#define TEST_CASE(role, name, description)                     \
	{                                                      \
		.test_id = "relay_latency_" #role "_" #name,          \
		.test_descr = description,                     \
		.test_tick_f = bt_mesh_test_timeout,           \
		.test_main_f = test_##role##_##name,           \
	}

static const struct bst_test_instance test_relay_latency[] = {
	TEST_CASE(tester, send_timestamped_msgs, ""),
	TEST_CASE(tester, recv_and_check_latency, ""),
	TEST_CASE(dut, relay_only, ""),

	BSTEST_END_MARKER
};

struct bst_test_list *test_relay_latency_install(struct bst_test_list *tests)
{
	tests = bst_add_tests(tests, test_relay_latency);
	return tests;
}

#endif /* CONFIG_BT_EXT_ADV */
