/*
 * Copyright (c) 2024 Nordic Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "bstests.h"
#include "common.h"

#include "host/conn_internal.h"

#include <zephyr/settings/settings.h>
#include <zephyr/storage/flash_map.h>

#define LOG_MODULE_NAME main
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME, LOG_LEVEL_DBG);

CREATE_FLAG(is_connected);
CREATE_FLAG(flag_l2cap_connected);

#define SDU_NUM          2000
#define SDU_LEN          27
#define RESCHEDULE_DELAY K_MSEC(100)
#define CHANNELS         4
#define FLASH_PAGE_SIZE  4096

static struct k_work_q dut_work_q;
static K_THREAD_STACK_DEFINE(dut_work_stack, 1024);

static void heavy_work_handler(struct k_work *work)
{
	LOG_DBG("Heavy work started");
	k_busy_wait(100 * 1000);
	LOG_DBG("Heavy work done");

	k_sleep(K_MSEC(1));
	k_work_submit_to_queue(&dut_work_q, work);
}

static K_WORK_DEFINE(heavy_work, heavy_work_handler);

struct bt_conn *default_conn;

static void sdu_destroy(struct net_buf *buf)
{
	LOG_DBG("%p", buf);

	net_buf_destroy(buf);
}

static void rx_destroy(struct net_buf *buf)
{
	LOG_DBG("%p", buf);

	net_buf_destroy(buf);
}

/* Only one SDU per link will be transmitted at a time */
NET_BUF_POOL_DEFINE(sdu_tx_pool,
		    CONFIG_BT_MAX_CONN * CHANNELS, BT_L2CAP_SDU_BUF_SIZE(SDU_LEN),
		    CONFIG_BT_CONN_TX_USER_DATA_SIZE, sdu_destroy);

/* Only one SDU per link will be received at a time */
NET_BUF_POOL_DEFINE(sdu_rx_pool,
		    CONFIG_BT_MAX_CONN * CHANNELS, BT_L2CAP_SDU_BUF_SIZE(SDU_LEN),
		    8, rx_destroy);

static uint8_t tx_data[SDU_LEN];
static uint16_t rx_cnt;

struct test_ctx {
	struct k_work_delayable work_item;
	struct bt_l2cap_le_chan le_chan;
	size_t tx_left;
};

static struct test_ctx test_ctx[CHANNELS];

struct test_ctx *get_ctx(struct bt_l2cap_chan *chan)
{
	struct bt_l2cap_le_chan *le_chan = CONTAINER_OF(chan, struct bt_l2cap_le_chan, chan);
	struct test_ctx *ctx = CONTAINER_OF(le_chan, struct test_ctx, le_chan);

	return ctx;
}

int l2cap_chan_send(struct bt_l2cap_chan *chan, uint8_t *data, size_t len)
{
	LOG_DBG("chan %p conn %u data %p len %d", chan, bt_conn_index(chan->conn), data, len);

	struct net_buf *buf = net_buf_alloc(&sdu_tx_pool, K_NO_WAIT);

	if (buf == NULL) {
		FAIL("No more memory\n");
		return -ENOMEM;
	}

	net_buf_reserve(buf, BT_L2CAP_SDU_CHAN_SEND_RESERVE);
	net_buf_add_mem(buf, data, len);

	int ret = bt_l2cap_chan_send(chan, buf);

	if (ret == -EAGAIN) {
		LOG_DBG("L2CAP error %d, attempting to reschedule sending", ret);
		net_buf_unref(buf);
		k_work_reschedule(&(get_ctx(chan)->work_item), RESCHEDULE_DELAY);

		return ret;
	}

	if (ret < 0) {
		LOG_WRN("Failed sending: err %d", ret);
	} else {
		LOG_DBG("sent %d len %d", ret, len);
	}

	return ret;
}

struct net_buf *alloc_buf_cb(struct bt_l2cap_chan *chan)
{
	return net_buf_alloc(&sdu_rx_pool, K_NO_WAIT);
}

void continue_sending(struct test_ctx *ctx)
{
	struct bt_l2cap_chan *chan = &ctx->le_chan.chan;

	LOG_DBG("%p, left %d", chan, ctx->tx_left);

	if (ctx->tx_left) {
		l2cap_chan_send(chan, tx_data, sizeof(tx_data));
	} else {
		LOG_DBG("Done sending %u", bt_conn_index(chan->conn));
	}
}

void sent_cb(struct bt_l2cap_chan *chan)
{
	struct test_ctx *ctx = get_ctx(chan);

	LOG_DBG("%p", chan);

	if (ctx->tx_left) {
		ctx->tx_left--;
	}

	continue_sending(ctx);
}

int recv_cb(struct bt_l2cap_chan *chan, struct net_buf *buf)
{
	LOG_DBG("len %d", buf->len);
	rx_cnt++;

	/* Verify SDU data matches TX'd data. */
	int pos = memcmp(buf->data, tx_data, buf->len);

	if (pos != 0) {
		LOG_ERR("RX data doesn't match TX: pos %d", pos);
		LOG_HEXDUMP_ERR(buf->data, buf->len, "RX data");
		LOG_HEXDUMP_INF(tx_data, buf->len, "TX data");

		for (uint16_t p = 0; p < buf->len; p++) {
			__ASSERT(buf->data[p] == tx_data[p],
				 "Failed rx[%d]=%x != expect[%d]=%x",
				 p, buf->data[p], p, tx_data[p]);
		}
	}

	/* We need to call any random API which calls bt_hci_cmd_send_sync inside. This should
	 * result in Command Complete event from the controller which we will never receive because
	 * the hci_rx_pool is full and we are holding the buffer from being released by sitting here
	 * in the receive callback.
	 */
	struct bt_conn_le_tx_power power_level = {0};
	int err;

	printk("Sleeping to fill up evt buffer\n");

#if 0
	/* Sleep for a while to fill up the hci_rx_pool */
	k_sleep(K_MSEC(100));
#endif

	printk("Triggering deadlock in host\n");

	err = bt_conn_le_get_tx_power_level(default_conn, &power_level);
	if (err) {
		FAIL("Failed to get tx power level (err %d)", err);
	}

	printf("Tx power level: %d\n", power_level.current_level);

	return 0;
}

void l2cap_chan_connected_cb(struct bt_l2cap_chan *l2cap_chan)
{
	struct bt_l2cap_le_chan *chan =
		CONTAINER_OF(l2cap_chan, struct bt_l2cap_le_chan, chan);

	SET_FLAG(flag_l2cap_connected);
	LOG_DBG("%x (tx mtu %d mps %d) (tx mtu %d mps %d)",
		l2cap_chan,
		chan->tx.mtu,
		chan->tx.mps,
		chan->rx.mtu,
		chan->rx.mps);
}

void l2cap_chan_disconnected_cb(struct bt_l2cap_chan *chan)
{
	UNSET_FLAG(flag_l2cap_connected);
	LOG_DBG("%p", chan);
}

static struct bt_l2cap_chan_ops ops = {
	.connected = l2cap_chan_connected_cb,
	.disconnected = l2cap_chan_disconnected_cb,
	.alloc_buf = alloc_buf_cb,
	.recv = recv_cb,
	.sent = sent_cb,
};

void deferred_send(struct k_work *item)
{
	struct test_ctx *ctx = CONTAINER_OF(k_work_delayable_from_work(item),
					    struct test_ctx, work_item);

	struct bt_l2cap_chan *chan = &ctx->le_chan.chan;

	LOG_DBG("continue %u left %d", bt_conn_index(chan->conn), ctx->tx_left);

	continue_sending(ctx);
}

struct test_ctx *alloc_test_context(void)
{
	static size_t ch;
	struct test_ctx *ctx = &test_ctx[ch];
	struct bt_l2cap_le_chan *le_chan = &ctx->le_chan;

	__ASSERT(le_chan->state == BT_L2CAP_DISCONNECTED,
		 "Channel is not in disconnected state");

	ch++;
	__ASSERT(ch <= CHANNELS, "No more available channels");

	memset(ctx, 0, sizeof(struct test_ctx));
	k_work_init_delayable(&ctx->work_item, deferred_send);

	return ctx;
}

int server_accept_cb(struct bt_conn *conn, struct bt_l2cap_server *server,
		     struct bt_l2cap_chan **chan)
{
	struct test_ctx *ctx;
	struct bt_l2cap_le_chan *le_chan;

	ctx = alloc_test_context();
	le_chan = &ctx->le_chan;

	memset(le_chan, 0, sizeof(*le_chan));
	le_chan->chan.ops = &ops;
	le_chan->rx.mtu = SDU_LEN;
	*chan = &le_chan->chan;

	return 0;
}

static struct bt_l2cap_server test_l2cap_server = {
	.accept = server_accept_cb
};

static int l2cap_server_register(bt_security_t sec_level)
{
	test_l2cap_server.psm = 0;
	test_l2cap_server.sec_level = sec_level;

	int err = bt_l2cap_server_register(&test_l2cap_server);

	ASSERT(err == 0, "Failed to register l2cap server.");

	return test_l2cap_server.psm;
}

static void connected(struct bt_conn *conn, uint8_t conn_err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (conn_err) {
		FAIL("Failed to connect to %s (%u)", addr, conn_err);
		return;
	}

	default_conn = conn;

	LOG_DBG("%s", addr);

	SET_FLAG(is_connected);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	LOG_ERR("%p %s (reason 0x%02x)", conn, addr, reason);

	UNSET_FLAG(is_connected);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
};

static void disconnect_device(struct bt_conn *conn, void *data)
{
	int err;

	SET_FLAG(is_connected);

	err = bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
	ASSERT(!err, "Failed to initate disconnect (err %d)", err);

	LOG_DBG("Waiting for disconnection...");
	WAIT_FOR_FLAG_UNSET(is_connected);
}

static void test_dut_main(void)
{
	LOG_DBG("DUT deadlock started");
	int err;

	/* Prepare tx_data */
	for (size_t i = 0; i < sizeof(tx_data); i++) {
		tx_data[i] = (uint8_t)i;
	}

	err = bt_enable(NULL);
	if (err) {
		FAIL("Can't enable Bluetooth (err %d)", err);
		return;
	}

	(void)settings_load();

	k_work_queue_start(&dut_work_q, dut_work_stack,
			   K_THREAD_STACK_SIZEOF(dut_work_stack),
			   K_PRIO_COOP(CONFIG_BT_RX_PRIO + 1), NULL);
	k_thread_name_set(&dut_work_q.thread, "DUT workq");

	LOG_DBG("DUT Bluetooth initialized.");
	LOG_DBG("Connectable advertising...");
	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, NULL, 0, NULL, 0);
	if (err) {
		FAIL("Advertising failed to start (err %d)", err);
		return;
	}

	LOG_DBG("Advertising started.");
	LOG_DBG("DUT waiting for connection...");
	WAIT_FOR_FLAG_SET(is_connected);
	LOG_DBG("DUT is connected.");

	int psm = l2cap_server_register(BT_SECURITY_L1);

	LOG_DBG("Registered server PSM %x", psm);

	LOG_DBG("DUT waiting for L2CAP channels to connect...");
	for (size_t i = 0; i < CHANNELS; i++) {
		WAIT_FOR_FLAG_SET(flag_l2cap_connected);
		UNSET_FLAG(flag_l2cap_connected);
	}

	LOG_DBG("All L2CAP channels connected.");

	LOG_DBG("DUT waiting for the transfer completion");

	/* This should block host threads to process ACL bufs thus causing deadlock. */
	k_sleep(K_SECONDS(2));
	k_work_submit_to_queue(&dut_work_q, &heavy_work);

	do {
		k_msleep(100);
	} while (rx_cnt < SDU_NUM * CHANNELS);

	LOG_DBG("All packets received, disconnecting...");
	bt_conn_foreach(BT_CONN_TYPE_LE, disconnect_device, NULL);
	WAIT_FOR_FLAG_UNSET(is_connected);

	PASS("DUT deadlock test passed\n");
}

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	struct bt_le_conn_param *param;
	struct bt_conn *conn;
	int err;

	err = bt_le_scan_stop();
	if (err) {
		FAIL("Stop LE scan failed (err %d)", err);
		return;
	}

	char str[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(addr, str, sizeof(str));

	LOG_DBG("Connecting to %s", str);

	param = BT_LE_CONN_PARAM_DEFAULT;
	err = bt_conn_le_create(addr, BT_CONN_LE_CREATE_CONN, param, &conn);
	if (err) {
		FAIL("Create conn failed (err %d)", err);
		return;
	}
}

static void connect_dut(void)
{
	struct bt_le_scan_param scan_param = {
		.type = BT_LE_SCAN_TYPE_ACTIVE,
		.options = BT_LE_SCAN_OPT_NONE,
		.interval = BT_GAP_SCAN_FAST_INTERVAL,
		.window = BT_GAP_SCAN_FAST_WINDOW,
	};

	UNSET_FLAG(is_connected);

	int err = bt_le_scan_start(&scan_param, device_found);

	ASSERT(!err, "Scanning failed to start (err %d)\n", err);

	LOG_DBG("Central initiating connection...");
	WAIT_FOR_FLAG_SET(is_connected);
}

static void connect_l2cap_channel(struct bt_conn *conn, void *data)
{
	for (size_t i = 0; i < CHANNELS; i++) {
		struct test_ctx *ctx = alloc_test_context();
		int err;

		ASSERT(ctx, "No more available test test_ctx\n");

		struct bt_l2cap_le_chan *le_chan = &ctx->le_chan;

		le_chan->chan.ops = &ops;
		le_chan->rx.mtu = SDU_LEN;

		LOG_DBG("Connecting L2CAP channel[%d] %p", i, &le_chan->chan);

		err = bt_l2cap_chan_connect(conn, &le_chan->chan, 0x0080);
		ASSERT(!err, "Error connecting l2cap channel (err %d)\n", err);

		WAIT_FOR_FLAG_SET(flag_l2cap_connected);
		UNSET_FLAG(flag_l2cap_connected);

		LOG_DBG("Connected L2CAP channel[%d] %p", i, &le_chan->chan);
	}
}

static void test_tester_main(void)
{
	int err;

	LOG_DBG("Tester deadlock started");

	/* Prepare tx_data */
	for (size_t i = 0; i < sizeof(tx_data); i++) {
		tx_data[i] = (uint8_t)i;
	}

	err = bt_enable(NULL);
	ASSERT(err == 0, "Can't enable Bluetooth (err %d)\n", err);
	LOG_DBG("Central Bluetooth initialized.");

	(void)settings_load();

	/* Connect to dut */
	connect_dut();

	/* Connect L2CAP channels */
	LOG_DBG("Connect L2CAP channels");
	bt_conn_foreach(BT_CONN_TYPE_LE, connect_l2cap_channel, NULL);

	LOG_DBG("All L2CAP channels connected.");

	k_sleep(K_SECONDS(2));

	/* Send SDU_NUM SDUs over each channel */
	for (size_t i = 0; i < CHANNELS; i++) {
		test_ctx[i].tx_left = SDU_NUM;
		l2cap_chan_send(&test_ctx[i].le_chan.chan, tx_data, sizeof(tx_data));
	}

	LOG_DBG("Wait until all transfers are completed.");
	int remaining_tx = 0;
	do {
		remaining_tx = 0;
		k_msleep(100);

		for (size_t i = 0; i < CHANNELS; i++) {
			remaining_tx += test_ctx[i].tx_left;
		}
	} while (remaining_tx || (rx_cnt < SDU_NUM * CHANNELS));

	LOG_DBG("Waiting until dut is disconnected..");
	WAIT_FOR_FLAG_UNSET(is_connected);

	PASS("Tester deadlock passed\n");
}

static const struct bst_test_instance test_def[] = {
	{
		.test_id = "dut",
		.test_descr = "DUT acting as peripheral",
		.test_pre_init_f = test_init,
		.test_tick_f = test_tick,
		.test_main_f = test_dut_main
	},
	{
		.test_id = "tester",
		.test_descr = "Tester acting as central",
		.test_pre_init_f = test_init,
		.test_tick_f = test_tick,
		.test_main_f = test_tester_main
	},
	BSTEST_END_MARKER
};

struct bst_test_list *test_main_l2cap_deadlock_install(struct bst_test_list *tests)
{
	return bst_add_tests(tests, test_def);
}

bst_test_install_t test_installers[] = {
	test_main_l2cap_deadlock_install,
	NULL
};

int main(void)
{
	bst_main();
	return 0;
}
