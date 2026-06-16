/* main.c - Application main entry point */

/*
 * Copyright (c) 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define NET_LOG_LEVEL CONFIG_NET_L2_ETHERNET_LOG_LEVEL

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_test, NET_LOG_LEVEL);

#include <zephyr/types.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <zephyr/sys/printk.h>
#include <zephyr/linker/sections.h>
#include <zephyr/random/random.h>

#include <zephyr/ztest.h>

#include <zephyr/net/net_if.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/wifi_nm.h>
#include <zephyr/sys/atomic.h>

#if NET_LOG_LEVEL >= LOG_LEVEL_DBG
#define DBG(fmt, ...) printk(fmt, ##__VA_ARGS__)
#else
#define DBG(fmt, ...)
#endif

struct wifi_drv_context {
	struct net_if *iface;
	uint8_t mac_addr[6];
	enum ethernet_if_types eth_if_type;
};

static struct wifi_drv_context wifi_context;

bool wifi_nm_op_called;
bool wifi_offload_op_called;

static void wifi_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct wifi_drv_context *context = dev->data;

	net_if_set_link_addr(iface, context->mac_addr,
			     sizeof(context->mac_addr),
			     NET_LINK_ETHERNET);

	net_eth_set_if_type_wifi(iface);

	ethernet_init(iface);
}

static int wifi_scan(const struct device *dev, struct net_if *iface,
		     struct wifi_scan_params *params, scan_result_cb_t cb)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);
	ARG_UNUSED(params);
	ARG_UNUSED(cb);

	wifi_offload_op_called = true;

	return 0;
}

/* State recorded by the connect/disconnect mocks below, used to verify the
 * CONFIG_WIFI_MGMT_DEFER_OPS hand-off.
 */
static k_tid_t connect_ran_on;
static k_tid_t disconnect_ran_on;
static atomic_t connect_calls;
static atomic_t disconnect_calls;
static int connect_rc;
static int disconnect_rc;
static uint8_t connect_ssid_len;
static bool reenter_disconnect;

static int wifi_connect_mock(const struct device *dev, struct net_if *iface,
			     struct wifi_connect_req_params *params)
{
	ARG_UNUSED(dev);

	connect_ran_on = k_current_get();
	connect_ssid_len = params->ssid_length;
	atomic_inc(&connect_calls);

	if (reenter_disconnect) {
		/* Issue a nested request from within the op. When deferral is
		 * enabled this runs on the workqueue thread, so the re-entrancy
		 * guard must execute it inline instead of self-deadlocking.
		 */
		reenter_disconnect = false;
		(void)net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
	}

	return connect_rc;
}

static int wifi_disconnect_mock(const struct device *dev, struct net_if *iface)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);

	disconnect_ran_on = k_current_get();
	atomic_inc(&disconnect_calls);

	return disconnect_rc;
}

static struct wifi_mgmt_ops wifi_mgmt_api = {
	.scan		= wifi_scan,
	.connect	= wifi_connect_mock,
	.disconnect	= wifi_disconnect_mock,
};

static struct net_wifi_mgmt_offload api_funcs = {
	.wifi_iface.iface_api.init = wifi_iface_init,
	.wifi_mgmt_api = &wifi_mgmt_api,
};

static void generate_mac(uint8_t *mac_addr)
{
	/* 00-00-5E-00-53-xx Documentation RFC 7042 */
	mac_addr[0] = 0x00;
	mac_addr[1] = 0x00;
	mac_addr[2] = 0x5E;
	mac_addr[3] = 0x00;
	mac_addr[4] = 0x53;
	mac_addr[5] = sys_rand8_get();
}

static int wifi_init(const struct device *dev)
{
	struct wifi_drv_context *context = dev->data;

	context->eth_if_type = L2_ETH_IF_TYPE_WIFI;

	generate_mac(context->mac_addr);

	return 0;
}

ETH_NET_DEVICE_INIT(wlan0, "wifi_test",
		    wifi_init, NULL,
		    &wifi_context, NULL, CONFIG_ETH_INIT_PRIORITY,
		    &api_funcs, NET_ETH_MTU);

static int wifi_nm_scan(const struct device *dev, struct net_if *iface,
			struct wifi_scan_params *params, scan_result_cb_t cb)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(iface);
	ARG_UNUSED(params);
	ARG_UNUSED(cb);

	wifi_nm_op_called = true;

	return 0;
}

static struct wifi_mgmt_ops wifi_nm_test_ops = {
	.scan		= wifi_nm_scan,
};

DEFINE_WIFI_NM_INSTANCE(test, &wifi_nm_test_ops);

static int request_scan(void)
{
	struct net_if *iface = net_if_get_first_wifi();

	if (net_mgmt(NET_REQUEST_WIFI_SCAN, iface, NULL, 0)) {
		printk("Scan request failed\n");

		return -ENOEXEC;
	}

	return 0;
}

ZTEST(net_wifi, test_wifi_offload)
{

	int ret;
#ifdef CONFIG_WIFI_NM
	struct wifi_nm_instance *nm = wifi_nm_get_instance("test");

	if (wifi_nm_get_instance_iface(net_if_get_first_wifi())) {
		ret = wifi_nm_unregister_mgd_iface(nm, net_if_get_first_wifi());
		zassert_equal(ret, 0, "Failed to unregister managed interface");
	}
#endif /* CONFIG_WIFI_NM */

	ret = request_scan();
	zassert_equal(ret, 0, "Scan request failed");
	zassert_true(wifi_offload_op_called, "Scan callback not called");
}

ZTEST(net_wifi, test_wifi_nm_managed)
{

	int ret;
	struct wifi_nm_instance *nm = wifi_nm_get_instance("test");

	zassert_equal(nm->ops, &wifi_nm_test_ops,
		      "Invalid wifi nm ops");

	/* Offload: in presence of registered NM but with no managed
	 *          interfaces.
	 */
	ret = request_scan();
	zassert_equal(ret, 0, "Scan request failed");
	zassert_true(wifi_offload_op_called, "Scan callback not called");

	ret = wifi_nm_register_mgd_iface(nm, net_if_get_first_wifi());
	zassert_equal(ret, 0, "Failed to register managed interface");

	zassert_equal(nm->ops, &wifi_nm_test_ops,
		      "Invalid wifi nm ops");

	ret = request_scan();
	zassert_equal(ret, 0, "Scan request failed");
	zassert_true(wifi_nm_op_called, "Scan callback not called");
}


/* Put the Wi-Fi iface into the offload path (no managed NM iface) and admin-up,
 * so NET_REQUEST_WIFI_CONNECT/DISCONNECT reach the device's mock ops.
 */
static struct net_if *prepare_wifi_iface(void)
{
	struct net_if *iface = net_if_get_first_wifi();

#ifdef CONFIG_WIFI_NM
	struct wifi_nm_instance *nm = wifi_nm_get_instance("test");

	if (wifi_nm_get_instance_iface(iface)) {
		(void)wifi_nm_unregister_mgd_iface(nm, iface);
	}
#endif /* CONFIG_WIFI_NM */

	if (!net_if_is_admin_up(iface)) {
		zassert_equal(net_if_up(iface), 0, "Failed to bring Wi-Fi iface up");
	}

	return iface;
}

static struct wifi_connect_req_params valid_connect_params(void)
{
	struct wifi_connect_req_params params = {
		.ssid = (const uint8_t *)"zephyr",
		.ssid_length = 6,
		.security = WIFI_SECURITY_TYPE_NONE,
		.channel = WIFI_CHANNEL_ANY,
	};

	return params;
}

/* The connect op is invoked, its parameters survive the hand-off, and (when
 * deferral is enabled) it runs on the Wi-Fi management workqueue rather than
 * the calling thread.
 */
ZTEST(net_wifi, test_wifi_connect_deferred)
{
	struct net_if *iface = prepare_wifi_iface();
	struct wifi_connect_req_params params = valid_connect_params();
	int ret;

	atomic_clear(&connect_calls);
	connect_ran_on = NULL;
	connect_ssid_len = 0;
	connect_rc = 0;

	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	zassert_equal(ret, 0, "connect should return the op result (got %d)", ret);
	zassert_equal(atomic_get(&connect_calls), 1, "connect op should be called once");
	zassert_equal(connect_ssid_len, params.ssid_length,
		      "connect params should survive the hand-off");
#if defined(CONFIG_WIFI_MGMT_DEFER_OPS)
	zassert_not_equal(connect_ran_on, k_current_get(),
			  "deferred connect should run on the Wi-Fi management workqueue");
#endif
}

/* The op return code is propagated back through net_mgmt(). */
ZTEST(net_wifi, test_wifi_connect_result_propagation)
{
	struct net_if *iface = prepare_wifi_iface();
	struct wifi_connect_req_params params = valid_connect_params();
	int ret;

	atomic_clear(&connect_calls);
	connect_rc = -EIO;

	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	zassert_equal(ret, -EIO, "connect should propagate the op error (got %d)", ret);
	zassert_equal(atomic_get(&connect_calls), 1, "connect op should be called once");
}

/* Argument validation stays synchronous in the caller's context: an invalid
 * request is rejected without ever reaching (or deferring to) the op.
 */
ZTEST(net_wifi, test_wifi_connect_invalid_params_sync)
{
	struct net_if *iface = prepare_wifi_iface();
	struct wifi_connect_req_params params = valid_connect_params();
	int ret;

	atomic_clear(&connect_calls);
	connect_rc = 0;
	params.ssid_length = 0; /* invalid */

	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	zassert_equal(ret, -EINVAL, "invalid params should be rejected (got %d)", ret);
	zassert_equal(atomic_get(&connect_calls), 0,
		      "op must not be reached for invalid params");
}

/* The disconnect op is invoked and (when deferral is enabled) runs on the
 * Wi-Fi management workqueue.
 */
ZTEST(net_wifi, test_wifi_disconnect_deferred)
{
	struct net_if *iface = prepare_wifi_iface();
	int ret;

	atomic_clear(&disconnect_calls);
	disconnect_ran_on = NULL;
	disconnect_rc = 0;

	ret = net_mgmt(NET_REQUEST_WIFI_DISCONNECT, iface, NULL, 0);
	zassert_equal(ret, 0, "disconnect should return the op result (got %d)", ret);
	zassert_equal(atomic_get(&disconnect_calls), 1, "disconnect op should be called once");
#if defined(CONFIG_WIFI_MGMT_DEFER_OPS)
	zassert_not_equal(disconnect_ran_on, k_current_get(),
			  "deferred disconnect should run on the Wi-Fi management workqueue");
#endif
}

/* A nested request issued from within an op (i.e. from the workqueue thread
 * when deferral is enabled) is handled inline by the re-entrancy guard and does
 * not dead-lock the single-threaded workqueue.
 */
ZTEST(net_wifi, test_wifi_connect_reentrant_disconnect)
{
	struct net_if *iface = prepare_wifi_iface();
	struct wifi_connect_req_params params = valid_connect_params();
	int ret;

	atomic_clear(&connect_calls);
	atomic_clear(&disconnect_calls);
	connect_rc = 0;
	disconnect_rc = 0;
	reenter_disconnect = true;

	ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, iface, &params, sizeof(params));
	zassert_equal(ret, 0, "outer connect should complete (got %d)", ret);
	zassert_equal(atomic_get(&connect_calls), 1, "connect op should be called once");
	zassert_equal(atomic_get(&disconnect_calls), 1,
		      "nested disconnect should have run inline");
}

ZTEST_SUITE(net_wifi, NULL, NULL, NULL, NULL, NULL);
