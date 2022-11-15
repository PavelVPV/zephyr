/*
 * Copyright (c) 2018 Nordic Semiconductor ASA
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/debug/stack.h>
#include <zephyr/sys/util.h>

#include <zephyr/net/buf.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/mesh.h>

#define BT_DBG_ENABLED IS_ENABLED(CONFIG_BT_MESH_DEBUG_ADV)
#define LOG_MODULE_NAME bt_mesh_adv
#include "common/log.h"

#include "adv.h"
#include "net.h"
#include "foundation.h"
#include "beacon.h"
#include "host/ecc.h"
#include "prov.h"
#include "proxy.h"
#include "pb_gatt_srv.h"

#include <soc.h>

static inline void pin_set(uint32_t pin)
{
	NRF_P0->OUTSET = 1 << pin;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
}

static inline void pin_clr(uint32_t pin)
{
	NRF_P0->OUTCLR = 1 << pin;
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
	__asm("NOP");
}

static inline void pin_toggle(uint32_t pin, uint32_t count)
{
	for (uint32_t i = 0; i < count; i++) {
		pin_set(pin);
		pin_clr(pin);
	}
}

/* Window and Interval are equal for continuous scanning */
#define MESH_SCAN_INTERVAL    BT_MESH_ADV_SCAN_UNIT(BT_MESH_SCAN_INTERVAL_MS)
#define MESH_SCAN_WINDOW      BT_MESH_ADV_SCAN_UNIT(BT_MESH_SCAN_WINDOW_MS)

const uint8_t bt_mesh_adv_type[BT_MESH_ADV_TYPES] = {
	[BT_MESH_ADV_PROV]   = BT_DATA_MESH_PROV,
	[BT_MESH_ADV_DATA]   = BT_DATA_MESH_MESSAGE,
	[BT_MESH_ADV_BEACON] = BT_DATA_MESH_BEACON,
	[BT_MESH_ADV_URI]    = BT_DATA_URI,
};

static K_FIFO_DEFINE(bt_mesh_adv_queue);
static K_FIFO_DEFINE(bt_mesh_relay_queue);
static K_FIFO_DEFINE(bt_mesh_friend_queue);

static void adv_buf_destroy(struct net_buf *buf)
{
	struct bt_mesh_adv adv = *BT_MESH_ADV(buf);

	net_buf_destroy(buf);

	bt_mesh_adv_send_end(0, &adv);
}

NET_BUF_POOL_DEFINE(adv_buf_pool, CONFIG_BT_MESH_ADV_BUF_COUNT,
		    BT_MESH_ADV_DATA_SIZE, BT_MESH_ADV_USER_DATA_SIZE,
		    adv_buf_destroy);

static struct bt_mesh_adv adv_local_pool[CONFIG_BT_MESH_ADV_BUF_COUNT];

#if defined(CONFIG_BT_MESH_RELAY)
NET_BUF_POOL_DEFINE(relay_buf_pool, CONFIG_BT_MESH_RELAY_BUF_COUNT,
		    BT_MESH_ADV_DATA_SIZE, BT_MESH_ADV_USER_DATA_SIZE,
		    adv_buf_destroy);

static struct bt_mesh_adv adv_relay_pool[CONFIG_BT_MESH_RELAY_BUF_COUNT];
#endif

#if defined(CONFIG_BT_MESH_ADV_EXT_FRIEND_SEPARATE)
NET_BUF_POOL_DEFINE(friend_buf_pool, CONFIG_BT_MESH_FRIEND_LPN_COUNT,
		    BT_MESH_ADV_DATA_SIZE, BT_MESH_ADV_USER_DATA_SIZE,
		    adv_buf_destroy);

static struct bt_mesh_adv adv_friend_pool[CONFIG_BT_MESH_FRIEND_LPN_COUNT];
#endif

static struct net_buf *bt_mesh_adv_create_from_pool(struct net_buf_pool *buf_pool,
						    struct bt_mesh_adv *adv_pool,
						    enum bt_mesh_adv_type type,
						    enum bt_mesh_adv_tag tag,
						    uint8_t xmit, k_timeout_t timeout)
{
	struct bt_mesh_adv *adv;
	struct net_buf *buf;

	if (atomic_test_bit(bt_mesh.flags, BT_MESH_SUSPENDED)) {
		BT_WARN("Refusing to allocate buffer while suspended");
		return NULL;
	}

	buf = net_buf_alloc(buf_pool, timeout);
	if (!buf) {
		return NULL;
	}

	adv = &adv_pool[net_buf_id(buf)];
	BT_MESH_ADV(buf) = adv;

	(void)memset(adv, 0, sizeof(*adv));

	adv->type         = type;
	adv->tag          = tag;
	adv->xmit         = xmit;

	return buf;
}

struct net_buf *bt_mesh_adv_create(enum bt_mesh_adv_type type,
				   enum bt_mesh_adv_tag tag,
				   uint8_t xmit, k_timeout_t timeout)
{
#if defined(CONFIG_BT_MESH_RELAY)
	if (tag & BT_MESH_RELAY_ADV) {
		return bt_mesh_adv_create_from_pool(&relay_buf_pool,
						    adv_relay_pool, type,
						    tag, xmit, timeout);
	}
#endif

#if defined(CONFIG_BT_MESH_ADV_EXT_FRIEND_SEPARATE)
	if (tag & BT_MESH_FRIEND_ADV) {
		return bt_mesh_adv_create_from_pool(&friend_buf_pool,
						    adv_friend_pool, type,
						    tag, xmit, timeout);
	}
#endif

	return bt_mesh_adv_create_from_pool(&adv_buf_pool, adv_local_pool, type,
					    tag, xmit, timeout);
}

#if CONFIG_BT_MESH_RELAY_ADV_SETS || CONFIG_BT_MESH_ADV_EXT_FRIEND_SEPARATE
static struct net_buf *process_events(struct k_poll_event *ev, int count)
{
	for (; count; ev++, count--) {
		BT_DBG("ev->state %u", ev->state);

		switch (ev->state) {
		case K_POLL_STATE_FIFO_DATA_AVAILABLE:
			return net_buf_get(ev->fifo, K_NO_WAIT);
		case K_POLL_STATE_NOT_READY:
		case K_POLL_STATE_CANCELLED:
			break;
		default:
			BT_WARN("Unexpected k_poll event state %u", ev->state);
			break;
		}
	}

	return NULL;
}

struct net_buf *bt_mesh_adv_buf_get(k_timeout_t timeout)
{
	int err;
	struct k_poll_event events[] = {
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&bt_mesh_adv_queue,
						0),
#if defined(CONFIG_BT_MESH_ADV_EXT_FRIEND_SEPARATE)
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&bt_mesh_friend_queue,
						0),
#endif
#if defined(CONFIG_BT_MESH_RELAY_ADV_SETS)
		K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_FIFO_DATA_AVAILABLE,
						K_POLL_MODE_NOTIFY_ONLY,
						&bt_mesh_relay_queue,
						0),
#endif

	};

	err = k_poll(events, ARRAY_SIZE(events), timeout);
	if (err) {
		return NULL;
	}

	return process_events(events, ARRAY_SIZE(events));
}

struct net_buf *bt_mesh_adv_buf_get_by_tag(uint8_t tag, k_timeout_t timeout)
{
	if (tag & BT_MESH_LOCAL_ADV
#if !defined(CONFIG_BT_MESH_ADV_EXT_FRIEND_SEPARATE)
	    || tag & BT_MESH_FRIEND_ADV
#endif
#if !defined(CONFIG_BT_MESH_RELAY_ADV_SETS)
	    || !(tag & BT_MESH_FRIEND_ADV) /* Any other tag except Friend */
#endif
	    ) {
		return bt_mesh_adv_buf_get(timeout);
#if defined(CONFIG_BT_MESH_ADV_EXT_FRIEND_SEPARATE)
	} else if (tag & BT_MESH_FRIEND_ADV) {
		return net_buf_get(&bt_mesh_friend_queue, timeout);
#endif
#if !defined(CONFIG_BT_MESH_RELAY_ADV_SETS)
	} else if (tag & BT_MESH_RELAY_ADV) {
		return net_buf_get(&bt_mesh_relay_queue, timeout);
#endif
	} else {
		return NULL;
	}
}
#else /* !CONFIG_BT_MESH_RELAY_ADV_SETS */
struct net_buf *bt_mesh_adv_buf_get(k_timeout_t timeout)
{
	return net_buf_get(&bt_mesh_adv_queue, timeout);
}

struct net_buf *bt_mesh_adv_buf_get_by_tag(uint8_t tag, k_timeout_t timeout)
{
	ARG_UNUSED(tag);

	return bt_mesh_adv_buf_get(timeout);
}
#endif /* CONFIG_BT_MESH_RELAY_ADV_SETS */

void bt_mesh_adv_buf_get_cancel(void)
{
	BT_DBG("");

	k_fifo_cancel_wait(&bt_mesh_adv_queue);

#if CONFIG_BT_MESH_RELAY_ADV_SETS
	k_fifo_cancel_wait(&bt_mesh_relay_queue);
#endif /* CONFIG_BT_MESH_RELAY_ADV_SETS */

#if CONFIG_BT_MESH_ADV_EXT_FRIEND_SEPARATE
	k_fifo_cancel_wait(&bt_mesh_friend_queue);
#endif
}

void bt_mesh_adv_send(struct net_buf *buf, const struct bt_mesh_send_cb *cb,
		      void *cb_data)
{
	BT_DBG("type 0x%02x len %u: %s", BT_MESH_ADV(buf)->type, buf->len,
	       bt_hex(buf->data, buf->len));

	BT_MESH_ADV(buf)->cb = cb;
	BT_MESH_ADV(buf)->cb_data = cb_data;
	BT_MESH_ADV(buf)->busy = 1U;

	if (BT_MESH_ADV(buf)->tag == BT_MESH_LOCAL_ADV) {
		net_buf_put(&bt_mesh_adv_queue, net_buf_ref(buf));
		bt_mesh_adv_buf_local_ready();
	} else if (BT_MESH_ADV(buf)->tag == BT_MESH_FRIEND_ADV) {
#if defined(CONFIG_BT_MESH_ADV_EXT_FRIEND_SEPARATE)
		net_buf_put(&bt_mesh_friend_queue, net_buf_ref(buf));
		bt_mesh_adv_buf_friend_ready();
#else
		net_buf_put(&bt_mesh_adv_queue, net_buf_ref(buf));
		bt_mesh_adv_buf_local_ready();
#endif
	} else {
#if CONFIG_BT_MESH_RELAY_ADV_SETS
		net_buf_put(&bt_mesh_relay_queue, net_buf_ref(buf));
		bt_mesh_adv_buf_relay_ready();
#else
		net_buf_put(&bt_mesh_adv_queue, net_buf_ref(buf));
		bt_mesh_adv_buf_local_ready();
#endif
	}
}

int bt_mesh_adv_gatt_send(void)
{
	if (bt_mesh_is_provisioned()) {
		if (IS_ENABLED(CONFIG_BT_MESH_GATT_PROXY)) {
			BT_DBG("Proxy Advertising");
			return bt_mesh_proxy_adv_start();
		}
	} else if (IS_ENABLED(CONFIG_BT_MESH_PB_GATT)) {
		BT_DBG("PB-GATT Advertising");
		return bt_mesh_pb_gatt_srv_adv_start();
	}

	return -ENOTSUP;
}

#if defined(CONFIG_BT_MESH_LOW_POWER)
#include <nrfx_timer.h>
#include <hal/nrf_radio.h>
#include <nrfx_ppi.h>
#include <nrfx_egu.h>
static nrf_ppi_channel_t radio_end_capture_ppi_ch = 0xFF;
static nrf_ppi_channel_t radio_ready_clear_ppi_ch = 0xFF;
static nrf_ppi_channel_t radio_ready_egu_ppi_ch = 0xFF;
static const nrfx_egu_t egu_ready = NRFX_EGU_INSTANCE(2);

static const nrfx_timer_t rx_timer = NRFX_TIMER_INSTANCE(2);
static const nrfx_timer_config_t timer_cfg = {
	.frequency = NRF_TIMER_FREQ_16MHz,
	.mode = NRF_TIMER_MODE_TIMER,
	.bit_width = NRF_TIMER_BIT_WIDTH_32,
	.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY,
	.p_context = NULL
};

static void timer_event_handler(nrf_timer_event_t event_type, void *p_context)
{
	(void)event_type;
	(void)p_context;
};

static void egu_ready_event_handler(uint8_t event_idx, void * p_context)
{
	nrfx_err_t err;

	err = nrfx_ppi_channel_disable(radio_ready_clear_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppi_channel_disable(radio_ready_egu_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	nrfx_egu_int_disable(&egu_ready, NRF_EGU_INT_TRIGGERED0);
}

static void timer_capture_setup(void)
{
	nrfx_err_t err;

	err = nrfx_timer_init(&rx_timer, &timer_cfg, timer_event_handler);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppi_channel_alloc(&radio_end_capture_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
	err = nrfx_ppi_channel_enable(radio_end_capture_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppi_channel_assign(
		radio_end_capture_ppi_ch,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS),
		nrfx_timer_task_address_get(&rx_timer, NRF_TIMER_TASK_CAPTURE0));
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

#if 0
	/* RXREADY -> TIMER_CLEAR -> EGU */
	IRQ_CONNECT(SWI2_EGU2_IRQn, IRQ_PRIO_LOWEST, nrfx_egu_2_irq_handler, NULL, 0);

	err = nrfx_egu_init(&egu_ready, NRFX_EGU_DEFAULT_CONFIG_IRQ_PRIORITY,
			    egu_ready_event_handler, NULL);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppi_channel_alloc(&radio_ready_clear_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
	err = nrfx_ppi_channel_alloc(&radio_ready_egu_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppi_channel_assign(
		radio_ready_clear_ppi_ch,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_RXREADY),
		nrfx_timer_task_address_get(&rx_timer, NRF_TIMER_TASK_CLEAR));
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppi_channel_assign(
		radio_ready_egu_ppi_ch,
		nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_RXREADY),
		nrf_egu_task_address_get(egu_ready.p_reg, NRF_EGU_TASK_TRIGGER0));
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	nrfx_egu_int_enable(&egu_ready, NRF_EGU_INT_TRIGGERED0);
#endif

	nrfx_timer_enable(&rx_timer);
}

static void timer_capture_clear(void)
{
	nrfx_err_t err;

#if 0
	err = nrfx_ppi_channel_enable(radio_ready_clear_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppi_channel_enable(radio_ready_egu_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
#else
	nrfx_timer_clear(&rx_timer);
#endif
}

#if 0
static void timer_capture_stop(void)
{
	nrfx_err_t err;

	err = nrfx_ppi_channel_disable(radio_end_capture_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);

	err = nrfx_ppi_channel_free(radio_end_capture_ppi_ch);
	__ASSERT_NO_MSG(err == NRFX_SUCCESS);
}
#endif

static struct {
	uint32_t min;
	uint32_t max;
	uint32_t avg;
} rx_time_delta;

static void timer_capture_print(void)
{
	static int skip_first = 2;
	uint32_t delta;

	if (skip_first) {
		skip_first--;
		return;
	}

	delta = nrfx_timer_capture_get(&rx_timer, NRF_TIMER_CC_CHANNEL0);

	if (delta < rx_time_delta.min || rx_time_delta.min == 0) {
		rx_time_delta.min = delta;
	}

	if (delta > rx_time_delta.max) {
		rx_time_delta.max = delta;
	}

	if (rx_time_delta.avg != 0) {
		rx_time_delta.avg += delta;
		rx_time_delta.avg /= 2;
	} else {
		rx_time_delta.avg = delta;
	}

	BT_ERR(">>>>>> rx_time_delta: cur %u min %u max %u avg %u ms <<<<<<",
	       ceiling_fraction(delta, 16000),
	       ceiling_fraction(rx_time_delta.min, 16000),
	       ceiling_fraction(rx_time_delta.max, 16000),
	       ceiling_fraction(rx_time_delta.avg, 16000));
}
#endif

void bt_mesh_scan_init(void)
{
#if defined(CONFIG_BT_MESH_LOW_POWER)
	timer_capture_setup();
#endif
}

static void bt_mesh_scan_cb(const bt_addr_le_t *addr, int8_t rssi,
			    uint8_t adv_type, struct net_buf_simple *buf)
{
	if (adv_type != BT_GAP_ADV_TYPE_ADV_NONCONN_IND) {
		return;
	}

	BT_DBG("len %u: %s", buf->len, bt_hex(buf->data, buf->len));

	while (buf->len > 1) {
		struct net_buf_simple_state state;
		uint8_t len, type;

		len = net_buf_simple_pull_u8(buf);
		/* Check for early termination */
		if (len == 0U) {
			return;
		}

		if (len > buf->len) {
			BT_WARN("AD malformed");
			return;
		}

		net_buf_simple_save(buf, &state);

		type = net_buf_simple_pull_u8(buf);

		buf->len = len - 1;

		switch (type) {
		case BT_DATA_MESH_MESSAGE:
#if defined(CONFIG_BT_MESH_LOW_POWER)
			timer_capture_print();
#endif
			bt_mesh_net_recv(buf, rssi, BT_MESH_NET_IF_ADV);
			break;
#if defined(CONFIG_BT_MESH_PB_ADV)
		case BT_DATA_MESH_PROV:
			bt_mesh_pb_adv_recv(buf);
			break;
#endif
		case BT_DATA_MESH_BEACON:
			bt_mesh_beacon_recv(buf);
			break;
		default:
			break;
		}

		net_buf_simple_restore(buf, &state);
		net_buf_simple_pull(buf, len);
	}
}

int bt_mesh_scan_enable(void)
{
	struct bt_le_scan_param scan_param = {
			.type       = BT_HCI_LE_SCAN_PASSIVE,
			.options    = BT_LE_SCAN_OPT_NONE,
			.interval   = MESH_SCAN_INTERVAL,
			.window     = MESH_SCAN_WINDOW };
	int err;

	BT_DBG("");

#if defined(CONFIG_BT_MESH_LOW_POWER)
	timer_capture_clear();
#endif

	err = bt_le_scan_start(&scan_param, bt_mesh_scan_cb);
	if (err && err != -EALREADY) {
		BT_ERR("starting scan failed (err %d)", err);
		return err;
	}

#if defined(CONFIG_BT_MESH_LOW_POWER)
	pin_set(4);
#endif

	return 0;
}

int bt_mesh_scan_disable(void)
{
	int err;

	BT_DBG("");

#if defined(CONFIG_BT_MESH_LOW_POWER)
	pin_clr(4);
#endif

	err = bt_le_scan_stop();
	if (err && err != -EALREADY) {
		BT_ERR("stopping scan failed (err %d)", err);
		return err;
	}

	return 0;
}
