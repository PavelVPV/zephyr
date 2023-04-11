/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <stdio.h>
#include <stdint.h>
//#include <zephyr/ztest.h>
#include <zephyr/bluetooth/mesh.h>
//#include "../../../../../subsys/bluetooth/hci/hci_core.h"
#include <conn.h>
#include <host/hci_core.h>
#include <net.h>

int bt_encrypt_be(const uint8_t key[16], const uint8_t plaintext[16],
		  uint8_t enc_data[16])
{ return 0; }

int bt_encrypt_le(const uint8_t key[16], const uint8_t plaintext[16],
		  uint8_t enc_data[16])
{ return 0; }

int bt_ccm_encrypt(const uint8_t key[16], uint8_t nonce[13],
		   const uint8_t *plaintext, size_t len, const uint8_t *aad,
		   size_t aad_len, uint8_t *enc_data, size_t mic_size)
{ return 0; }

int bt_ccm_decrypt(const uint8_t key[16], uint8_t nonce[13],
		   const uint8_t *enc_data, size_t len, const uint8_t *aad,
		   size_t aad_len, uint8_t *plaintext, size_t mic_size)
{ return 0; }

int bt_mesh_adv_enable(void)
{ return 0; }

void bt_mesh_adv_init(void)
{}

int settings_save_one(const char *name, const void *value, size_t val_len)
{ return 0; }

int settings_delete(const char *name)
{ return 0; }

void bt_mesh_adv_buf_local_ready(void)
{}

int bt_le_scan_stop(void)
{ return 0; }

int bt_le_scan_start(const struct bt_le_scan_param *param, bt_le_scan_cb_t cb)
{ return 0; }

const char *bt_hex(const void *buf, size_t len)
{ return ""; }

int settings_name_next(const char *name, const char **next)
{ return 0; }

struct bt_dev bt_dev;

#if 0
static void setup(void *f)
{
	IRQ_CONNECT(CONFIG_ARCH_POSIX_FUZZ_IRQ, 0, fuzz_isr, NULL, 0);
	irq_enable(CONFIG_ARCH_POSIX_FUZZ_IRQ);

	printf("setup\n");
}

static void teardown(void *f)
{
	printf("teardown\n");
}

ZTEST(mesh_test_suite, test_stub)
{
	printf("test_stub\n");
}

ZTEST_SUITE(mesh_test_suite, NULL, NULL, setup, teardown, NULL);
#endif

extern uint8_t *posix_fuzz_buf, posix_fuzz_sz;

static void fuzz_work_handler(struct k_work *work)
{
	struct net_buf_simple buf;

	net_buf_simple_init_with_data(&buf, posix_fuzz_buf, posix_fuzz_sz);
	bt_mesh_scan_cb(NULL, 0, BT_GAP_ADV_TYPE_ADV_NONCONN_IND, &buf);
}

static struct k_work fuzz_work = Z_WORK_INITIALIZER(fuzz_work_handler);
K_SEM_DEFINE(fuzz_sem, 0, K_SEM_MAX_LIMIT);

static void fuzz_isr(const void *arg)
{
//	k_work_submit(&fuzz_work);

	k_sem_give(&fuzz_sem);
}

void main(void)
{
	int err;

	printk("Initializing...\n");

	atomic_set_bit(bt_mesh.flags, BT_MESH_VALID);

	IRQ_CONNECT(CONFIG_ARCH_POSIX_FUZZ_IRQ, 0, fuzz_isr, NULL, 0);
	irq_enable(CONFIG_ARCH_POSIX_FUZZ_IRQ);

	while (true) {
#if 1
		k_sem_take(&fuzz_sem, K_FOREVER);

		/* Execute the fuzz case we got from LLVM and passed
		 * through an interrupt to this thread.
		 */
		fuzz_work_handler(NULL);
#endif
	}
}
