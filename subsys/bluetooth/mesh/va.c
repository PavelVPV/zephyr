#include <zephyr/kernel.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <zephyr/settings/settings.h>
#include <zephyr/net/buf.h>

#include <zephyr/bluetooth/mesh.h>

#include "common/bt_str.h"

#include "va.h"
#include "foundation.h"
#include "msg.h"
#include "net.h"
#include "crypto.h"
#include "settings.h"

#define LOG_LEVEL CONFIG_BT_MESH_TRANS_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_mesh_va);

static struct bt_mesh_va virtual_addrs[CONFIG_BT_MESH_LABEL_COUNT];

/* Virtual Address information for persistent storage. */
struct va_val {
	uint16_t ref;
	uint16_t addr;
	uint8_t uuid[16];
} __packed;

static void store_va_label(void)
{
	bt_mesh_settings_store_schedule(BT_MESH_SETTINGS_VA_PENDING);
}

static inline void va_store(struct bt_mesh_va *store)
{
	store->changed = 1U;
	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		store_va_label();
	}
}

uint8_t bt_mesh_va_add(const uint8_t uuid[16], const struct bt_mesh_va **entry)
{
	struct bt_mesh_va *va = NULL;
	int err;

	for (int i = 0; i < ARRAY_SIZE(virtual_addrs); i++) {
		if (!virtual_addrs[i].ref) {
			if (!va) {
				va = &virtual_addrs[i];
			}

			continue;
		}

		if (!memcmp(uuid, virtual_addrs[i].uuid,
			    ARRAY_SIZE(virtual_addrs[i].uuid))) {
			if (entry) {
				*entry = &virtual_addrs[i];
			}
			virtual_addrs[i].ref++;
			va_store(&virtual_addrs[i]);
			return STATUS_SUCCESS;
		}
	}

	if (!va) {
		return STATUS_INSUFF_RESOURCES;
	}

	memcpy(va->uuid, uuid, ARRAY_SIZE(va->uuid));
	err = bt_mesh_virtual_addr(uuid, &va->addr);
	if (err) {
		va->addr = BT_MESH_ADDR_UNASSIGNED;
		return STATUS_UNSPECIFIED;
	}

	va->ref = 1;
	va_store(va);

	if (entry) {
		*entry = va;
	}

	return STATUS_SUCCESS;
}

uint8_t bt_mesh_va_del(const uint8_t *uuid)
{
	struct bt_mesh_va *va;

	va = CONTAINER_OF(uuid, struct bt_mesh_va, uuid);

	if (!PART_OF_ARRAY(virtual_addrs, va) || va->ref == 0) {
		return STATUS_CANNOT_REMOVE;
	}

	va->ref--;
	va_store(va);

	return STATUS_SUCCESS;
}

const uint8_t *bt_mesh_va_uuid_get(uint16_t addr, const uint8_t *uuid, uint16_t *retaddr)
{
	int i;

	LOG_DBG("addr 0x%04x", addr);

	for (i = 0; i < ARRAY_SIZE(virtual_addrs); i++) {
		if (virtual_addrs[i].ref &&
		    (virtual_addrs[i].addr == addr || addr == BT_MESH_ADDR_UNASSIGNED)) {
			if (!uuid) {
				LOG_DBG("Found Label UUID for 0x%04x: %s", addr,
					bt_hex(virtual_addrs[i].uuid, 16));

				if (retaddr) {
					*retaddr = virtual_addrs[i].addr;
				}

				return virtual_addrs[i].uuid;
			} else if (uuid == virtual_addrs[i].uuid) {
				uuid = NULL;
			}
		}
	}

	LOG_WRN("No matching Label UUID for 0x%04x", addr);

	return NULL;
}

const struct bt_mesh_va *bt_mesh_va_get(const uint8_t *uuid)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(virtual_addrs); i++) {
		if (virtual_addrs[i].ref && !memcmp(virtual_addrs[i].uuid, uuid, 16)) {
			return &virtual_addrs[i];
		}
	}

	return NULL;
}

const uint8_t *bt_mesh_va_get_uuid_by_idx(uint16_t idx)
{
	if (idx >= ARRAY_SIZE(virtual_addrs)) {
		return NULL;
	}

	return virtual_addrs[idx].uuid;
}

uint16_t bt_mesh_va_get_idx_by_uuid(const uint8_t *uuid)
{
	struct bt_mesh_va *va = CONTAINER_OF(uuid, struct bt_mesh_va, uuid);

	if (!PART_OF_ARRAY(virtual_addrs, va) || va->ref == 0) {
		return (uint16_t)(-1);
	}

	return ARRAY_INDEX(virtual_addrs, va);
}

#if CONFIG_BT_MESH_LABEL_COUNT > 0
static struct bt_mesh_va *bt_mesh_va_get_by_index(uint16_t index)
{
	if (index >= ARRAY_SIZE(virtual_addrs)) {
		return NULL;
	}

	return &virtual_addrs[index];
}

static int va_set(const char *name, size_t len_rd,
		  settings_read_cb read_cb, void *cb_arg)
{
	struct va_val va;
	struct bt_mesh_va *lab;
	uint16_t index;
	int err;

	if (!name) {
		LOG_ERR("Insufficient number of arguments");
		return -ENOENT;
	}

	index = strtol(name, NULL, 16);

	if (len_rd == 0) {
		LOG_WRN("Mesh Virtual Address length = 0");
		return 0;
	}

	err = bt_mesh_settings_set(read_cb, cb_arg, &va, sizeof(va));
	if (err) {
		LOG_ERR("Failed to set \'virtual address\'");
		return err;
	}

	if (va.ref == 0) {
		LOG_WRN("Ignore Mesh Virtual Address ref = 0");
		return 0;
	}

	lab = bt_mesh_va_get_by_index(index);
	if (lab == NULL) {
		LOG_WRN("Out of labels buffers");
		return -ENOBUFS;
	}

	memcpy(lab->uuid, va.uuid, 16);
	lab->addr = va.addr;
	lab->ref = va.ref;

	LOG_DBG("Restored Virtual Address, addr 0x%04x ref 0x%04x", lab->addr, lab->ref);

	return 0;
}

BT_MESH_SETTINGS_DEFINE(va, "Va", va_set);

#define IS_VA_DEL(_label)	((_label)->ref == 0)
void bt_mesh_va_pending_store(void)
{
	struct bt_mesh_va *lab;
	struct va_val va;
	char path[18];
	uint16_t i;
	int err;

	for (i = 0; (lab = bt_mesh_va_get_by_index(i)) != NULL; i++) {
		if (!lab->changed) {
			continue;
		}

		lab->changed = 0U;

		snprintk(path, sizeof(path), "bt/mesh/Va/%x", i);

		if (IS_VA_DEL(lab)) {
			err = settings_delete(path);
		} else {
			va.ref = lab->ref;
			va.addr = lab->addr;
			memcpy(va.uuid, lab->uuid, 16);

			err = settings_save_one(path, &va, sizeof(va));
		}

		if (err) {
			LOG_ERR("Failed to %s %s value (err %d)",
				IS_VA_DEL(lab) ? "delete" : "store", path, err);
		} else {
			LOG_DBG("%s %s value", IS_VA_DEL(lab) ? "Deleted" : "Stored", path);
		}
	}
}
#else
void bt_mesh_va_pending_store(void)
{
	/* Do nothing. */
}
#endif /* CONFIG_BT_MESH_LABEL_COUNT > 0 */

void bt_mesh_va_clear(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(virtual_addrs); i++) {
		if (virtual_addrs[i].ref) {
			virtual_addrs[i].ref = 0U;
			virtual_addrs[i].changed = 1U;
		}
	}

	if (IS_ENABLED(CONFIG_BT_SETTINGS)) {
		store_va_label();
	}
}
