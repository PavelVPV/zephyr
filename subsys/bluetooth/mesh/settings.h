/*
 * Copyright (c) 2018 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Pending storage actions. */
enum bt_mesh_settings_flag {
	BT_MESH_SETTINGS_RPL_PENDING,
	BT_MESH_SETTINGS_NET_KEYS_PENDING,
	BT_MESH_SETTINGS_APP_KEYS_PENDING,
	BT_MESH_SETTINGS_NET_PENDING,
	BT_MESH_SETTINGS_IV_PENDING,
	BT_MESH_SETTINGS_SEQ_PENDING,
	BT_MESH_SETTINGS_HB_PUB_PENDING,
	BT_MESH_SETTINGS_CFG_PENDING,
	BT_MESH_SETTINGS_MOD_PENDING,
	BT_MESH_SETTINGS_VA_PENDING,
	BT_MESH_SETTINGS_CDB_PENDING,

	BT_MESH_SETTINGS_FLAG_COUNT,
};

struct bt_mesh_settings_handler {
	const char *name;
	int (*h_commit)(void);
	void (*h_pending)(void);
	/* Private */
	int flag;
};

#ifdef CONFIG_BT_SETTINGS
#define MESH_SETTINGS_STATIC_HANDLER_DEFINE(_hname, _tree, _get, _set,	\
					    _commit, _export,		\
					    _pending)			\
	const Z_STRUCT_SECTION_ITERABLE(bt_mesh_settings_handler,	     \
					bt_mesh_settings_handler_ ## _hname) = {     \
		.name = _tree,						     \
		.h_commit = _commit,					     \
		.h_pending = _pending,					     \
		.flag = -1,						     \
	}; \
	SETTINGS_STATIC_HANDLER_DEFINE(_hname, _tree, _get, _set, NULL, _export);

#else
#define MESH_SETTINGS_STATIC_HANDLER_DEFINE(_hname, _tree, _get, _set,	\
					    _commit, _export,		\
					    _pending)			\
	const struct bt_mesh_settings_handler				\
				bt_mesh_settings_handler_ ## _hname = {	\
		.h_commit = _commit,					\
		.h_pending = _pending,					\
	};								\
	const struct settings_handler settings_handler_ ## _hname = {	\
		.h_set = _set,						\
		.h_get = _get,						\
		.h_commit = _commit,					\
		.h_export = _export,					\
	};

#define CONFIG_BT_MESH_STORE_TIMEOUT 0
#define CONFIG_BT_MESH_RPL_STORE_TIMEOUT 0
#endif /* CONFIG_BT_SETTINGS */

void bt_mesh_settings_init(void);
//void bt_mesh_settings_store_schedule(enum bt_mesh_settings_flag flag);
void bt_mesh_settings_store_schedule(const char *tree, int32_t timeout_s);
int bt_mesh_settings_set(settings_read_cb read_cb, void *cb_arg,
			 void *out, size_t read_len);

