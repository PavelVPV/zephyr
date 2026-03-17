/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

void bt_mesh_sol_reset(void);

/**
 * @brief Process a received Mesh Proxy Solicitation advertising structure.
 *
 * The @p buf parameter shall point to the start of a single complete
 * Advertising Data (AD) structure as encoded in an advertising packet:
 *   - 1 octet Length (AD structure length, excluding the length octet),
 *   - 1 octet AD Type,
 *   - AD data payload (Length - 1 octets).
 *
 * This function is intended to be called from the advertising report parser
 * when the AD Type is BT_DATA_UUID16_SOME/BT_DATA_UUID16_ALL (i.e. the AD
 * structure contains a 16-bit UUID list and any subsequent AD structures for
 * the same advertisement).
 *
 * @param buf Advertising data buffer containing the complete AD structure
 *            (Length + Type + payload).
 */
void bt_mesh_sol_recv(struct net_buf_simple *buf);

void bt_mesh_srpl_entry_clear(uint16_t addr);

void bt_mesh_srpl_pending_store(void);

void bt_mesh_sseq_pending_store(void);

int bt_mesh_sol_send(void);
