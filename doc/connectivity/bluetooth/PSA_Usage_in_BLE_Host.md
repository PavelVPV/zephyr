# PSA (Platform Security Architecture) Usage in Zephyr BLE Host

## Table of Contents
1. [Introduction and BLE Core Specification Context](#introduction-and-ble-core-specification-context)
2. [PSA in BLE Security Operations](#psa-in-ble-security-operations)
3. [Step-by-Step: How BLE Host Uses PSA](#step-by-step-how-ble-host-uses-psa)
4. [Key Lifecycle and Usage](#key-lifecycle-and-usage)
5. [Dependency on Secure Storage](#dependency-on-secure-storage)
6. [Kconfig Dependencies](#kconfig-dependencies)
7. [Code References](#code-references)

---

## Introduction and BLE Core Specification Context

### BLE Core Specification and Security Requirements

The Bluetooth Core Specification defines security mechanisms for protecting BLE communications. Key security features mandated by the specification include:

#### **1. LE Legacy Pairing (Bluetooth Core Specification Vol 3, Part H, Section 2.3)**
- Uses AES-128 encryption for key generation
- Requires cryptographic toolbox functions:
  - **AES-128** encryption (e function) - [crypto_psa.c#L66-L116](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L66-L116)
  - Random number generation for challenges

#### **2. LE Secure Connections (LESC) (Bluetooth Core Specification Vol 3, Part H, Section 2.3.5)**
LESC is the more secure pairing method introduced in Bluetooth 4.2, requiring:
- **ECDH (Elliptic Curve Diffie-Hellman)** key agreement using NIST P-256 (secp256r1) curve
  - Public key generation (P-256 key pairs) - [ecc.c#L128-L195](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L128-L195)
  - ECDH shared secret computation - [ecc.c#L197-L261](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L197-L261)
- **AES-CMAC** for key confirmation
- Debug key support for testing (Core Specification Vol 3, Part H, Section 2.3.5.6.1) - [ecc.c#L64-L83](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L64-L83)

#### **3. GATT Database Security (Bluetooth Core Specification Vol 3, Part G)**
- **Database Hash** computation using AES-CMAC (for GATT caching feature)
  - Ensures database integrity across reconnections
  - Defined in Core Specification Vol 3, Part G, Section 7.1

### Why PSA in Zephyr?

Zephyr implements these BLE security requirements using the **PSA Crypto API** (ARM Platform Security Architecture), which provides:
- **Standardized cryptographic interface** - Portable across hardware and software crypto implementations
- **Key isolation** - Keys handled through opaque handles, never exposed directly in memory
- **Hardware acceleration support** - Transparent use of crypto accelerators when available
- **Compliance** - Meets security certification requirements (FIPS, Common Criteria)

---

## PSA in BLE Security Operations

### Architecture Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                    BLE Host Stack (Host API)                     │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │   SMP Layer  │  │   ATT/GATT   │  │  Connection Mgmt    │   │
│  │  (Pairing)   │  │   (GATT DB)  │  │   (Encryption)      │   │
│  └──────┬───────┘  └──────┬───────┘  └─────────┬───────────┘   │
│         │                  │                     │               │
│         │ ECDH, Keys       │ CMAC Hash          │ AES Encrypt   │
│         v                  v                     v               │
│  ┌──────────────┐  ┌──────────────┐  ┌─────────────────────┐   │
│  │    ecc.c     │  │   gatt.c     │  │   crypto_psa.c      │   │
│  │  (ECC/ECDH)  │  │ (DB Hash)    │  │  (AES Encryption)   │   │
│  └──────┬───────┘  └──────┬───────┘  └─────────┬───────────┘   │
│         │                  │                     │               │
│         └──────────────────┴─────────────────────┘               │
│                            │                                     │
│                    ┌───────v─────────┐                          │
│                    │  PSA Crypto API  │                          │
│                    └───────┬─────────┘                          │
└────────────────────────────┼──────────────────────────────────┘
                             │
                    ┌────────v────────┐
                    │  mbedTLS PSA    │ (or Hardware Crypto)
                    │  Implementation │
                    └─────────────────┘
```

### PSA Operations by BLE Use Case

| BLE Operation | PSA Function | Key Type | Algorithm | File |
|---------------|--------------|----------|-----------|------|
| **Pairing: Public Key Generation** | `psa_generate_key()` | ECC P-256 key pair | secp256r1 | [ecc.c:140](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L140) |
| **Pairing: ECDH Key Agreement** | `psa_raw_key_agreement()` | ECC P-256 key pair | ECDH | [ecc.c:223](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L223) |
| **Encryption: AES-128 (LE)** | `psa_cipher_encrypt()` | AES-128 | ECB_NO_PADDING | [crypto_psa.c:96](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L96) |
| **Encryption: AES-128 (BE)** | `psa_cipher_encrypt()` | AES-128 | ECB_NO_PADDING | [crypto_psa.c:143](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L143) |
| **GATT: Database Hash** | `psa_mac_sign_finish()` | AES-128 | AES-CMAC | [gatt.c:780+](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/gatt.c#L780) |
| **Random Number Generation** | `psa_generate_random()` | N/A | PSA RNG | [crypto_psa.c:46](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L46) |
| **Key Import (Public Key)** | `psa_import_key()` | ECC P-256 public | secp256r1 | [ecc.c:108](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L108) |
| **Key Cleanup** | `psa_destroy_key()` | Any | N/A | Multiple locations |

---

## Step-by-Step: How BLE Host Uses PSA

### Step 1: Initialization
**When:** During Bluetooth stack initialization (`bt_enable()`)

```c
// File: crypto_psa.c, Function: bt_crypto_init()
int bt_crypto_init(void)
{
    psa_status_t status = psa_crypto_init();  // Initialize PSA subsystem
    if (status != PSA_SUCCESS) {
        return -EIO;
    }
    return 0;
}
```
📍 [crypto_psa.c#L32-L41](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L32-L41)

**What happens:**
- PSA Crypto framework is initialized
- Crypto drivers (hardware or software) are registered
- Random number generator is seeded

---

### Step 2: LE Secure Connections - Public Key Generation
**When:** Device prepares for LESC pairing

```c
// File: ecc.c, Function: generate_pub_key() [Work queue handler]
static void generate_pub_key(struct k_work *work)
{
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;
    
    // 1. Configure key attributes
    psa_set_key_type(&attr, PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1));
    psa_set_key_bits(&attr, 256);  // P-256 curve
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_DERIVE);
    psa_set_key_algorithm(&attr, PSA_ALG_ECDH);
    
    // 2. Generate ECC key pair
    ret = psa_generate_key(&attr, &key_id);
    
    // 3. Export public key (64 bytes: X || Y coordinates)
    ret = psa_export_public_key(key_id, tmp_pub_key_buf, sizeof(tmp_pub_key_buf), &tmp_len);
    
    // 4. Export private key (32 bytes) for later ECDH
    ret = psa_export_key(key_id, ecc.private_key_be, BT_PRIV_KEY_LEN, &tmp_len);
    
    // 5. Destroy key handle (keep exported keys in RAM)
    ret = psa_destroy_key(key_id);
    
    // 6. Convert to little-endian for BLE and store
    sys_memcpy_swap(pub_key, ecc.public_key_be, BT_PUB_KEY_COORD_LEN);
}
```
📍 [ecc.c#L128-L195](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L128-L195)

**PSA Key Attributes Set:**
- **Type:** `PSA_KEY_TYPE_ECC_KEY_PAIR(PSA_ECC_FAMILY_SECP_R1)` - Elliptic curve key pair (NIST P-256)
- **Bits:** 256 - Key size
- **Usage:** `PSA_KEY_USAGE_EXPORT | PSA_KEY_USAGE_DERIVE` - Allow export and ECDH operations
- **Algorithm:** `PSA_ALG_ECDH` - Elliptic Curve Diffie-Hellman

**Note:** This operation runs in the **long work queue** (`BT_LONG_WQ`) because ECC operations are CPU-intensive (typically 50-200ms).

---

### Step 3: LE Secure Connections - ECDH Key Agreement
**When:** During pairing, after exchanging public keys with peer

```c
// File: ecc.c, Function: generate_dh_key() [Work queue handler]
static void generate_dh_key(struct k_work *work)
{
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;
    uint8_t tmp_pub_key_buf[BT_PUB_KEY_LEN + 1] = { 0x04 };  // 0x04 prefix required by PSA
    
    // 1. Configure key attributes (same as generation)
    set_key_attributes(&attr);
    
    // 2. Import our private key
    const uint8_t *priv_key = IS_ENABLED(CONFIG_BT_USE_DEBUG_KEYS) ?
                               debug_private_key_be : ecc.private_key_be;
    ret = psa_import_key(&attr, priv_key, BT_PRIV_KEY_LEN, &key_id);
    
    // 3. Prepare peer's public key (add 0x04 prefix for PSA format)
    memcpy(&tmp_pub_key_buf[1], ecc.public_key_be, BT_PUB_KEY_LEN);
    
    // 4. Perform ECDH key agreement
    ret = psa_raw_key_agreement(PSA_ALG_ECDH, key_id, 
                                tmp_pub_key_buf, sizeof(tmp_pub_key_buf),
                                ecc.dhkey_be, BT_DH_KEY_LEN, &tmp_len);
    
    // 5. Destroy key handle
    ret = psa_destroy_key(key_id);
    
    // 6. Invoke callback with computed DH key
    dh_key_cb(dhkey);  // 32-byte shared secret
}
```
📍 [ecc.c#L197-L261](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L197-L261)

**ECDH Process:**
1. **Input:** Our private key (32 bytes) + Peer's public key (64 bytes)
2. **PSA Operation:** `psa_raw_key_agreement(PSA_ALG_ECDH, ...)`
3. **Output:** Shared secret DH Key (32 bytes)
4. **Security:** Private key never leaves PSA-protected memory during operation

**BLE Context:** The computed DH key is used by SMP to derive:
- MacKey (for authentication)
- LTK (Long Term Key for encryption)
- According to Bluetooth Core Specification Vol 3, Part H, Section 2.3.5.6

---

### Step 4: AES Encryption (Legacy Pairing & Data Encryption)
**When:** Encrypting session keys, computing cryptographic functions during pairing

```c
// File: crypto_psa.c, Function: bt_encrypt_le()
int bt_encrypt_le(const uint8_t key[16], const uint8_t plaintext[16],
                  uint8_t enc_data[16])
{
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    psa_key_id_t key_id;
    size_t out_len;
    
    // 1. Configure AES-128 key attributes
    psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attr, 128);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_ENCRYPT);
    psa_set_key_algorithm(&attr, PSA_ALG_ECB_NO_PADDING);
    
    // 2. Import AES key (swap byte order for LE)
    sys_memcpy_swap(tmp, key, 16);
    status = psa_import_key(&attr, tmp, 16, &key_id);
    
    // 3. Encrypt plaintext
    sys_memcpy_swap(tmp, plaintext, 16);
    status = psa_cipher_encrypt(key_id, PSA_ALG_ECB_NO_PADDING, 
                                tmp, 16, enc_data, 16, &out_len);
    
    // 4. Destroy key
    destroy_status = psa_destroy_key(key_id);
    
    // 5. Swap output back to little-endian
    sys_mem_swap(enc_data, 16);
    
    return 0;
}
```
📍 [crypto_psa.c#L66-L116](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L66-L116)

**PSA Key Attributes:**
- **Type:** `PSA_KEY_TYPE_AES` - Symmetric AES key
- **Bits:** 128 - AES-128
- **Usage:** `PSA_KEY_USAGE_ENCRYPT` - Encryption only
- **Algorithm:** `PSA_ALG_ECB_NO_PADDING` - AES-ECB mode (as specified by BLE)

**BLE Usage:**
- **e() function** (BLE Security Toolbox) - Used in STK/LTK generation
- **ah() function** - Resolvable Private Address generation (with IRK)
- **Data encryption** during established connections

**Note:** Similar function `bt_encrypt_be()` exists for big-endian operations at [crypto_psa.c#L118-L161](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L118-L161)

---

### Step 5: GATT Database Hash (Optional - GATT Caching)
**When:** Client requests database hash to verify cached GATT database

```c
// File: gatt.c, Function: db_hash_gen() [Simplified]
static int db_hash_gen(struct gen_hash_state *state)
{
    psa_key_attributes_t attr = PSA_KEY_ATTRIBUTES_INIT;
    
    // 1. Configure AES-128 key for CMAC
    psa_set_key_type(&attr, PSA_KEY_TYPE_AES);
    psa_set_key_bits(&attr, 128);
    psa_set_key_usage_flags(&attr, PSA_KEY_USAGE_SIGN_MESSAGE);
    psa_set_key_algorithm(&attr, PSA_ALG_CMAC);
    
    // 2. Import all-zeros key (per BLE spec for database hash)
    uint8_t zero_key[16] = {0};
    status = psa_import_key(&attr, zero_key, 16, &state->key);
    
    // 3. Setup CMAC operation
    status = psa_mac_sign_setup(&state->operation, state->key, PSA_ALG_CMAC);
    
    // 4. Hash each GATT attribute incrementally
    foreach(attribute in database) {
        psa_mac_update(&state->operation, attr_data, attr_len);
    }
    
    // 5. Finalize CMAC (produce 16-byte hash)
    status = psa_mac_sign_finish(&state->operation, hash, 16, &hash_len);
    
    // 6. Cleanup
    psa_destroy_key(state->key);
    
    return 0;
}
```
📍 [gatt.c (Database hash functions)](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/gatt.c)

**CMAC Details:**
- **Algorithm:** AES-CMAC (Cipher-based Message Authentication Code)
- **Key:** 128-bit all-zeros key (per Bluetooth Core Spec Vol 3, Part G, Section 7.1)
- **Output:** 128-bit hash representing entire GATT database structure
- **Purpose:** Client verifies database hasn't changed since last connection

---

### Step 6: Random Number Generation
**When:** Generating nonces, challenges, random addresses

```c
// File: crypto_psa.c, Function: bt_rand()
int bt_rand(void *buf, size_t len)
{
    psa_status_t status = psa_generate_random(buf, len);
    
    if (status == PSA_SUCCESS) {
        return 0;
    }
    
    return -EIO;
}
```
📍 [crypto_psa.c#L43-L54](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L43-L54)

**Alternative:** If `CONFIG_BT_HOST_CRYPTO_PRNG` is disabled, falls back to HCI LE Random command (controller-based RNG).

**BLE Usage:**
- Nonce generation during pairing
- Random static addresses
- Random resolvable addresses (with IRK)
- Challenge-response authentication

---

## Key Lifecycle and Usage

### ECC Keys (LESC Pairing)

```
┌─────────────────────────────────────────────────────────────────────┐
│                     LESC Key Lifecycle                              │
└─────────────────────────────────────────────────────────────────────┘

1. KEY GENERATION (when pairing starts)
   ┌────────────────────────────────────────────┐
   │  psa_generate_key()                        │  PSA generates P-256 key pair
   │  ├─> Private Key (32 bytes)                │  └─> Stored in psa_key_id handle
   │  └─> Public Key (64 bytes, X||Y)           │
   └────────────────────────────────────────────┘
              │
              │ psa_export_key() / psa_export_public_key()
              v
   ┌────────────────────────────────────────────┐
   │  Export to RAM (ecc.c static storage)     │
   │  ├─> ecc.private_key_be[32]                │  Kept in RAM during pairing
   │  └─> ecc.public_key_be[64]                 │
   └────────────────────────────────────────────┘
              │
              │ sys_memcpy_swap() - Convert to LE
              v
   ┌────────────────────────────────────────────┐
   │  pub_key[64] (global, little-endian)       │  Sent to peer over SMP
   │  └─> Transmitted via SMP Public Key cmd    │
   └────────────────────────────────────────────┘

2. KEY AGREEMENT (after receiving peer's public key)
   ┌────────────────────────────────────────────┐
   │  psa_import_key(ecc.private_key_be)        │  Import our private key
   │  psa_raw_key_agreement(                    │  + Peer's public key
   │      our_private_key_handle,                │  = Compute shared secret
   │      peer_public_key)                       │
   │  └─> DH Key (32 bytes)                      │
   └────────────────────────────────────────────┘
              │
              │ dh_key_cb() callback
              v
   ┌────────────────────────────────────────────┐
   │  SMP (smp.c)                                │  Uses DH Key to derive:
   │  └─> Derive MacKey, LTK                     │  - MacKey (authentication)
   │       (using cryptographic functions)        │  - LTK (encryption key)
   └────────────────────────────────────────────┘

3. KEY STORAGE (if bonding enabled)
              │
              v
   ┌────────────────────────────────────────────┐
   │  bt_keys structure (keys.c)                 │
   │  └─> struct bt_keys {                       │  Stored in keys.c:key_pool[]
   │       bt_addr_le_t addr;                    │  (RAM + optionally flash)
   │       struct bt_ltk ltk;  // 16 bytes       │
   │       struct bt_irk irk;  // 16 bytes       │
   │       ...                                    │
   │  }                                           │
   └────────────────────────────────────────────┘
              │
              │ if CONFIG_BT_SETTINGS enabled
              v
   ┌────────────────────────────────────────────┐
   │  Settings Subsystem (settings.c)            │  Persistent storage via
   │  └─> bt_keys_store()                        │  Settings API
   │       └─> settings_save_one(               │  Format: "bt/keys/<addr>/<key>"
   │            "bt/keys/...", ...)               │
   └────────────────────────────────────────────┘
              │
              v
   ┌────────────────────────────────────────────┐
   │  Flash Storage (NVS/FCB backend)            │  Keys persisted in flash
   │  └─> Survives reboot/power cycle            │  for bonding
   └────────────────────────────────────────────┘

4. KEY DESTRUCTION (after pairing or on error)
   ┌────────────────────────────────────────────┐
   │  psa_destroy_key(key_id)                    │  PSA frees key handle
   └────────────────────────────────────────────┘
   ┌────────────────────────────────────────────┐
   │  memset(ecc.private_key_be, 0, 32)          │  Clear RAM copies
   │  memset(ecc.public_key_be, 0, 64)           │  (done at next operation)
   └────────────────────────────────────────────┘
```

### AES Keys (Ephemeral - Used Once)

```
┌─────────────────────────────────────────────────────────────────────┐
│                   AES Key Lifecycle (Ephemeral)                     │
└─────────────────────────────────────────────────────────────────────┘

1. KEY IMPORT (for each encryption operation)
   ┌────────────────────────────────────────────┐
   │  Application/SMP provides key[16]          │  From LTK, STK, or computed
   └────────────────────────────────────────────┘
              │
              v
   ┌────────────────────────────────────────────┐
   │  psa_import_key(key_data, 16, &key_id)     │  PSA creates key handle
   └────────────────────────────────────────────┘

2. ENCRYPTION
   ┌────────────────────────────────────────────┐
   │  psa_cipher_encrypt(                        │  Single operation
   │      key_id,                                 │
   │      PSA_ALG_ECB_NO_PADDING,                │
   │      plaintext[16],                          │
   │      ciphertext[16])                         │
   └────────────────────────────────────────────┘

3. KEY DESTRUCTION (immediately after use)
   ┌────────────────────────────────────────────┐
   │  psa_destroy_key(key_id)                    │  Key exists for <1ms typically
   └────────────────────────────────────────────┘

Notes:
- AES keys are NOT stored by PSA - created and destroyed per operation
- This follows PSA best practice: minimize key lifetime
- Original key data remains in caller's memory (e.g., struct bt_keys)
```

### Key Storage Locations

| Key Type | Temporary Storage (RAM) | Persistent Storage (Flash) | PSA Role |
|----------|-------------------------|---------------------------|----------|
| **ECC Private Key** | `ecc.private_key_be[32]` (ecc.c) | ❌ Never stored (regenerated) | Generate/import/export only |
| **ECC Public Key** | `pub_key[64]` (ecc.c) | ❌ Never stored (regenerated) | Generate/export only |
| **LTK (Long Term Key)** | `struct bt_keys::ltk` (keys.c) | ✅ `bt/keys/<addr>/ltk` | Used transiently for encryption |
| **IRK (Identity Resolving Key)** | `struct bt_keys::irk` (keys.c) | ✅ `bt/keys/<addr>/irk` | Used for RPA resolution |
| **CSRK (Connection Signature Resolving Key)** | `struct bt_keys::remote_csrk` (keys.c) | ✅ `bt/keys/<addr>/csrk` | Used for data signing |
| **DH Key (Shared Secret)** | `ecc.dhkey_be[32]` (ecc.c, transient) | ❌ Never stored (ephemeral) | Computed via ECDH |

**Important:** PSA acts as a **transient key processor** - keys are imported for specific operations then destroyed. Long-term storage is handled by the Settings subsystem.

---

## Dependency on Secure Storage

### Settings Subsystem Integration

The BLE host depends on the **Settings subsystem** (`CONFIG_SETTINGS`) for persistent key storage:

```
┌─────────────────────────────────────────────────────────────────────┐
│                    Storage Architecture                              │
└─────────────────────────────────────────────────────────────────────┘

┌──────────────────────┐
│   BLE Host (keys.c)  │  In-memory key pool: key_pool[CONFIG_BT_MAX_PAIRED]
│                      │  └─> struct bt_keys { ltk, irk, csrk, ... }
└──────────┬───────────┘
           │
           │ bt_keys_store() - Save key to persistent storage
           │ bt_keys_clear() - Remove key from memory
           v
┌──────────────────────┐
│  Settings Subsystem  │  Key-value store API
│  (settings.c)        │  └─> settings_save_one("bt/keys/<addr>/<type>", data, len)
└──────────┬───────────┘
           │
           │ Backend implementation (selected by Kconfig)
           v
┌────────────────────────────────────────────────────────────┐
│  Storage Backend (one of):                                  │
│  ├─ CONFIG_SETTINGS_NVS → NVS (Non-Volatile Storage)       │ Flash-based
│  ├─ CONFIG_SETTINGS_FCB → Flash Circular Buffer            │ Flash-based
│  ├─ CONFIG_SETTINGS_FILE → File System                     │ FS-based
│  └─ CONFIG_SETTINGS_CUSTOM → Custom backend                │ User-defined
└────────────────────────────────────────────────────────────┘
```

### Key Storage Format

📍 [settings.c (Key encoding)](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/settings.c)

**Storage Path Format:**
```
bt/<subsys>/<addr><type>/<key>

Examples:
- bt/keys/001122334455a/ltk      → LTK for address 00:11:22:33:44:55 (public)
- bt/keys/ffeeddccbbaa1/irk      → IRK for address FF:EE:DD:CC:BB:AA (random)
- bt/id/<id>                     → Identity information
- bt/name                        → Device name
```

**Address Encoding:** 12-char hex (6 bytes) + 1-char type
- Last character: `0` = Public, `1` = Random

### Kconfig Dependencies for Storage

📍 [Kconfig#L249-L268](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/Kconfig#L249-L268)

```kconfig
config BT_SETTINGS
    bool "Store Bluetooth state and configuration persistently"
    depends on SETTINGS
    select MPU_ALLOW_FLASH_WRITE if ARM_MPU
    help
      When selected, the Bluetooth stack will take care of storing
      (and restoring) the Bluetooth state (e.g. pairing keys) and
      configuration persistently in flash.
```

**Related Options:**
- `BT_SETTINGS_CCC_STORE_ON_WRITE` - Store GATT CCC descriptors immediately
- `BT_SETTINGS_CF_STORE_ON_WRITE` - Store GATT Characteristic Format descriptors immediately
- `BT_SETTINGS_DELAYED_STORE` - Delay storage to batch writes (reduces flash wear)
- `BT_KEYS_OVERWRITE_OLDEST` - Overwrite oldest keys when storage full

### Security Considerations

**Without Secure Storage:**
- Keys lost on reboot → **No bonding support**
- Devices must re-pair every connection
- Suitable for: Beacon applications, non-bonded connections

**With Secure Storage (CONFIG_BT_SETTINGS):**
- Keys persist across reboots → **Bonding enabled**
- Devices reconnect without re-pairing
- **Security Risk:** Keys stored in plaintext flash
  - ⚠️ **Recommendation:** Use flash encryption (e.g., TF-M, ARM TrustZone)
  - ⚠️ **Recommendation:** Use secure boot to prevent unauthorized access

**PSA and Secure Storage:**
PSA Crypto API does NOT provide key persistence - that's intentional:
- PSA focuses on **key operations** (generation, encryption, etc.)
- **Key storage** is application/system responsibility
- For production systems, consider:
  - **PSA Secure Storage API** (separate from PSA Crypto) - [Future enhancement]
  - **TF-M (Trusted Firmware-M)** - Secure storage in TrustZone Secure World
  - **Hardware-backed keystores** (e.g., secure element)

---

## Kconfig Dependencies

### Dependency Graph

📍 [Kconfig (PSA-related options)](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/Kconfig)

```
BT (Bluetooth Stack)
 │
 ├─ BT_SMP (Security Manager Protocol)
 │   │
 │   ├─ selects BT_ECC (unless BT_SMP_OOB_LEGACY_PAIR_ONLY)
 │   │   │
 │   │   ├─ selects PSA_CRYPTO ─────────────────┐
 │   │   ├─ selects PSA_WANT_ALG_ECDH           │
 │   │   ├─ selects PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_GENERATE
 │   │   ├─ selects PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_IMPORT
 │   │   ├─ selects PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_EXPORT
 │   │   ├─ selects PSA_WANT_ECC_SECP_R1_256    │
 │   │   ├─ imply MBEDTLS_PSA_P256M_DRIVER_ENABLED (optimized P-256)
 │   │   └─ imply BT_LONG_WQ (workqueue)        │
 │   │                                            │
 │   └─ (uses keys from keys.c)                  │
 │                                                │
 ├─ BT_HOST_CRYPTO (default y if !BT_CTLR_CRYPTO)│
 │   │                                            │
 │   ├─ selects PSA_CRYPTO ──────────────────────┤
 │   ├─ selects PSA_WANT_KEY_TYPE_AES            │
 │   └─ selects PSA_WANT_ALG_ECB_NO_PADDING      │
 │                                                │
 │   └─ BT_HOST_CRYPTO_PRNG (default y)          │
 │       └─ depends on BT_HOST_CRYPTO            │
 │                                                │
 ├─ BT_SETTINGS (optional, for bonding)          │
 │   │                                            │
 │   └─ depends on SETTINGS ─────────────────────┼─ Flash storage
 │                                                │
 └─ BT_GATT_CACHING (optional)                   │
     │                                            │
     └─ (uses PSA_ALG_CMAC via gatt.c)           │
                                                  │
                                                  │
┌─────────────────────────────────────────────────┼────┐
│              PSA Crypto Stack                    │    │
└──────────────────────────────────────────────────┼────┘
                                                   │
                     PSA_CRYPTO ◄──────────────────┘
                         │
         ┌───────────────┴───────────────┐
         │                               │
    PSA_WANT_*                     MBEDTLS_PSA_CRYPTO_C
    (Algorithm/Key                 (mbedTLS PSA implementation)
     selection)                              │
         │                                   │
         │                          ┌────────┴─────────┐
         │                          │                  │
         │                    mbedTLS Crypto     HW Crypto Drivers
         │                    (Software impl)    (Platform-specific)
         │                                               │
         └───────────────────────────────────────────────┘
```

### Detailed Kconfig Analysis

#### 1. BT_HOST_CRYPTO
📍 [Kconfig#L225-L233](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/Kconfig#L225-L233)

```kconfig
config BT_HOST_CRYPTO
    bool "Use crypto functionality implemented in the Bluetooth host"
    default y if !BT_CTLR_CRYPTO
    select PSA_CRYPTO
    select PSA_WANT_KEY_TYPE_AES
    select PSA_WANT_ALG_ECB_NO_PADDING
```

**Purpose:** Enables AES-128 encryption in host (crypto_psa.c)

**When enabled:**
- Host provides cryptographic functions (AES encryption)
- Used when controller lacks crypto support (`!BT_CTLR_CRYPTO`)
- **Required for:** Legacy pairing, LE Secure Connections STK/LTK operations

**PSA Requirements:**
- `PSA_CRYPTO` - Core PSA framework
- `PSA_WANT_KEY_TYPE_AES` - AES key support
- `PSA_WANT_ALG_ECB_NO_PADDING` - AES-ECB algorithm (BLE uses ECB mode)

#### 2. BT_HOST_CRYPTO_PRNG
📍 [Kconfig#L235-L247](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/Kconfig#L235-L247)

```kconfig
config BT_HOST_CRYPTO_PRNG
    bool "Use PSA crypto API library for random number generation"
    default y
    depends on BT_HOST_CRYPTO
    help
      When selected, will use PSA Crypto API library for random number generation.
      This will consume additional ram, but may speed up the generation of random
      numbers.
      
      Otherwise, random numbers will be generated through multiple HCI calls,
      which will not consume additional resources, but may take a long time,
      depending on the length of the random data.
```

**Trade-offs:**
| Enabled (PSA RNG) | Disabled (HCI RNG) |
|-------------------|---------------------|
| ✅ Faster (1-2ms) | ⚠️ Slower (10-50ms depending on controller) |
| ⚠️ Uses more RAM (~2-4KB for RNG state) | ✅ No additional RAM |
| ✅ Host-side randomness | ⚠️ Controller-dependent quality |
| ✅ Good for: Frequent pairing, RPA generation | ✅ Good for: RAM-constrained devices |

#### 3. BT_ECC
📍 [Kconfig#L1104-L1117](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/Kconfig#L1104-L1117)

```kconfig
config BT_ECC
    bool
    select PSA_CRYPTO
    select PSA_WANT_ALG_ECDH
    select PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_GENERATE
    select PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_IMPORT
    select PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_EXPORT
    select PSA_WANT_ECC_SECP_R1_256
    imply MBEDTLS_PSA_P256M_DRIVER_ENABLED if MBEDTLS_PSA_CRYPTO_C
    imply BT_LONG_WQ
```

**Purpose:** Enables ECDH for LE Secure Connections (LESC)

**Hidden config:** Automatically selected by `BT_SMP` (unless legacy-only mode)

**PSA Requirements:**
- `PSA_WANT_ALG_ECDH` - ECDH key agreement algorithm
- `PSA_WANT_KEY_TYPE_ECC_KEY_PAIR_*` - ECC key operations (generate/import/export)
- `PSA_WANT_ECC_SECP_R1_256` - NIST P-256 (secp256r1) elliptic curve
- `MBEDTLS_PSA_P256M_DRIVER_ENABLED` - Optimized P-256 implementation (mbedTLS)

**Performance:**
- ECC operations are **CPU-intensive** (50-200ms on typical Cortex-M4)
- **BT_LONG_WQ** workqueue prevents blocking BT RX thread
- Consider hardware acceleration (e.g., ARM CryptoCell, STM32 PKA) for better performance

### Dependency Table

| Kconfig Option | PSA Dependencies | Code File | BLE Feature |
|----------------|------------------|-----------|-------------|
| **BT_HOST_CRYPTO** | PSA_CRYPTO<br>PSA_WANT_KEY_TYPE_AES<br>PSA_WANT_ALG_ECB_NO_PADDING | [crypto_psa.c](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c) | AES encryption (e function, STK/LTK) |
| **BT_HOST_CRYPTO_PRNG** | *(inherits from BT_HOST_CRYPTO)* | [crypto_psa.c#L44](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L44) | Random number generation |
| **BT_ECC** | PSA_CRYPTO<br>PSA_WANT_ALG_ECDH<br>PSA_WANT_KEY_TYPE_ECC_*<br>PSA_WANT_ECC_SECP_R1_256 | [ecc.c](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c) | LE Secure Connections (LESC) |
| **BT_GATT_CACHING** | PSA_CRYPTO<br>PSA_WANT_ALG_CMAC<br>PSA_WANT_KEY_TYPE_AES | [gatt.c](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/gatt.c) | GATT database hash |
| **BT_SETTINGS** | SETTINGS<br>*(no direct PSA dependency)* | [settings.c](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/settings.c) | Persistent key storage (bonding) |

---

## Code References

### Primary PSA Implementation Files

| File | Purpose | Key Functions | Link |
|------|---------|---------------|------|
| **crypto_psa.c** | AES encryption, RNG | `bt_crypto_init()`<br>`bt_encrypt_le()`<br>`bt_encrypt_be()`<br>`bt_rand()` | [View File](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c) |
| **ecc.c** | ECDH key agreement | `bt_pub_key_gen()`<br>`bt_dh_key_gen()`<br>`generate_pub_key()`<br>`generate_dh_key()` | [View File](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c) |
| **gatt.c** | GATT database hash | `db_hash_gen()` (internal)<br>`bt_gatt_hash_generate()` | [View File](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/gatt.c) |
| **keys.c** | Key management | `bt_keys_get_addr()`<br>`bt_keys_store()`<br>`bt_keys_clear()` | [View File](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/keys.c) |
| **settings.c** | Persistent storage | `bt_settings_encode_key()`<br>`bt_keys_store()` | [View File](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/settings.c) |
| **smp.c** | Security Manager | *(uses ecc.c and crypto_psa.c)* | [View File](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/smp.c) |

### Key Code Locations

#### crypto_psa.c

| Function | Line Range | Description |
|----------|------------|-------------|
| `bt_crypto_init()` | [32-41](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L32-L41) | Initialize PSA Crypto (`psa_crypto_init()`) |
| `bt_rand()` (PSA) | [44-54](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L44-L54) | Generate random data via `psa_generate_random()` |
| `bt_rand()` (HCI fallback) | [56-63](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L56-L63) | Fallback to HCI LE Rand command |
| `bt_encrypt_le()` | [66-116](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L66-L116) | AES-128 ECB encryption (little-endian) |
| `bt_encrypt_be()` | [118-161](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/crypto_psa.c#L118-L161) | AES-128 ECB encryption (big-endian) |

#### ecc.c

| Function | Line Range | Description |
|----------|------------|-------------|
| Debug keys (constants) | [65-83](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L65-L83) | BLE Core Spec debug public/private keys |
| `bt_pub_key_is_valid()` | [90-118](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L90-L118) | Validate public key via `psa_import_key()` |
| `generate_pub_key()` | [128-195](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L128-L195) | Generate P-256 key pair, export keys |
| `generate_dh_key()` | [197-261](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L197-L261) | Perform ECDH key agreement (`psa_raw_key_agreement()`) |
| `bt_pub_key_gen()` | [263-305](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L263-L305) | Public API: Request public key generation |
| `bt_dh_key_gen()` | [335-367](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/ecc.c#L335-L367) | Public API: Request DH key computation |

#### Kconfig (PSA dependencies)

| Section | Line Range | Description |
|---------|------------|-------------|
| `BT_HOST_CRYPTO` | [225-233](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/Kconfig#L225-L233) | AES crypto via PSA |
| `BT_HOST_CRYPTO_PRNG` | [235-247](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/Kconfig#L235-L247) | PSA RNG option |
| `BT_SETTINGS` | [249-268](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/Kconfig#L249-L268) | Persistent storage configuration |
| `BT_ECC` | [1104-1117](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/Kconfig#L1104-L1117) | ECDH/ECC via PSA |

#### CMakeLists.txt

| Line Range | Description |
|------------|-------------|
| [35-37](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/CMakeLists.txt#L35-L37) | Conditional build of `crypto_psa.c` (if `CONFIG_BT_HOST_CRYPTO`) |
| [39-42](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/CMakeLists.txt#L39-L42) | Conditional build of `ecc.c` (if `CONFIG_BT_ECC`) |
| [117](https://github.com/PavelVPV/zephyr/blob/05b84df99688864910186370334889aebbb1505b/subsys/bluetooth/host/CMakeLists.txt#L117) | Link mbedTLS library (PSA implementation) |

### Related Documentation

- **Bluetooth Core Specification:** [Bluetooth SIG Specifications](https://www.bluetooth.com/specifications/specs/)
  - Volume 3, Part H: Security Manager Specification
  - Volume 3, Part G: Generic Attribute Profile (GATT)
- **PSA Crypto API Specification:** [ARM PSA Crypto API](https://arm-software.github.io/psa-api/crypto/)
- **Zephyr Bluetooth:** [Zephyr Bluetooth Documentation](https://docs.zephyrproject.org/latest/connectivity/bluetooth/index.html)
- **Zephyr Settings:** [Settings Subsystem](https://docs.zephyrproject.org/latest/services/settings/index.html)

---

## Summary

The Zephyr BLE host leverages **PSA Crypto API** to implement security requirements mandated by the Bluetooth Core Specification:

1. **BLE Spec Context:**
   - LE Legacy Pairing requires AES-128 encryption
   - LE Secure Connections requires ECDH (P-256) and AES-CMAC
   - GATT Caching requires database hash (AES-CMAC)

2. **PSA Operations:**
   - **ECC Key Generation** (`psa_generate_key`) - P-256 key pairs for LESC
   - **ECDH Key Agreement** (`psa_raw_key_agreement`) - Shared secret computation
   - **AES Encryption** (`psa_cipher_encrypt`) - Session key encryption
   - **CMAC** (`psa_mac_sign_finish`) - Database hashing
   - **Random Generation** (`psa_generate_random`) - Nonces, addresses

3. **Key Lifecycle:**
   - ECC keys: Generated → Used for ECDH → Derived LTK stored
   - AES keys: Ephemeral (import → use → destroy in <1ms)
   - Storage: PSA handles operations, Settings subsystem handles persistence

4. **Kconfig Dependencies:**
   - `BT_HOST_CRYPTO` → PSA + AES → AES encryption
   - `BT_ECC` → PSA + ECDH + P-256 → LESC pairing
   - `BT_SETTINGS` → SETTINGS → Persistent bonding

5. **Security:**
   - PSA provides key isolation (opaque handles)
   - Flash storage requires additional protection (encryption, secure boot)
   - Consider TF-M for production secure storage

**Key Takeaway:** PSA Crypto API provides a standardized, secure, hardware-acceleratable cryptographic foundation for BLE security operations in Zephyr, ensuring compliance with Bluetooth Core Specification requirements.

---

**Document Version:** 1.0  
**Based on Zephyr Commit:** [05b84df99688864910186370334889aebbb1505b](https://github.com/PavelVPV/zephyr/commit/05b84df99688864910186370334889aebbb1505b)  
**Last Updated:** 2026-02-16
