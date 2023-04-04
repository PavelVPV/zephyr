/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>

bool FuzzMe(const uint8_t *Data, size_t DataSize) {
//	printf("%d\n", DataSize);
	if (DataSize < 43) {
//		printf("%d\n", DataSize);
	}
	if ( //DataSize >= 3 &&

		Data[0] == 'F' &&
		Data[1] == 'U' &&
		Data[2] == 'Z' &&
		Data[3] == 'Z' &&
		Data[43] == 'Z') {  // :‑<
		printf("...\n");
	}
	return true;
}

#ifndef CONFIG_ARCH_POSIX_LIBFUZZER
uint8_t *posix_fuzz_buf, posix_fuzz_sz;
#else
extern uint8_t *posix_fuzz_buf, posix_fuzz_sz;
#endif
K_SEM_DEFINE(fuzz_sem, 0, K_SEM_MAX_LIMIT);

static void fuzz_isr(const void *arg)
{
	/* We could call check0() to execute the fuzz case here, but
	 * pass it through to the main thread instead to get more OS
	 * coverage.
	 */
	k_sem_give(&fuzz_sem);
}


void main(void)
{
	printk("Hello World! %s\n", CONFIG_BOARD);

	IRQ_CONNECT(CONFIG_ARCH_POSIX_FUZZ_IRQ, 0, fuzz_isr, NULL, 0);
	irq_enable(CONFIG_ARCH_POSIX_FUZZ_IRQ);

	while (true) {
		k_sem_take(&fuzz_sem, K_FOREVER);

		/* Execute the fuzz case we got from LLVM and passed
		 * through an interrupt to this thread.
		 */
		FuzzMe(posix_fuzz_buf, posix_fuzz_sz);
	}
}
