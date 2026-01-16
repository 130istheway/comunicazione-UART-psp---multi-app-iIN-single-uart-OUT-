/*
 * UART LOCK CONTROL - Template for BT_MANAGER (and other applications)
 * 
 * Copy these functions into your application to ensure exclusive 
 * access to UART-Controller. Uses kernel semaphore "uart_lock" 
 * created by the UART Manager kernel module.
 * 
 * Usage:
 *   1. At startup: if (!acquire_uart_lock()) { handle_error(); }
 *   2. At shutdown: release_uart_lock();
 * 
 * The semaphore is created by uart-controller.c with timeout = 5 seconds
 */

#define LOCK_TIMEOUT_MS 5000

// --- UART LOCK CONTROL FOR BT_MANAGER ---

int acquire_uart_lock(void) {
    // Try to acquire the "uart_lock" semaphore created by kernel module
    // timeout: 5 seconds (5000 milliseconds)
    int result = sceKernelWaitSema_timed("uart_lock", 1, LOCK_TIMEOUT_MS);
    
    if (result == 0) {
        return 1; // Lock acquired successfully
    } else {
        // Could not acquire lock within timeout
        return 0; // Lock acquisition failed
    }
}

void release_uart_lock(void) {
    // Release the "uart_lock" semaphore
    sceKernelSignalSema_by_name("uart_lock", 1);
}

int is_uart_locked_by_other(void) {
    // Try to acquire lock with 0 timeout (non-blocking check)
    int result = sceKernelWaitSema_timed("uart_lock", 1, 0);
    
    if (result == 0) {
        // We acquired it, so it's not locked by anyone else
        sceKernelSignalSema_by_name("uart_lock", 1);
        return 0;
    } else {
        // Could not acquire, so it's locked by someone else
        return 1;
    }
}

/*
 * INTEGRATION INSTRUCTIONS FOR BT_MANAGER:
 * 
 * 1. Copy these three functions to bt_manager_plugin.c
 * 2. In module_start() or main_thread():
 *    
 *    if (!acquire_uart_lock()) {
 *        debug_print(10, 50, "ERROR: UART locked by IR Controller!", 0xFFFF0000);
 *        debug_print(10, 70, "Wait or exit IR Controller first.", 0xFFFF0000);
 *        return -1;
 *    }
 * 
 * 3. In module_stop():
 *    
 *    release_uart_lock();
 * 
 * SEMAPHORE BEHAVIOR:
 * - Semaphore name: "uart_lock"
 * - Type: Binary semaphore (initial value = 1)
 * - Created by: uart-controller.c (kernel module)
 * - Timeout: 5 seconds (5000 ms)
 * - Only ONE app can hold it at a time
 * 
 * MUTUAL EXCLUSION:
 * - IR_CONTROLLER acquires lock at startup
 * - BT_MANAGER tries to acquire → WAITS up to 5 seconds
 * - If timeout, BT_MANAGER cannot access UART
 * - IR_CONTROLLER releases lock on exit
 * - BT_MANAGER can then acquire it
 * 
 * ADVANTAGES OVER FILE LOCK:
 * ✓ No file I/O overhead
 * ✓ Atomic operation (no race conditions)
 * ✓ Built-in timeout handling
 * ✓ Automatic cleanup on app crash
 * ✓ Kernel-managed (more reliable)
 */
