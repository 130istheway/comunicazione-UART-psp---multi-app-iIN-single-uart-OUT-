/*
 * APP LOCK - Template for BT_MANAGER
 * 
 * Simple mutual exclusion: only ONE app (IR Controller or BT Manager) can run at a time.
 * Uses a single general "app_lock" semaphore created by uart-controller.c
 * 
 * Copy these two functions into bt_manager_plugin.c
 */

// --- GENERAL APP LOCK ---

int try_acquire_app_lock(void) {
    // Try to wait on "app_lock" semaphore (non-blocking, 0 timeout)
    int result = sceKernelWaitSema_timed("app_lock", 1, 0);
    
    if (result == 0) {
        return 1; // Lock acquired - we can run
    } else {
        return 0; // Lock already held by another app
    }
}

void release_app_lock(void) {
    // Release the app lock for other apps to use
    sceKernelSignalSema_by_name("app_lock", 1);
}

/*
 * INTEGRATION FOR BT_MANAGER:
 * 
 * In main_thread() or module_start():
 * 
 *   // Try to acquire general app lock
 *   if (!try_acquire_app_lock()) {
 *       debug_print(10, 50, "ERROR: Another app is running!", 0xFFFF0000);
 *       debug_print(10, 70, "(IR Controller or BT Manager)", 0xFFFF0000);
 *       return -1;
 *   }
 *   
 *   // ... rest of initialization ...
 * 
 * In module_stop():
 * 
 *   release_app_lock();
 * 
 * HOW IT WORKS:
 * 
 * uart-controller.c creates "app_lock" semaphore (initial value = 1)
 * 
 * First app to start:
 *   try_acquire_app_lock() → sceKernelWaitSema("app_lock") → SUCCESS (gets 1)
 *   → App runs with lock held
 * 
 * Second app tries to start:
 *   try_acquire_app_lock() → sceKernelWaitSema("app_lock") → FAIL (already 0)
 *   → Shows error and exits
 * 
 * First app exits:
 *   release_app_lock() → sceKernelSignalSema("app_lock") → increments to 1
 *   → Lock available for next app
 * 
 * SIMPLE AND ELEGANT!
 * ✓ One semaphore for all
 * ✓ No app-specific tracking
 * ✓ Automatic on kernel shutdown
 * ✓ Non-blocking check (returns immediately)
 */
