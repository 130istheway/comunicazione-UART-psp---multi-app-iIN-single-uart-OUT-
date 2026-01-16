/*
==========================================================
KCX BT EMITTER MANAGER PLUGIN v1.11
C Implementation for PSP Kernel Module
==========================================================

This plugin communicates with the UART Manager kernel module
to control the KCX BT EMITTER 5 Bluetooth module via AT commands.

Features:
- Scan for available Bluetooth devices
- Connect to specific MAC addresses
- Query auto-connect memory (VM Link)
- Manage device connections
- Display connection status
*/

#include <pspsdk.h>
#include <pspkernel.h>
#include <pspdebug.h>
#include <pspctrl.h>
#include <pspge.h>
#include <pspgum.h>
#include <pspgu.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

PSP_MODULE_INFO("BTManager", 0x0000, 1, 1); // User Module
PSP_MAIN_THREAD_ATTR(PSP_THREAD_ATTR_USER);

// --- GRAPHICS CONSTANTS ---
#define SCREEN_WIDTH    480
#define SCREEN_HEIGHT   272
#define LINE_HEIGHT     12

// --- STATE STRUCTURE ---
typedef struct {
    SceUID cmd_pipe;
    int connected;
    char device_name[32];
    char mac_address[12];
    char software_version[16];
    
    int menu_state;     // 0=main, 1=scanning, 2=vm_link
    int selected_device;
    int device_count;
    
    char status_msg[64];
    unsigned int last_update_time;
} BT_Manager_State;

static BT_Manager_State state = {0};

// Device list for scanning results
typedef struct {
    char name[32];
    char mac[12];
} BT_Device;

static BT_Device device_list[16] = {0};
static int device_count = 0;

// VM Link memory storage
typedef struct {
    char info[64];
} VM_Link_Entry;

static VM_Link_Entry vm_list[10] = {0};
static int vm_count = 0;

// --- SCREEN DRAWING UTILITIES ---
void debug_print(int x, int y, const char *text, unsigned int color) {
    pspDebugScreenSetXY(x / 6, y / LINE_HEIGHT);
    pspDebugScreenSetTextColor(color);
    pspDebugScreenPuts(text);
}

void init_graphics(void) {
    pspDebugScreenInit();
    pspDebugScreenSetBackColor(0x00000000); // Black background
    pspDebugScreenSetTextColor(0xFFFFFFFF); // White text
}

// --- AT COMMAND SENDING ---
void send_at_command(const char *command) {
    if (state.cmd_pipe < 0) return;
    
    unsigned int cmd_len = strlen(command);
    sceKernelSendMsgPipe(state.cmd_pipe, (void*)command, cmd_len, 
                         PSP_MSGPIPE_NOWAIT, NULL, NULL);
    
    snprintf(state.status_msg, sizeof(state.status_msg), "Sent: %s", command);
    state.last_update_time = sceKernelGetSystemTimeLow();
}

void send_status_query(void) {
    send_at_command("AT+STATUS");
}

void send_scan_command(void) {
    device_count = 0;
    memset(device_list, 0, sizeof(device_list));
    send_at_command("AT+SCAN");
    snprintf(state.status_msg, sizeof(state.status_msg), "Scanning...");
    state.menu_state = 1;
}

void send_connect_mac(const char *mac) {
    char at_cmd[32];
    snprintf(at_cmd, sizeof(at_cmd), "AT+CONADD=%s", mac);
    send_at_command(at_cmd);
}

void send_disconnect(void) {
    send_at_command("AT+DISCON");
}

void send_vmlink_query(void) {
    vm_count = 0;
    memset(vm_list, 0, sizeof(vm_list));
    send_at_command("AT+VMLINK?");
    snprintf(state.status_msg, sizeof(state.status_msg), "Querying VM Link...");
    state.menu_state = 2;
}

void save_to_vmlink(const char *mac) {
    char at_cmd[32];
    snprintf(at_cmd, sizeof(at_cmd), "AT+ADDLINKADD=%s", mac);
    send_at_command(at_cmd);
    snprintf(state.status_msg, sizeof(state.status_msg), "Device saved to auto-connect");
}

// --- INPUT HANDLING ---
void handle_input(SceCtrlData *pad) {
    if (pad->Buttons & PSP_CTRL_HOME) {
        // Exit application
        sceKernelExitGame();
    }
    
    if (pad->Buttons & PSP_CTRL_TRIANGLE) {
        send_scan_command();
    }
    
    if (pad->Buttons & PSP_CTRL_CIRCLE) {
        send_status_query();
    }
    
    if (pad->Buttons & PSP_CTRL_START) {
        state.menu_state = (state.menu_state == 2) ? 0 : 2;
        if (state.menu_state == 2) {
            send_vmlink_query();
        } else {
            snprintf(state.status_msg, sizeof(state.status_msg), "Ready.");
        }
    }
    
    if (device_count > 0 && state.menu_state == 0) {
        if (pad->Buttons & PSP_CTRL_UP && state.selected_device > 0) {
            state.selected_device--;
        }
        if (pad->Buttons & PSP_CTRL_DOWN && state.selected_device < device_count - 1) {
            state.selected_device++;
        }
        
        if (pad->Buttons & PSP_CTRL_CROSS) {
            send_connect_mac(device_list[state.selected_device].mac);
            strncpy(state.device_name, device_list[state.selected_device].name, 
                    sizeof(state.device_name) - 1);
            state.connected = 1;
        }
        
        if (pad->Buttons & PSP_CTRL_RIGHT) {
            save_to_vmlink(device_list[state.selected_device].mac);
        }
        
        if (pad->Buttons & PSP_CTRL_SQUARE && state.connected) {
            send_disconnect();
            state.connected = 0;
        }
    }
}

// --- DISPLAY RENDERING ---
void render_main_menu(void) {
    pspDebugScreenClear();
    
    debug_print(10, 10, "KCX BT Manager v1.11", 0xFFFFFF00);
    
    // Status line
    if (state.connected) {
        char status[64];
        snprintf(status, sizeof(status), "CONNECTED TO: %s", state.device_name);
        debug_print(10, 25, status, 0xFF00FF00); // Green
    } else {
        debug_print(10, 25, "DISCONNECTED", 0xFFCCCCCC); // Gray
    }
    
    debug_print(10, 40, "Status Message:", 0xFFFFFF00);
    debug_print(10, 55, state.status_msg, 0xFFFFFFFF);
    
    debug_print(10, 75, "================================", 0xFF666666);
    
    if (device_count > 0) {
        debug_print(10, 90, "Found Devices:", 0xFF00FFFF);
        
        for (int i = 0; i < device_count && i < 12; i++) {
            char line[80];
            char prefix[4] = "  ";
            
            if (i == state.selected_device) {
                strcpy(prefix, "> ");
            }
            
            snprintf(line, sizeof(line), "%s%s (%s)", prefix, 
                    device_list[i].name, device_list[i].mac);
            
            unsigned int color = (i == state.selected_device) ? 0xFF00FF00 : 0xFFCCCCCC;
            debug_print(10, 105 + (i * 12), line, color);
        }
    }
    
    // Controls help
    debug_print(10, 220, "TRIANGLE: Scan  |  CIRCLE: Status  |  X: Connect  |  R: Save", 0xFF888888);
    debug_print(10, 235, "START: VM Link  |  SQUARE: Disconnect  |  HOME: Exit", 0xFF888888);
}

void render_vmlink_menu(void) {
    pspDebugScreenClear();
    
    debug_print(10, 10, "KCX BT Manager - Auto-Connect Memory", 0xFFFFFF00);
    debug_print(10, 30, "================================", 0xFF666666);
    
    if (vm_count > 0) {
        debug_print(10, 50, "Stored Devices:", 0xFF00FFFF);
        
        for (int i = 0; i < vm_count && i < 14; i++) {
            debug_print(10, 65 + (i * 12), vm_list[i].info, 0xFFFFFFFF);
        }
    } else {
        debug_print(10, 50, "No devices stored in auto-connect memory", 0xFFCCCCCC);
    }
    
    debug_print(10, 235, "START: Back to Main Menu", 0xFF888888);
}

// --- MAIN THREAD ---
int main_thread(SceSize args, void *argp) {
    init_graphics();
    
    // Open message pipe to UART Manager
    state.cmd_pipe = sceKernelCreateMsgPipe("SioCmdPipe", 1, 0, 256, NULL);
    if (state.cmd_pipe < 0) {
        pspDebugScreenPuts("ERROR: Cannot open UART Manager pipe!");
        sceKernelSleepThread();
        return -1;
    }
    
    state.connected = 0;
    state.menu_state = 0;
    state.selected_device = 0;
    strncpy(state.status_msg, "Initialized.", sizeof(state.status_msg) - 1);
    
    // Query initial status
    send_status_query();
    
    SceCtrlData pad;
    
    while (1) {
        // Read controller input
        sceCtrlReadBufferPositive(&pad, 1);
        handle_input(&pad);
        
        // Render appropriate menu
        if (state.menu_state == 2) {
            render_vmlink_menu();
        } else {
            render_main_menu();
        }
        
        sceDisplayWaitVblankStart();
        pspDebugScreenSwapBuffers();
        
        sceKernelDelayThread(16000); // ~60 FPS
    }
    
    return 0;
}

// --- MODULE START/STOP ---
int module_start(SceSize args, void *argp) {
    SceUID thid = sceKernelCreateThread("BTManager", main_thread, 0x20, 
                                         0x10000, 0, NULL);
    if (thid >= 0) {
        sceKernelStartThread(thid, args, argp);
    }
    return 0;
}

int module_stop(SceSize args, void *argp) {
    if (state.cmd_pipe >= 0) {
        sceKernelDeleteMsgPipe(state.cmd_pipe);
    }
    return 0;
}
