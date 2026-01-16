#include <pspsdk.h>
#include <pspkernel.h>
#include <pspdebug.h>
#include <psppower.h>
#include <systemctrl.h> // For sctrlHENFindFunction
#include <string.h>
#include <stdio.h>

PSP_MODULE_INFO("UART_Manager", 0x1000, 1, 1); // Kernel Mode
PSP_MAIN_THREAD_ATTR(0x10); // High Priority Thread

// --- GLOBAL STATE VARIABLES ---
static SceUID cmd_pipe_out = -1;
static SceUID ir_cmd_pipe = -1;
static SceUID app_lock_semaphore = -1;          // General semaphore - prevents multiple apps from running

// --- ESP32 IR TRANSMITTER STRUCTURES ---
typedef struct {
    unsigned char ir_enabled;          // 1=enabled, 0=disabled
    unsigned char ir_mode;             // 0=TX (transmitter), 1=RX (receiver)
    unsigned int last_ir_code;         // Last transmitted IR code
    unsigned char ir_protocol;         // 0=NEC, 1=RC5, 2=RC6, etc.
    unsigned int ir_frequency;         // Carrier frequency in Hz
} ESP32_IR_State;

static ESP32_IR_State ir_state = {0};

// --- BT MODULE AT COMMAND STRUCTURES ---
typedef struct {
    unsigned char bt_connected;        // 0=no connection, 1=connected
    unsigned char mac_address[12];     // Hex string MAC address
    unsigned char device_name[32];     // Device name
    unsigned char software_version[16]; // Software version
    unsigned char status_value;        // STATUS:x response
} BT_Module_State;

static BT_Module_State bt_state = {0};

// --- COMMUNICATION BUFFERS ---
#define AT_BUFFER_SIZE 256
#define IR_BUFFER_SIZE 128
static char at_response_buffer[AT_BUFFER_SIZE];
static int at_buffer_index = 0;
static char ir_response_buffer[IR_BUFFER_SIZE];
static int ir_buffer_index = 0;
static int last_char_time = 0;


// --- AT COMMAND PARSER ---
// Parses incoming AT command responses from Bluetooth module
void parse_at_response(const char *response) {
    if (strstr(response, "OK+") != NULL) {
        // Generic OK response
        return;
    }
    
    if (strstr(response, "STATUS:") != NULL) {
        // < 4>: AT+STATUS response - parse connection status
        // Format: STATUS:x (x=0: no connection, x=1: connected)
        unsigned char status_val = response[strlen(response) - 1] - '0';
        bt_state.bt_connected = status_val;
        pspDebugScreenPrintf("BT Status: %s\n", status_val ? "CONNECTED" : "DISCONNECTED");
        return;
    }
    
    if (strstr(response, "KCX_BTEMITTER") != NULL) {
        // < 3>: AT+GMR? response - software version
        // Format: KCX_BTEMITTER_Vx.x
        strncpy((char*)bt_state.software_version, response, 15);
        bt_state.software_version[15] = '\0';
        pspDebugScreenPrintf("BT Version: %s\n", bt_state.software_version);
        return;
    }
    
    if (strstr(response, "CON:0x") != NULL) {
        // < 5>: AT+CONADD response - MAC address connection
        // Format: CON:0xf44efdecd39d
        char *mac_start = strstr(response, "0x");
        if (mac_start) {
            strncpy((char*)bt_state.mac_address, mac_start + 2, 12);
            bt_state.mac_address[12] = '\0';
            pspDebugScreenPrintf("Connected to MAC: %s\n", bt_state.mac_address);
        }
        return;
    }
    
    if (strstr(response, "CONNECTED") != NULL) {
        // Connection established
        bt_state.bt_connected = 1;
        pspDebugScreenPrintf("BT: Device Connected\n");
        return;
    }
    
    if (strstr(response, "DISCONNECT") != NULL) {
        // Connection lost
        bt_state.bt_connected = 0;
        pspDebugScreenPrintf("BT: Device Disconnected\n");
        return;
    }
    
    if (strstr(response, "MacAdd:0x") != NULL) {
        // < 7>: AT+PAIR response - device discovery
        // Format: MacAdd:0xf44efdecd39d
        char *mac_start = strstr(response, "0x");
        if (mac_start) {
            pspDebugScreenPrintf("Found BT Device: %s\n", mac_start);
        }
        return;
    }
    
    if (strstr(response, "VM_MacAdd") != NULL) {
        // < 10>: AT+VMLINK? response - auto-connect memory
        // Format: VM_MacAdd0=0xf44efdecd39d
        pspDebugScreenPrintf("VM Link: %s\n", response);
        return;
    }
}

// --- IR RESPONSE PARSER ---
// Parses incoming IR responses from ESP32 module
void parse_ir_response(const char *response) {
    if (strstr(response, "IR:TX:") != NULL) {
        // IR transmission acknowledgment
        // Format: IR:TX:0x1A2B3C4D (code transmitted)
        unsigned int code = 0;
        sscanf(response, "IR:TX:0x%x", &code);
        ir_state.last_ir_code = code;
        pspDebugScreenPrintf("IR TX: 0x%08X\n", code);
        return;
    }
    
    if (strstr(response, "IR:RX:") != NULL) {
        // IR code received (when in RX mode)
        // Format: IR:RX:0x1A2B3C4D
        unsigned int code = 0;
        sscanf(response, "IR:RX:0x%x", &code);
        pspDebugScreenPrintf("IR RX: 0x%08X\n", code);
        return;
    }
    
    if (strstr(response, "IR:MODE:") != NULL) {
        // IR mode change response
        // Format: IR:MODE:TX or IR:MODE:RX
        unsigned char mode = (strstr(response, "TX") != NULL) ? 0 : 1;
        ir_state.ir_mode = mode;
        pspDebugScreenPrintf("IR Mode: %s\n", mode ? "RX (Receiver)" : "TX (Transmitter)");
        return;
    }
    
    if (strstr(response, "IR:OK") != NULL) {
        // Generic IR command OK
        pspDebugScreenPrintf("IR: OK\n");
        return;
    }
    
    if (strstr(response, "IR:ERR") != NULL) {
        // IR command error
        pspDebugScreenPrintf("IR: ERROR\n");
        return;
    }
}

// --- UART READ FUNCTION WITH AT COMMAND BUFFERING ---
// Reads characters from UART and buffers complete AT responses
void process_uart_at_command() {
    int ch = pspDebugSioGetchar();
    
    if (ch != -1) {
        unsigned char c = (unsigned char)ch;
        
        // Buffer the character
        if (at_buffer_index < AT_BUFFER_SIZE - 1) {
            at_response_buffer[at_buffer_index++] = c;
        }
        
        // Look for complete AT response (ends with newline or carriage return)
        if (c == '\n' || c == '\r') {
            at_response_buffer[at_buffer_index] = '\0';
            
            // Parse the complete response if it's not empty
            if (at_buffer_index > 1) {
                parse_at_response(at_response_buffer);
            }
            
            // Reset buffer for next message
            at_buffer_index = 0;
            memset(at_response_buffer, 0, AT_BUFFER_SIZE);
        }
    }
}

// --- UART COMMAND SENDER HELPER ---
// Send AT commands to Bluetooth module via UART
void send_at_command(const char *command) {
    const char *p = command;
    while (*p) {
        pspDebugSioPutchar(*p++);
    }
    pspDebugSioPutchar('\n'); // Terminate with newline
}

// --- IR COMMAND SENDER ---
// Send IR control commands to ESP32 module via UART
void send_ir_command(const char *command) {
    const char *p = command;
    while (*p) {
        pspDebugSioPutchar(*p++);
    }
    pspDebugSioPutchar('\n'); // Terminate with newline
}


// --- MANAGER THREAD: The Orchestrator (High Priority: 0x10) ---
int uart_manager_thread(SceSize args, void *argp) {
    // Create general app lock semaphore
    // Initial value = 1 (available), only ONE app can hold this
    app_lock_semaphore = sceKernelCreateSema("app_lock", 0, 1, 1, NULL);
    if (app_lock_semaphore < 0) {
        pspDebugScreenPrintf("ERROR: Failed to create app lock semaphore\n");
        return 1;
    }
    
    pspDebugSioInit();
    pspDebugSioSetBaud(9600);

    while(1) {
        // A. Process incoming AT command responses from UART
        process_uart_at_command();

        // B. Process outgoing AT commands from Worker Plugins (Pipe -> UART)
        // Plugins send AT command strings via message pipe
        unsigned char at_cmd[256];
        int recv_size = sceKernelReceiveMsgPipe(cmd_pipe_out, at_cmd, 256, PSP_MSGPIPE_NOWAIT, NULL, NULL);
        
        if (recv_size > 0) {
            // Null-terminate the received string
            at_cmd[recv_size] = '\0';
            
            // Send AT command to Bluetooth module via UART
            send_at_command((const char*)at_cmd);
            pspDebugScreenPrintf("Sent AT Cmd: %s\n", at_cmd);
        }

        // C. Process outgoing IR commands from IR Plugin (IR Pipe -> UART)
        // IR plugin sends IR control commands
        unsigned char ir_cmd[256];
        recv_size = sceKernelReceiveMsgPipe(ir_cmd_pipe, ir_cmd, 256, PSP_MSGPIPE_NOWAIT, NULL, NULL);
        
        if (recv_size > 0) {
            // Null-terminate the received string
            ir_cmd[recv_size] = '\0';
            
            // Send IR command to ESP32 module via UART
            send_ir_command((const char*)ir_cmd);
            pspDebugScreenPrintf("Sent IR Cmd: %s\n", ir_cmd);
        }
        
        sceKernelDelayThread(0); // Yield to other threads
    }
    
    // Cleanup
    if (app_lock_semaphore >= 0) {
        sceKernelDeleteSema(app_lock_semaphore);
    }
    
    return 0;
}

// --- MODULE START ---
int module_start(SceSize args, void *argp) {
    cmd_pipe_out = sceKernelCreateMsgPipe("SioCmdPipe", 1, 0, 256, NULL);
    ir_cmd_pipe = sceKernelCreateMsgPipe("IrCmdPipe", 1, 0, 256, NULL);
    SceUID thid = sceKernelCreateThread("UART_Manager", uart_manager_thread, 0x10, 0x1000, 0, NULL);
    sceKernelStartThread(thid, 0, NULL);
    return 0;
}

// --- MODULE STOP ---
int module_stop(SceSize args, void *argp) {
    if (cmd_pipe_out >= 0) {
        sceKernelDeleteMsgPipe(cmd_pipe_out);
    }
    if (ir_cmd_pipe >= 0) {
        sceKernelDeleteMsgPipe(ir_cmd_pipe);
    }
    return 0;
}