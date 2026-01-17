/*
    * A uart controller for PSP that allow multiple connection to this UART manager.
    Can be used to communicate with everything that uses uart.
    REFACTORED: Pure UART relay - plugins handle their own command formatting and parsing
*/


#include <pspsdk.h>
#include <pspkernel.h>
#include <pspdebug.h>
#include <psppower.h>
#include <systemctrl.h>
#include <string.h>
#include <stdio.h>

PSP_MODULE_INFO("UART_Manager", 0x1000, 1, 1); // Kernel Mode
PSP_MAIN_THREAD_ATTR(0x10); // High Priority Thread

// --- GLOBAL STATE VARIABLES ---
static SceUID cmd_pipe_out = -1;
static SceUID ir_cmd_pipe = -1;
static SceUID app_lock_semaphore = -1;          // General semaphore - prevents multiple apps from running

// --- COMMUNICATION BUFFERS FOR UART ---
#define UART_RX_BUFFER_SIZE 256
static unsigned char uart_rx_buffer[UART_RX_BUFFER_SIZE];
static int uart_rx_index = 0;


// --- UART RESPONSE RELAY ---
// Simply buffer UART data and consume it
// NOTE: In the future, this could relay responses back to requesting plugin via a response pipe
void process_uart_data(void) {
    int ch = pspDebugSioGetchar();
    
    if (ch != -1) {
        unsigned char c = (unsigned char)ch;
        
        // Buffer the character
        if (uart_rx_index < UART_RX_BUFFER_SIZE - 1) {
            uart_rx_buffer[uart_rx_index++] = c;
        }
        
        // Look for complete response (ends with newline)
        if (c == '\n') {
            uart_rx_buffer[uart_rx_index] = '\0';
            
            // TODO: In future, relay response back to the requesting plugin
            // For now, just consume the data
            // pspDebugScreenPrintf("UART RX: %s\n", uart_rx_buffer);
            
            // Reset buffer for next message
            uart_rx_index = 0;
            memset(uart_rx_buffer, 0, UART_RX_BUFFER_SIZE);
        }
    }
}


// --- GENERIC UART COMMAND SENDER ---
// Send any command to UART (AT commands, IR commands, etc.)
// Plugins are responsible for formatting their commands correctly
void send_uart_command(const char *command) {
    if (command == NULL) return;
    
    const char *p = command;
    while (*p) {
        pspDebugSioPutchar(*p++);
    }
    pspDebugSioPutchar('\n'); // Terminate with newline
}


// --- MANAGER THREAD: Pure UART Relay (High Priority: 0x10) ---
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
        // A. Process incoming UART responses (from devices)
        process_uart_data();

        // B. Process outgoing commands from BT Plugin (Pipe -> UART)
        unsigned char cmd_msg[256];
        int recv_size = sceKernelReceiveMsgPipe(cmd_pipe_out, cmd_msg, 256, PSP_MSGPIPE_NOWAIT, NULL, NULL);
        
        if (recv_size > 0) {
            // Null-terminate the received string
            cmd_msg[recv_size] = '\0';
            
            // Send command to device via UART (plugin is responsible for command format)
            send_uart_command((const char*)cmd_msg);
        }

        // C. Process outgoing commands from IR Plugin (Pipe -> UART)
        unsigned char ir_msg[256];
        recv_size = sceKernelReceiveMsgPipe(ir_cmd_pipe, ir_msg, 256, PSP_MSGPIPE_NOWAIT, NULL, NULL);
        
        if (recv_size > 0) {
            // Null-terminate the received string
            ir_msg[recv_size] = '\0';
            
            // Send command to device via UART (plugin is responsible for command format)
            send_uart_command((const char*)ir_msg);
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
