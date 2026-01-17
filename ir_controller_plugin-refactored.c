/*
==========================================================
ESP32 IR CONTROLLER PLUGIN v1.2
C Implementation for PSP User Module
==========================================================

REFACTORED: Handles its own IR command formatting and response parsing.
The UART Manager is now a pure relay - this plugin constructs IR commands
and parses responses independently.

Features:
- Transmit IR codes (NEC, RC5, RC6 protocols)
- Receive IR codes (when in RX mode)
- Switch between TX (transmitter) and RX (receiver) modes
- Display last received/transmitted codes
- Save received codes to protocol files
- IR code history tracking
- File storage: ms0:/game/ircontroller/
*/

#include <pspsdk.h>
#include <pspkernel.h>
#include <pspdebug.h>
#include <pspctrl.h>
#include <pspge.h>
#include <pspgum.h>
#include <pspgu.h>
#include <pspiofilemgr.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

PSP_MODULE_INFO("IRController", 0x0000, 1, 1); // User Module
PSP_MAIN_THREAD_ATTR(PSP_THREAD_ATTR_USER);

// --- GRAPHICS CONSTANTS ---
#define SCREEN_WIDTH    480
#define SCREEN_HEIGHT   272
#define LINE_HEIGHT     12

// --- FILE PATHS ---
#define IR_DATA_DIR "ms0:/game/ircontroller/"
#define IR_CODE_FILENAME "codes.txt"
#define IR_CONFIG_FILE "ms0:/game/ircontroller/config.txt"

// --- IR MODE DEFINITIONS ---
#define IR_MODE_TX 0
#define IR_MODE_RX 1

// --- IR PROTOCOL DEFINITIONS ---
#define MAX_PROTOCOLS 16
#define IR_PROTOCOL_NEC 0
#define IR_PROTOCOL_RC5 1
#define IR_PROTOCOL_RC6 2
#define IR_PROTOCOL_SONY 3

// --- IR PROTOCOL PARAMETERS ---
typedef struct {
    char name[32];
    uint16_t carrier_freq;      // Carrier frequency (e.g., 38000 Hz for NEC)
    uint8_t bits;               // Number of bits (e.g., 32 for NEC)
} IR_Protocol_Params;

// Standard protocols with their parameters
const IR_Protocol_Params protocol_params[] = {
    {"NEC", 38000, 32},
    {"RC5", 36000, 12},
    {"RC6", 36000, 20},
    {"SONY", 40000, 12}
};

typedef struct {
    char name[32];                      // Protocol name (e.g., "NEC", "RC5")
    unsigned int *codes;                // Dynamic array of codes
    char **code_names;                  // Dynamic array of code names
    int code_count;
    int code_capacity;                  // Allocated capacity
    char filename[64];                  // Full path to protocol file
    uint16_t carrier_freq;              // Carrier frequency for this protocol
    uint8_t bits;                       // Number of bits for this protocol
} IR_Protocol;

// --- ESP32 IR STATE STRUCTURE ---
typedef struct {
    unsigned char ir_enabled;          // 1=enabled, 0=disabled
    unsigned char ir_mode;             // 0=TX (transmitter), 1=RX (receiver)
    unsigned int last_ir_code;         // Last transmitted/received IR code
    unsigned char ir_protocol;         // Current protocol index
    unsigned int ir_frequency;         // Carrier frequency in Hz
} ESP32_IR_State;

static ESP32_IR_State ir_state = {0};

// --- STATE STRUCTURE ---
typedef struct {
    SceUID ir_pipe;
    
    unsigned char ir_mode;      // 0=TX, 1=RX
    int current_protocol_idx;   // Index in protocol list
    unsigned int current_code;  // Current/last code
    
    int menu_state;             // 0=main, 1=receive, 2=protocol, 3=history
    int selected_preset;        // Selected code in current protocol
    int selected_protocol;      // Selected protocol in protocol menu
    int current_page;           // Current page for pagination in menu 0
    
    char status_msg[64];
    unsigned int last_update_time;
    
    // Keyboard state
    int keyboard_active;        // 0=inactive, 1=active
    char keyboard_input[96];    // Current input buffer
    int keyboard_cursor_x;      // Keyboard cursor position X
    int keyboard_cursor_y;      // Keyboard cursor position Y
} IR_Controller_State;

static IR_Controller_State state = {0};

// --- PRESET IR CODES ---
typedef struct {
    char name[32];
    unsigned int code;
    unsigned char protocol;
} IR_Preset;

static IR_Preset presets[] = {
    {"Power", 0x00FF40BF, IR_PROTOCOL_NEC},
    {"Volume +", 0x00FF18E7, IR_PROTOCOL_NEC},
    {"Volume -", 0x00FF10EF, IR_PROTOCOL_NEC},
    {"Ch +", 0x00FF50AF, IR_PROTOCOL_NEC},
    {"Ch -", 0x00FF48B7, IR_PROTOCOL_NEC},
    {"Mute", 0x00FF20DF, IR_PROTOCOL_NEC},
    {"Menu", 0x00FFE01F, IR_PROTOCOL_NEC},
    {"OK", 0x00FF906F, IR_PROTOCOL_NEC},
};

#define PRESET_COUNT (sizeof(presets) / sizeof(presets[0]))

// Dynamic protocol storage
static IR_Protocol protocols[MAX_PROTOCOLS] = {0};
static int protocol_count = 0;

// --- IR CODE HISTORY ---
#define HISTORY_SIZE 20
typedef struct {
    unsigned int code;
    unsigned char protocol;
    unsigned char mode;         // 0=TX, 1=RX
    unsigned int timestamp;
} IR_History_Entry;

static IR_History_Entry history[HISTORY_SIZE] = {0};
static int history_count = 0;
static int history_index = 0;

// --- IR RESPONSE BUFFER ---
#define IR_RESPONSE_BUFFER_SIZE 128
static char ir_response_buffer[IR_RESPONSE_BUFFER_SIZE];
static int ir_response_index = 0;

// --- IR RESPONSE PARSER ---
// This plugin now parses responses from IR commands
void parse_ir_response(const char *response) {
    if (strstr(response, "IR:TX:") != NULL) {
        // IR transmission acknowledgment
        // Format: IR:TX:0x1A2B3C4D (code transmitted)
        unsigned int code = 0;
        sscanf(response, "IR:TX:0x%x", &code);
        ir_state.last_ir_code = code;
        snprintf(state.status_msg, sizeof(state.status_msg), "IR TX: 0x%08X", code);
        return;
    }
    
    if (strstr(response, "IR:RX:") != NULL) {
        // IR code received (when in RX mode)
        // Format: IR:RX:0x1A2B3C4D
        unsigned int code = 0;
        sscanf(response, "IR:RX:0x%x", &code);
        state.current_code = code;
        ir_state.last_ir_code = code;
        snprintf(state.status_msg, sizeof(state.status_msg), "IR RX: 0x%08X", code);
        return;
    }
    
    if (strstr(response, "IR:MODE:") != NULL) {
        // IR mode change response
        // Format: IR:MODE:TX or IR:MODE:RX
        unsigned char mode = (strstr(response, "TX") != NULL) ? 0 : 1;
        ir_state.ir_mode = mode;
        snprintf(state.status_msg, sizeof(state.status_msg), "IR Mode: %s", 
                 mode ? "RX (Receiver)" : "TX (Transmitter)");
        return;
    }
    
    if (strstr(response, "IR:OK") != NULL) {
        // Generic IR command OK
        snprintf(state.status_msg, sizeof(state.status_msg), "IR: OK");
        return;
    }
    
    if (strstr(response, "IR:ERR") != NULL) {
        // IR command error
        snprintf(state.status_msg, sizeof(state.status_msg), "IR: ERROR");
        return;
    }
}

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

const char* get_protocol_name(int protocol_idx) {
    if (protocol_idx >= 0 && protocol_idx < protocol_count) {
        return protocols[protocol_idx].name;
    }
    return "UNKNOWN";
}

// --- UART APP LOCK ---
// General semaphore: if already acquired by another app, we can't start
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

// --- IR COMMAND SENDING ---
void send_ir_command(const char *command) {
    if (state.ir_pipe < 0) return;
    
    unsigned int cmd_len = strlen(command);
    sceKernelSendMsgPipe(state.ir_pipe, (void*)command, cmd_len, 
                         PSP_MSGPIPE_NOWAIT, NULL, NULL);
    
    snprintf(state.status_msg, sizeof(state.status_msg), "Sent: %s", command);
    state.last_update_time = sceKernelGetSystemTimeLow();
}

void transmit_ir_code(unsigned int code) {
    // Send IR transmission command - plugin is responsible for command format
    char ir_cmd[64];
    snprintf(ir_cmd, sizeof(ir_cmd), "AT+IRTX=%08X", code);
    send_ir_command(ir_cmd);
    state.current_code = code;
    
    // Add to history
    if (history_count < HISTORY_SIZE) {
        history[history_index].code = code;
        history[history_index].protocol = state.ir_protocol;
        history[history_index].mode = IR_MODE_TX;
        history[history_index].timestamp = sceKernelGetSystemTimeLow();
        history_index = (history_index + 1) % HISTORY_SIZE;
        if (history_count < HISTORY_SIZE) history_count++;
    }
}

// --- FILE I/O FUNCTIONS ---
void create_data_directory(void) {
    // Create ms0:/game/ircontroller/ if it doesn't exist
    sceIoMkdir(IR_DATA_DIR, 0777);
}

void scan_protocols(void) {
    SceUID dir = sceIoDopen(IR_DATA_DIR);
    if (dir >= 0) {
        SceIoDirent entry;
        protocol_count = 0;
        
        while (sceIoDread(dir, &entry) > 0 && protocol_count < MAX_PROTOCOLS) {
            if (!FIO_S_ISDIR(entry.d_stat.st_mode)) {
                char *filename = entry.d_name;
                
                // Check if file ends with "_codes.txt"
                if (strstr(filename, "_codes.txt") != NULL) {
                    // Extract protocol name from filename
                    char proto_name[32] = {0};
                    int len = strlen(filename) - strlen("_codes.txt");
                    if (len > 0 && len < sizeof(proto_name)) {
                        strncpy(proto_name, filename, len);
                        proto_name[len] = '\0';
                        
                        // Convert to uppercase
                        for (int i = 0; proto_name[i]; i++) {
                            proto_name[i] = (proto_name[i] >= 'a' && proto_name[i] <= 'z') ? 
                                           proto_name[i] - 32 : proto_name[i];
                        }
                        
                        // Add to protocol list
                        strncpy(protocols[protocol_count].name, proto_name, 31);
                        snprintf(protocols[protocol_count].filename, sizeof(protocols[protocol_count].filename),
                                "%s%s", IR_DATA_DIR, filename);
                        
                        // Load codes from file (includes parameters from first two lines)
                        load_protocol_codes(protocol_count);
                        
                        protocol_count++;
                    }
                }
            }
        }
        sceIoDclose(dir);
    }
}

void load_protocol_codes(int protocol_idx);  // Forward declaration

void save_code_to_protocol(unsigned int code, int protocol_idx) {
    // Save code with default name
    save_code_to_protocol_with_name(code, protocol_idx, "");
}

void save_code_to_protocol_with_name(unsigned int code, int protocol_idx, const char *code_name);  // Forward declaration

void load_protocol_codes(int protocol_idx) {
    if (protocol_idx < 0 || protocol_idx >= protocol_count) return;
    
    SceUID fd = sceIoOpen(protocols[protocol_idx].filename, PSP_O_RDONLY, 0);
    protocols[protocol_idx].code_count = 0;
    protocols[protocol_idx].code_capacity = 0;
    
    // Initialize with default parameters
    protocols[protocol_idx].carrier_freq = 38000;
    protocols[protocol_idx].bits = 32;
    
    if (fd >= 0) {
        char line[128];
        int bytes_read;
        int line_num = 0;
        
        while ((bytes_read = sceIoRead(fd, line, sizeof(line) - 1)) > 0) {
            line[bytes_read] = '\0';
            
            // Remove trailing newline/carriage return
            while (bytes_read > 0 && (line[bytes_read - 1] == '\n' || line[bytes_read - 1] == '\r')) {
                line[--bytes_read] = '\0';
            }
            
            // First two lines are parameters: carrier_freq and bits
            if (line_num == 0) {
                // First line: carrier frequency
                uint16_t carrier = 0;
                if (sscanf(line, "%hu", &carrier) == 1) {
                    protocols[protocol_idx].carrier_freq = carrier;
                }
                line_num++;
                continue;
            } else if (line_num == 1) {
                // Second line: bit count
                uint8_t bits = 0;
                if (sscanf(line, "%hhu", &bits) == 1) {
                    protocols[protocol_idx].bits = bits;
                }
                line_num++;
                continue;
            }
            
            // Remaining lines are: Name -> 0xCODE
            unsigned int code = 0;
            char code_name[96] = {0};
            
            // Try to parse: Name -> 0xCODE
            char *arrow = strstr(line, "->");
            if (arrow != NULL) {
                // Extract name (before arrow)
                int name_len = arrow - line;
                if (name_len > 0 && name_len < sizeof(code_name)) {
                    strncpy(code_name, line, name_len);
                    code_name[name_len] = '\0';
                    
                    // Trim whitespace from name
                    while (name_len > 0 && code_name[name_len - 1] == ' ') {
                        code_name[--name_len] = '\0';
                    }
                }
                
                // Extract code (after arrow)
                if (sscanf(arrow + 2, "%x", &code) == 1) {
                    // Valid code found
                } else {
                    continue; // Skip malformed lines
                }
            } else {
                // No arrow, just try to parse as hex code
                if (sscanf(line, "%x", &code) != 1) {
                    continue;
                }
            }
            
            // Expand arrays if needed
            if (protocols[protocol_idx].code_count >= protocols[protocol_idx].code_capacity) {
                int new_capacity = (protocols[protocol_idx].code_capacity == 0) ? 16 : 
                                  protocols[protocol_idx].code_capacity * 2;
                
                // Expand codes array
                unsigned int *new_codes = (unsigned int *)realloc(protocols[protocol_idx].codes, 
                                                                  new_capacity * sizeof(unsigned int));
                if (new_codes == NULL) break;
                protocols[protocol_idx].codes = new_codes;
                
                // Expand names array
                char **new_names = (char **)realloc(protocols[protocol_idx].code_names, 
                                                   new_capacity * sizeof(char *));
                if (new_names == NULL) break;
                protocols[protocol_idx].code_names = new_names;
                
                protocols[protocol_idx].code_capacity = new_capacity;
            }
            
            // Store code
            protocols[protocol_idx].codes[protocols[protocol_idx].code_count] = code;
            
            // Store name (allocate and copy)
            int name_len = strlen(code_name);
            protocols[protocol_idx].code_names[protocols[protocol_idx].code_count] = 
                (char *)malloc(name_len + 1);
            if (protocols[protocol_idx].code_names[protocols[protocol_idx].code_count] != NULL) {
                strcpy(protocols[protocol_idx].code_names[protocols[protocol_idx].code_count], code_name);
            }
            
            protocols[protocol_idx].code_count++;
        }
        sceIoClose(fd);
    }
}

void save_code_to_protocol_with_name(unsigned int code, int protocol_idx, const char *code_name) {
    if (protocol_idx < 0 || protocol_idx >= protocol_count) return;
    
    // Expand arrays if needed
    if (protocols[protocol_idx].code_count >= protocols[protocol_idx].code_capacity) {
        int new_capacity = (protocols[protocol_idx].code_capacity == 0) ? 16 : 
                          protocols[protocol_idx].code_capacity * 2;
        
        // Expand codes array
        unsigned int *new_codes = (unsigned int *)realloc(protocols[protocol_idx].codes, 
                                                          new_capacity * sizeof(unsigned int));
        if (new_codes == NULL) {
            snprintf(state.status_msg, sizeof(state.status_msg), "Memory allocation failed!");
            return;
        }
        protocols[protocol_idx].codes = new_codes;
        
        // Expand names array
        char **new_names = (char **)realloc(protocols[protocol_idx].code_names, 
                                           new_capacity * sizeof(char *));
        if (new_names == NULL) {
            snprintf(state.status_msg, sizeof(state.status_msg), "Memory allocation failed!");
            return;
        }
        protocols[protocol_idx].code_names = new_names;
        
        protocols[protocol_idx].code_capacity = new_capacity;
    }
    
    // Add code to memory
    protocols[protocol_idx].codes[protocols[protocol_idx].code_count] = code;
    
    // Generate name for received code
    char final_name[96];
    if (code_name != NULL && strlen(code_name) > 0) {
        strcpy(final_name, code_name);
    } else {
        snprintf(final_name, sizeof(final_name), "Code_%d", protocols[protocol_idx].code_count + 1);
    }
    
    protocols[protocol_idx].code_names[protocols[protocol_idx].code_count] = (char *)malloc(strlen(final_name) + 1);
    if (protocols[protocol_idx].code_names[protocols[protocol_idx].code_count] != NULL) {
        strcpy(protocols[protocol_idx].code_names[protocols[protocol_idx].code_count], final_name);
    }
    
    protocols[protocol_idx].code_count++;
    
    // Save to file
    SceUID fd = sceIoOpen(protocols[protocol_idx].filename, PSP_O_WRONLY | PSP_O_CREAT | PSP_O_APPEND, 0777);
    
    if (fd >= 0) {
        char line[128];
        snprintf(line, sizeof(line), "%s -> 0x%08X\n", final_name, code);
        sceIoWrite(fd, line, strlen(line));
        sceIoClose(fd);
        
        snprintf(state.status_msg, sizeof(state.status_msg), "Saved '%s' to %s", 
                 final_name, protocols[protocol_idx].name);
    } else {
        snprintf(state.status_msg, sizeof(state.status_msg), "Error saving file!");
    }
}

void save_default_protocol(int protocol_idx) {
    SceUID fd = sceIoOpen(IR_CONFIG_FILE, PSP_O_WRONLY | PSP_O_CREAT | PSP_O_TRUNC, 0777);
    
    if (fd >= 0) {
        char line[32];
        snprintf(line, sizeof(line), "%d\n", protocol_idx);
        sceIoWrite(fd, line, strlen(line));
        sceIoClose(fd);
    }
}

void load_default_protocol(void) {
    SceUID fd = sceIoOpen(IR_CONFIG_FILE, PSP_O_RDONLY, 0);
    
    state.current_protocol_idx = 0; // Default to first protocol
    
    if (fd >= 0) {
        char line[32];
        int bytes_read = sceIoRead(fd, line, sizeof(line) - 1);
        if (bytes_read > 0) {
            line[bytes_read] = '\0';
            int idx = 0;
            if (sscanf(line, "%d", &idx) == 1 && idx >= 0 && idx < protocol_count) {
                state.current_protocol_idx = idx;
            }
        }
        sceIoClose(fd);
    }
}

void set_ir_mode_tx(void) {
    send_ir_command("IR:MODE:TX");
    state.ir_mode = IR_MODE_TX;
    snprintf(state.status_msg, sizeof(state.status_msg), "Mode: TX (Transmitter)");
}

void set_ir_mode_rx(void) {
    send_ir_command("IR:MODE:RX");
    state.ir_mode = IR_MODE_RX;
    snprintf(state.status_msg, sizeof(state.status_msg), "Mode: RX (Receiver)");
}

void send_protocol_parameters(int protocol_idx) {
    // Send protocol parameters to ESP32
    // Command: AT+IRPROTO=<name>,<carrier_freq>,<bits>
    if (protocol_idx >= 0 && protocol_idx < protocol_count) {
        IR_Protocol *proto = &protocols[protocol_idx];
        
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "AT+IRPROTO=%s,%d,%d", 
                 proto->name, proto->carrier_freq, proto->bits);
        
        send_ir_command(cmd);
    }
}

void set_ir_protocol(int protocol_idx) {
    if (protocol_idx >= 0 && protocol_idx < protocol_count) {
        state.current_protocol_idx = protocol_idx;
        save_default_protocol(protocol_idx);
        
        // Send protocol parameters to ESP32
        send_protocol_parameters(protocol_idx);
        
        snprintf(state.status_msg, sizeof(state.status_msg), "Protocol: %s", protocols[protocol_idx].name);
    }
}

// --- KEYBOARD INPUT ---
const char keyboard_layout[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 ";
#define KEYBOARD_WIDTH 10
#define KEYBOARD_HEIGHT 4
#define KEYBOARD_COLS 10

void render_keyboard(void) {
    pspDebugScreenClear();
    debug_print(10, 10, "Enter Name for IR Code", 0xFFFFFF00);
    debug_print(10, 30, state.keyboard_input, 0xFF00FF00);
    debug_print(10, 50, "--------------------------------------------", 0xFF666666);
    
    int keyboard_size = strlen(keyboard_layout);
    int row = 0;
    
    for (int i = 0; i < keyboard_size; i++) {
        int col = i % KEYBOARD_COLS;
        if (col == 0 && i > 0) row++;
        
        int x = 20 + col * 40;
        int y = 70 + row * 25;
        
        char key_str[2];
        key_str[0] = keyboard_layout[i];
        key_str[1] = '\0';
        
        unsigned int color = 0xFFCCCCCC;
        if (state.keyboard_cursor_x == col && state.keyboard_cursor_y == row) {
            color = 0xFF00FF00;
            debug_print(x - 10, y, "[ " , 0xFF00FF00);
            debug_print(x + 10, y, " ]", 0xFF00FF00);
        }
        
        debug_print(x, y, key_str, color);
    }
    
    debug_print(10, 220, "UP/DOWN/LEFT/RIGHT: Navigate  |   TRIANGLE: Backspace  |  CROSS: Select  |  SQUARE: Done", 0xFF888888);
}

void handle_keyboard_input(SceCtrlData *pad) {
    int keyboard_size = strlen(keyboard_layout);
    int max_rows = (keyboard_size + KEYBOARD_COLS - 1) / KEYBOARD_COLS;
    
    if (pad->Buttons & PSP_CTRL_UP) {
        state.keyboard_cursor_y = (state.keyboard_cursor_y > 0) ? state.keyboard_cursor_y - 1 : max_rows - 1;
        sceKernelDelayThread(150000);
    }
    if (pad->Buttons & PSP_CTRL_DOWN) {
        state.keyboard_cursor_y = (state.keyboard_cursor_y < max_rows - 1) ? state.keyboard_cursor_y + 1 : 0;
        sceKernelDelayThread(150000);
    }
    if (pad->Buttons & PSP_CTRL_LEFT) {
        state.keyboard_cursor_x = (state.keyboard_cursor_x > 0) ? state.keyboard_cursor_x - 1 : KEYBOARD_COLS - 1;
        sceKernelDelayThread(150000);
    }
    if (pad->Buttons & PSP_CTRL_RIGHT) {
        state.keyboard_cursor_x = (state.keyboard_cursor_x < KEYBOARD_COLS - 1) ? state.keyboard_cursor_x + 1 : 0;
        sceKernelDelayThread(150000);
    }
    
    if (pad->Buttons & PSP_CTRL_CROSS) {
        // Select character
        int index = state.keyboard_cursor_y * KEYBOARD_COLS + state.keyboard_cursor_x;
        if (index < keyboard_size) {
            int input_len = strlen(state.keyboard_input);
            if (input_len < sizeof(state.keyboard_input) - 1) {
                state.keyboard_input[input_len] = keyboard_layout[index];
                state.keyboard_input[input_len + 1] = '\0';
            }
        }
        sceKernelDelayThread(200000);
    }
    
    if (pad->Buttons & PSP_CTRL_TRIANGLE) {
        // Backspace
        int input_len = strlen(state.keyboard_input);
        if (input_len > 0) {
            state.keyboard_input[input_len - 1] = '\0';
        }
        sceKernelDelayThread(200000);
    }
    
    if (pad->Buttons & PSP_CTRL_SQUARE) {
        // Done - save code with name
        if (state.current_code != 0) {
            save_code_to_protocol_with_name(state.current_code, state.current_protocol_idx, state.keyboard_input);
        }
        state.keyboard_active = 0;
        memset(state.keyboard_input, 0, sizeof(state.keyboard_input));
        state.keyboard_cursor_x = 0;
        state.keyboard_cursor_y = 0;
        sceKernelDelayThread(300000);
    }
}

// --- INPUT HANDLING ---
void handle_input(SceCtrlData *pad) {
    if (pad->Buttons & PSP_CTRL_HOME) {
        // Exit application
        sceKernelExitGame();
    }
    
    if (pad->Buttons & PSP_CTRL_LTRIGGER) {
        // Previous menu
        state.menu_state = (state.menu_state > 0) ? (state.menu_state - 1) : 3;
        sceKernelDelayThread(200000); // Debounce
    }
    
    if (pad->Buttons & PSP_CTRL_RTRIGGER) {
        // Next menu
        state.menu_state = (state.menu_state + 1) % 4;
        sceKernelDelayThread(200000); // Debounce
    }
    
    if (state.menu_state == 0) {
        // Main menu - load and transmit codes from current protocol
        int current_code_count = protocols[state.current_protocol_idx].code_count;
        int codes_per_page = 10;
        int total_pages = (current_code_count > 0) ? (current_code_count + codes_per_page - 1) / codes_per_page : 1;
        int page_start = state.current_page * codes_per_page;
        int page_end = (state.current_page + 1) * codes_per_page;
        if (page_end > current_code_count) page_end = current_code_count;
        
        if (pad->Buttons & PSP_CTRL_UP && state.selected_preset > 0) {
            state.selected_preset--;
            sceKernelDelayThread(150000);
        }
        if (pad->Buttons & PSP_CTRL_DOWN && state.selected_preset < page_end - page_start - 1) {
            state.selected_preset++;
            sceKernelDelayThread(150000);
        }
        
        if (pad->Buttons & PSP_CTRL_LEFT && state.current_page > 0) {
            state.current_page--;
            state.selected_preset = 0;
            sceKernelDelayThread(200000);
        }
        if (pad->Buttons & PSP_CTRL_RIGHT && state.current_page < total_pages - 1) {
            state.current_page++;
            state.selected_preset = 0;
            sceKernelDelayThread(200000);
        }
        
        if (pad->Buttons & PSP_CTRL_CROSS && current_code_count > 0) {
            // Transmit code from current protocol
            unsigned int code = protocols[state.current_protocol_idx].codes[page_start + state.selected_preset];
            transmit_ir_code(code);
            sceKernelDelayThread(300000);
        }
    }
    
    if (state.menu_state == 1) {
        // Receive mode menu
        if (state.keyboard_active) {
            // Handle keyboard input
            handle_keyboard_input(pad);
        } else {
            // Normal menu input
            if (pad->Buttons & PSP_CTRL_CROSS) {
                // Enter receive mode
                set_ir_mode_rx();
                sceKernelDelayThread(300000);
            }
            
            if (pad->Buttons & PSP_CTRL_SQUARE) {
                // Exit receive mode (back to TX)
                set_ir_mode_tx();
                sceKernelDelayThread(300000);
            }
            
            if (pad->Buttons & PSP_CTRL_LTRIGGER && state.ir_mode == IR_MODE_RX && state.current_code != 0) {
                // Activate keyboard to name the code
                state.keyboard_active = 1;
                memset(state.keyboard_input, 0, sizeof(state.keyboard_input));
                state.keyboard_cursor_x = 0;
                state.keyboard_cursor_y = 0;
                sceKernelDelayThread(300000);
            }
        }
    }
    
    if (state.menu_state == 2) {
        // Protocol selection menu
        if (pad->Buttons & PSP_CTRL_UP && state.selected_protocol > 0) {
            state.selected_protocol--;
            sceKernelDelayThread(150000);
        }
        if (pad->Buttons & PSP_CTRL_DOWN && state.selected_protocol < protocol_count - 1) {
            state.selected_protocol++;
            sceKernelDelayThread(150000);
        }
        
        if (pad->Buttons & PSP_CTRL_CROSS) {
            // Select protocol
            set_ir_protocol(state.selected_protocol);
            sceKernelDelayThread(300000);
        }
    }
    
    if (state.menu_state == 3) {
        // History view
        if (pad->Buttons & PSP_CTRL_UP || pad->Buttons & PSP_CTRL_DOWN) {
            // Scroll through history (can be enhanced)
            sceKernelDelayThread(150000);
        }
    }
}

// --- DISPLAY RENDERING ---
void render_main_menu(void) {
    pspDebugScreenClear();
    
    debug_print(10, 10, "ESP32 IR Controller v1.2", 0xFFFFFF00);
    
    // Mode indicator
    const char *mode_str = (state.ir_mode == IR_MODE_TX) ? "TX (Transmitter)" : "RX (Receiver)";
    unsigned int mode_color = (state.ir_mode == IR_MODE_TX) ? 0xFF00FF00 : 0xFFFF6600;
    debug_print(10, 25, mode_str, mode_color);
    
    // Protocol indicator
    char protocol_str[64];
    snprintf(protocol_str, sizeof(protocol_str), "Protocol: %s", get_protocol_name(state.current_protocol_idx));
    debug_print(200, 25, protocol_str, 0xFF00FFFF);
    
    // Status message
    debug_print(10, 40, state.status_msg, 0xFFFFFFFF);
    
    debug_print(10, 60, "================================", 0xFF666666);
    
    // Current code display
    if (state.current_code != 0) {
        char code_str[64];
        snprintf(code_str, sizeof(code_str), "Last Code: 0x%08X", state.current_code);
        debug_print(10, 75, code_str, 0xFF00FF00);
    }
    
    // Display codes from current protocol with pagination
    int current_code_count = protocols[state.current_protocol_idx].code_count;
    int codes_per_page = 10;
    int total_pages = (current_code_count > 0) ? (current_code_count + codes_per_page - 1) / codes_per_page : 1;
    int page_start = state.current_page * codes_per_page;
    int page_end = (state.current_page + 1) * codes_per_page;
    if (page_end > current_code_count) page_end = current_code_count;
    
    if (current_code_count > 0) {
        debug_print(10, 90, "Available IR Codes:", 0xFF00FFFF);
        
        char page_info[64];
        snprintf(page_info, sizeof(page_info), "[Page %d/%d] (%d codes total)", 
                 state.current_page + 1, total_pages, current_code_count);
        debug_print(350, 90, page_info, 0xFFFFCC00);
        
        for (int i = page_start; i < page_end && i < current_code_count; i++) {
            char line[96];
            char prefix[4] = "  ";
            
            if ((i - page_start) == state.selected_preset) {
                strcpy(prefix, "> ");
            }
            
            unsigned int code = protocols[state.current_protocol_idx].codes[i];
            char *code_name = protocols[state.current_protocol_idx].code_names[i];
            
            if (code_name && strlen(code_name) > 0) {
                snprintf(line, sizeof(line), "%s[%d] %s (0x%08X)", prefix, i + 1, code_name, code);
            } else {
                snprintf(line, sizeof(line), "%s[%d] 0x%08X", prefix, i + 1, code);
            }
            
            unsigned int color = ((i - page_start) == state.selected_preset) ? 0xFF00FF00 : 0xFFCCCCCC;
            debug_print(10, 105 + ((i - page_start) * 12), line, color);
        }
    } else {
        debug_print(10, 90, "No codes in current protocol!", 0xFFFF0000);
        debug_print(10, 105, "Add codes via Receive Mode", 0xFFCCCCCC);
    }
    
    // Controls help
    debug_print(10, 215, "UP/DOWN: Select Code  |  X: Transmit", 0xFF888888);
    debug_print(10, 230, "LEFT/RIGHT: Page  |  L/R: Menu  |  HOME: Exit", 0xFF888888);
}

void render_receive_menu(void) {
    if (state.keyboard_active) {
        // Show keyboard overlay
        render_keyboard();
    } else {
        // Show normal receive menu
        pspDebugScreenClear();
        
        debug_print(10, 10, "IR Receive Mode", 0xFFFFFF00);
        debug_print(10, 30, "================================", 0xFF666666);
        
        if (state.ir_mode == IR_MODE_RX) {
            debug_print(10, 50, "Status: RECEIVING...", 0xFF00FF00);
            debug_print(10, 65, "Waiting for IR codes...", 0xFFFFFFFF);
            
            char proto_str[80];
            snprintf(proto_str, sizeof(proto_str), "Saving to: %s", get_protocol_name(state.current_protocol_idx));
            debug_print(10, 85, proto_str, 0xFF00FFFF);
            
            if (state.current_code != 0) {
                char code_str[80];
                snprintf(code_str, sizeof(code_str), "Last Code: 0x%08X", state.current_code);
                debug_print(10, 105, code_str, 0xFF00FFFF);
                
                debug_print(10, 225, "L: Name & Save  |  SQUARE: Exit RX  |  L/R: Change Menu", 0xFF888888);
            } else {
                debug_print(10, 225, "Waiting for IR signal...  |  SQUARE: Exit RX  |  L/R: Menu", 0xFF888888);
            }
        } else {
            debug_print(10, 50, "Status: TRANSMIT MODE", 0xFFCCCCCC);
            debug_print(10, 70, "Press X to enter receive mode", 0xFFFFFFFF);
            
            debug_print(10, 235, "X: Enter RX Mode  |  L/R: Menu", 0xFF888888);
        }
    }
}

void render_protocol_menu(void) {
    pspDebugScreenClear();
    
    debug_print(10, 10, "Select IR Protocol", 0xFFFFFF00);
    debug_print(10, 30, "================================", 0xFF666666);
    
    // Sync selected protocol with current protocol
    state.selected_protocol = state.current_protocol_idx;
    
    for (int i = 0; i < protocol_count && i < 14; i++) {
        char line[80];
        char prefix[4] = "  ";
        
        if (i == state.selected_protocol) {
            strcpy(prefix, "> ");
        }
        
        snprintf(line, sizeof(line), "%s%s (%d codes)", prefix, protocols[i].name, protocols[i].code_count);
        
        unsigned int color = (i == state.selected_protocol) ? 0xFF00FF00 : 0xFFCCCCCC;
        debug_print(10, 50 + (i * 12), line, color);
    }
    
    if (protocol_count == 0) {
        debug_print(10, 50, "No protocols found!", 0xFFFF0000);
        debug_print(10, 65, "Create protocol files in ms0:/game/ircontroller/", 0xFFCCCCCC);
    }
    
    debug_print(10, 235, "UP/DOWN: Select  |  X: Confirm  |  L/R: Menu", 0xFF888888);
}

void render_history_menu(void) {
    pspDebugScreenClear();
    
    debug_print(10, 10, "IR Code History", 0xFFFFFF00);
    debug_print(10, 30, "================================", 0xFF666666);
    
    if (history_count > 0) {
        for (int i = 0; i < history_count && i < 14; i++) {
            char line[80];
            const char *mode_str = (history[i].mode == IR_MODE_TX) ? "TX" : "RX";
            snprintf(line, sizeof(line), "[%s] 0x%08X (%s)", mode_str, 
                    history[i].code, get_protocol_name(history[i].protocol));
            
            debug_print(10, 50 + (i * 12), line, 0xFFFFFFFF);
        }
    } else {
        debug_print(10, 50, "No history yet", 0xFFCCCCCC);
    }
    
    debug_print(10, 235, "START: Back to Main Menu", 0xFF888888);
}

// --- MAIN THREAD ---
int main_thread(SceSize args, void *argp) {
    init_graphics();
    
    // Try to acquire general app lock (only one app can run at a time)
    if (!try_acquire_app_lock()) {
        pspDebugScreenPuts("ERROR: Another app is already running!");
        pspDebugScreenPuts("(IR Controller or BT Manager)");
        pspDebugScreenPuts("");
        pspDebugScreenPuts("Exit the other app first.");
        pspDebugScreenPuts("");
        pspDebugScreenPuts("Press HOME to exit.");
        while (1) {
            SceCtrlData pad;
            sceCtrlReadBufferPositive(&pad, 1);
            if (pad.Buttons & PSP_CTRL_HOME) {
                sceKernelExitGame();
            }
            sceKernelDelayThread(100000);
        }
        return -1;
    }
    
    // Open message pipe to UART Manager
    state.ir_pipe = sceKernelCreateMsgPipe("IrCmdPipe", 1, 0, 256, NULL);
    if (state.ir_pipe < 0) {
        pspDebugScreenPuts("ERROR: Cannot open IR Manager pipe!");
        release_app_lock();
        sceKernelSleepThread();
        return -1;
    }
    
    // Initialize data directory and scan for protocols
    create_data_directory();
    scan_protocols();
    load_default_protocol();
    
    if (protocol_count == 0) {
        strncpy(state.status_msg, "No protocols found!", sizeof(state.status_msg) - 1);
    } else {
        snprintf(state.status_msg, sizeof(state.status_msg), "Loaded %d protocol(s)", protocol_count);
    }
    
    state.ir_mode = IR_MODE_TX;
    state.menu_state = 0;
    state.selected_preset = 0;
    state.selected_protocol = state.current_protocol_idx;
    strncpy(state.status_msg, "Ready.", sizeof(state.status_msg) - 1);
    
    // Initialize IR mode
    set_ir_mode_tx();
    
    SceCtrlData pad;
    
    while (1) {
        // Read controller input
        sceCtrlReadBufferPositive(&pad, 1);
        handle_input(&pad);
        
        // Render appropriate menu
        if (state.menu_state == 3) {
            render_history_menu();
        } else if (state.menu_state == 2) {
            render_protocol_menu();
        } else if (state.menu_state == 1) {
            render_receive_menu();
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
    SceUID thid = sceKernelCreateThread("IRController", main_thread, 0x20, 
                                         0x10000, 0, NULL);
    if (thid >= 0) {
        sceKernelStartThread(thid, args, argp);
    }
    return 0;
}

int module_stop(SceSize args, void *argp) {
    // Release the app lock so other apps can run
    release_app_lock();
    
    if (state.ir_pipe >= 0) {
        sceKernelDeleteMsgPipe(state.ir_pipe);
    }
    
    // Free allocated memory
    for (int i = 0; i < protocol_count; i++) {
        if (protocols[i].codes != NULL) {
            free(protocols[i].codes);
            protocols[i].codes = NULL;
        }
        if (protocols[i].code_names != NULL) {
            for (int j = 0; j < protocols[i].code_count; j++) {
                if (protocols[i].code_names[j] != NULL) {
                    free(protocols[i].code_names[j]);
                }
            }
            free(protocols[i].code_names);
            protocols[i].code_names = NULL;
        }
    }
    
    return 0;
}
