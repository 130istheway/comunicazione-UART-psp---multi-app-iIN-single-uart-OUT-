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
#define IR_CONFIG_FILE "ms0:/game/ircontroller/config.txt"

// --- IR MODE DEFINITIONS ---
#define IR_MODE_TX 0
#define IR_MODE_RX 1

// --- DATA STRUCTURES ---

// - IR PROTOCOL STRUCTURE -

typedef struct{
unsigned int MAX_PROTOCOLS = 16;
unsigned int HISTORY_SIZE = 20;
unsigned int IR_RESPONSE_BUFFER_SIZE = 128;
unsigned int KEYBOARD_COLS = 10;
unsigned int CODES_PER_PAGE = 10;
} IR_Config;

IR_config g_ir_config = {16, 20, 138, 10, 10};

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

// - STATE STRUCTURE -
typedef struct {
    SceUID ir_pipe;
    SceUID resp_pipe;
    
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

// - IR CODE HISTORY -
typedef struct {
    unsigned int code;
    unsigned char protocol;
    unsigned char mode;         // 0=TX, 1=RX
    unsigned int timestamp;
} IR_History_Entry;

// --- STATIC ALLOCATION ---

// Dynamic protocol storage
static int protocol_count = 0;
static int history_count = 0;
static int history_index = 0;
static int ir_response_index = 0;
static IR_Controller_State state = {0};
const char keyboard_layout[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789 ";

// --- FORWARD DECLARATIONS ---

void load_protocol_codes(int protocol_idx); // Forward declaration

// --- LOADING CONFIGURATION FILE ---

void load_config() {
    SceUID fd = sceIoOpen(IR_CONFIG_FILE, PSP_O_RDONLY, 0777);
    if (fd >= 0) {
        char file_buf[512]; // Buffer leggermente più grande per sicurezza
        int bytesRead = sceIoRead(fd, file_buf, sizeof(file_buf) - 1);
        if (bytesRead > 0) {
            file_buf[bytesRead] = '\0';
            
            int mp, hs, ir, kc, cpp;
            char def_name[32] = {0};

            // sscanf cercherà i pattern nel buffer. 
            // Usiamo %31s per leggere il nome del protocollo dopo "DEFAULT="
            int matched = sscanf(file_buf, 
                "MAX_PROTOCOLS=%d\n"
                "HISTORY_SIZE=%d\n"
                "IR_BUFFER=%d\n"
                "COLS=%d\n"
                "CODES_PER_PAGE=%d\n"
                "DEFAULT=%31s", 
                &mp, &hs, &ir, &kc, &cpp, def_name);

            if (matched >= 5) {
                g_ir_config.MAX_PROTOCOL = mp;
                g_ir_config.HYSTORY_SIZE = hs;
                g_ir_config.KEYBOARD_COLS = kc;
                g_ir_config.CODES_PER_PAGE = cpp;
                
                // Protezione buffer IR
                if (ir < MAX_BUFFER_SIZE) g_ir_config.IR_RESPONSE_BUFFER_SIZE = ir;
                else g_ir_config.IR_RESPONSE_BUFFER_SIZE = MAX_BUFFER_SIZE - 1;

                // Se abbiamo trovato anche il nome del protocollo DEFAULT (matched == 6)
                if (matched == 6) {
                    state.current_protocol_idx = 0; // Default di sicurezza
                    
                    // Cerchiamo quale indice corrisponde al nome letto
                    for (int i = 0; i < protocol_count; i++) {
                        if (strcmp(protocols[i].name, def_name) == 0) {
                            state.current_protocol_idx = i;
                            break;
                        }
                    }
                }
            }
        }
        sceIoClose(fd);
    } else {
        save_default_protocol(0);
    }
}

// --- THINGS --- after the actual loading of the configuration file

static IR_Protocol protocols[g_ir_config.MAX_PROTOCOLS] = {0};
static IR_History_Entry history[g_ir_config.HISTORY_SIZE] = {0};
static char ir_response_buffer[g_ir_config.IR_RESPONSE_BUFFER_SIZE];

// --- TOUCHING THE IR CONFIGURATION FILE ---

void save_default_protocol(int state.protocol_idx) {
    SceUID fd = sceIoOpen(IR_CONFIG_FILE, PSP_O_WRONLY | PSP_O_CREAT | PSP_O_TRUNC, 0777);
    
    if (fd >= 0) {
        char buffer[512];

        // 1. Righe 1-5: Configurazioni globali
        snprintf(buffer, sizeof(buffer), 
            "MAX_PROTOCOLS=%d\n"       // Riga 1
            "HISTORY_SIZE=%d\n"        // Riga 2
            "IR_BUFFER=%d\n"           // Riga 3
            "COLS=%d\n"                // Riga 4
            "CODES_PER_PAGE=%d\n",     // Riga 5
            g_ir_config.MAX_PROTOCOLS, 
            g_ir_config.HISTORY_SIZE, 
            g_ir_config.IR_RESPONSE_BUFFER_SIZE, 
            g_ir_config.KEYBOARD_COLS, 
            g_ir_config.CODES_PER_PAGE);
        sceIoWrite(fd, buffer, strlen(buffer));

        // 2. Riga 6: Il protocollo di default
        char line6[64];
        snprintf(line6, sizeof(line6), "DEFAULT=%s\n", protocols[protocol_idx].name);
        sceIoWrite(fd, line6, strlen(line6));

        // 3. Righe 7-10: Filler (per garantire che i protocolli inizino alla riga 11)
        const char *filler = "UNUSED=0\nUNUSED=0\nUNUSED=0\nUNUSED=0\n";
        sceIoWrite(fd, filler, strlen(filler));

        // 4. Riga 11+: Blocchi protocolli (4 righe per protocollo)
        for (int i = 0; i < protocol_count; i++) {
            char proto_block[256];
            snprintf(proto_block, sizeof(proto_block), 
                "protocolName=%s\n"
                "codeCount=%d\n"
                "carrierFrequency=%d\n"
                "bits=%d\n",
                protocols[i].name, 
                protocols[i].code_count,
                protocols[i].carrier_freq, 
                protocols[i].bits);
            sceIoWrite(fd, proto_block, strlen(proto_block));
        }

        sceIoClose(fd);
    }
}

int read_line(SceUID fd, char *buf, int max_len) {
    int i = 0;
    char c;
    while (i < max_len - 1) {
        if (sceIoRead(fd, &c, 1) <= 0) break;
        if (c == '\r') continue; // Ignore carriage returns
        if (c == '\n') break;    // Stop at newline
        buf[i++] = c;
    }
    buf[i] = '\0';
    return i; // Returns length of line, 0 if empty/EOF
}

void load_protocol_from_config(int protocol_idx) {
    SceUID fd = sceIoOpen(IR_CONFIG_FILE, PSP_O_RDONLY, 0777);
    if (fd < 0) return;

    char line[128];
    int current_line = 0;

    // 1. Skip the first 10 lines (Global Config)
    while (current_line < 10) {
        if (read_line(fd, line, sizeof(line)) <= 0) {
            sceIoClose(fd);
            return; // End of file reached too early
        }
        current_line++;
    }

    // 2. Skip to the specific protocol block
    // Each protocol block = 4 lines. We skip (protocol_idx * 4) lines.
    int lines_to_skip = protocol_idx * 4;
    for (int i = 0; i < lines_to_skip; i++) {
        if (read_line(fd, line, sizeof(line)) <= 0) {
            sceIoClose(fd);
            return;
        }
    }

    // 3. Read the 4 lines for the current protocol
    char name_line[64], count_line[64], freq_line[64], bits_line[64];
    if (read_line(fd, name_line, sizeof(name_line)) > 0 &&
        read_line(fd, count_line, sizeof(count_line)) > 0 &&
        read_line(fd, freq_line, sizeof(freq_line)) > 0 &&
        read_line(fd, bits_line, sizeof(bits_line)) > 0) {

        // Parse keys: protocolName=, codeCount=, carrierFrequency=, bits=
        sscanf(name_line, "protocolName=%31s", protocols[protocol_idx].name);
        sscanf(count_line, "codeCount=%d", &protocols[protocol_idx].code_count);
        int temp_freq, temp_bits;
        sscanf(freq_line, "carrierFrequency=%d", &temp_freq);
        sscanf(bits_line, "bits=%d", &temp_bits);
        
        protocols[protocol_idx].carrier_freq = (uint16_t)temp_freq;
        protocols[protocol_idx].bits = (uint8_t)temp_bits;
        
        // Note: Initialize your dynamic arrays here if needed
        protocols[protocol_idx].codes = NULL; 
        protocols[protocol_idx].code_names = NULL;
    }

    sceIoClose(fd);
}


// --- TOUCHING THE IR DATA FILE ---

void scan_protocols(void) {
    SceUID dir = sceIoDopen(IR_DATA_DIR);
    if (dir >= 0) {
        SceIoDirent entry;
        protocol_count = 0;
        
        while (sceIoDread(dir, &entry) > 0 && protocol_count < g_ir_config.MAX_PROTOCOLS) {
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

void load_protocol_codes(int protocol_idx) {
    if (protocol_idx < 0 || protocol_idx >= g_ir_config.MAX_PROTOCOLS) return;
    
    SceUID fd = sceIoOpen(protocols[protocol_idx].filename, PSP_O_RDONLY, 0);
    if (fd < 0) return;

    // Reset counts for this protocol
    protocols[protocol_idx].code_count = 0;
    
    char line[128];
    int line_num = 0;
    
    // Process remaining lines: "Name -> 0xCODE"
    while (read_line(fd, line, sizeof(line)) > 0) {
        unsigned int code = 0;
        char code_name[64] = {0};
        
        char *arrow = strstr(line, "->");
        if (arrow != NULL) {
            // Extract Name
            int name_len = arrow - line;
            if (name_len > 63) name_len = 63;
            strncpy(code_name, line, name_len);
            code_name[name_len] = '\0';
            
            // Trim trailing spaces
            while (name_len > 0 && code_name[name_len-1] == ' ') code_name[--name_len] = '\0';

            // Extract Hex Code
            sscanf(arrow + 2, " %x", &code);
        } else {
            // No arrow, treat whole line as hex or skip
            if (sscanf(line, "%x", &code) != 1) continue;
            snprintf(code_name, sizeof(code_name), "Code_%d", protocols[protocol_idx].code_count + 1);
        }

        // --- DYNAMIC REALLOCATION ---
        if (protocols[protocol_idx].code_count >= protocols[protocol_idx].code_capacity) {
            int new_cap = (protocols[protocol_idx].code_capacity == 0) ? 16 : protocols[protocol_idx].code_capacity * 2;
            
            unsigned int *new_codes = realloc(protocols[protocol_idx].codes, new_cap * sizeof(unsigned int));
            char **new_names = realloc(protocols[protocol_idx].code_names, new_cap * sizeof(char *));
            
            if (new_codes && new_names) {
                protocols[protocol_idx].codes = new_codes;
                protocols[protocol_idx].code_names = new_names;
                protocols[protocol_idx].code_capacity = new_cap;
            } else {
                break; // Memory failure
            }
        }

        // --- STORAGE ---
        protocols[protocol_idx].codes[protocols[protocol_idx].code_count] = code;
        protocols[protocol_idx].code_names[protocols[protocol_idx].code_count] = strdup(code_name);
        protocols[protocol_idx].code_count++;
    }
    sceIoClose(fd);
}

int find_protocol_file_by_name(const char *target_name, char *out_full_path) {
    SceUID dfd = sceIoDopen(IR_DATA_DIR);
    if (dfd < 0) {
        // Directory doesn't exist or cannot be opened
        return -1; 
    }

    SceIoDirent entry;
    memset(&entry, 0, sizeof(SceIoDirent));

    // Iterate through all files in the directory
    while (sceIoDread(dfd, &entry) > 0) {
        // Check if the file name matches the protocol name (case-sensitive)
        // Note: You may want to check for .txt or other extensions specifically
        if (strcmp(entry.d_name, target_name) == 0) {
            snprintf(out_full_path, 256, "%s%s", IR_DATA_DIR, entry.d_name);
            sceIoDclose(dfd);
            return 0; // Found it
        }
    }

    sceIoDclose(dfd);
    return -2; // Not found
}

void save_code_to_protocol_with_name(unsigned int code, int protocol_idx, const char *code_name) {
    if (protocol_idx < 0 || protocol_idx >= protocol_count) return;
    
    char found_path[256];
    // Search the directory for the protocol's filename before saving
    if (find_protocol_file_by_name(protocols[protocol_idx].name, found_path) == 0) {
        // Update the internal path to the one found in the search
        strncpy(protocols[protocol_idx].filename, found_path, sizeof(protocols[protocol_idx].filename));
    } else {
        // If not found, use a default path in the IR_DATA_DIR
        snprintf(protocols[protocol_idx].filename, sizeof(protocols[protocol_idx].filename), 
                 "%s%s.txt", IR_DATA_DIR, protocols[protocol_idx].name);
    }

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
        char **new_names = (char **)realloc(protocols[protocol_idx].code_names, new_capacity * sizeof(char *));
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

// --- IR RESPONSE PARSER ---
// This plugin now parses responses from IR commands
void parse_at_response(const char *response) {
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
        snprintf(state.status_msg, sizeof(state.status_msg), "IR Mode: %s", mode ? "RX (Receiver)" : "TX (Transmitter)");
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

// --- UART RESPONSE PROCESSING (Polling from UART) --- in case i want to read the responses
void process_uart_responses(void) {
    char response_buffer[256];
    
    // Legge dalla pipe in modo non bloccante
    int bytes_received = sceKernelReceiveMsgPipe(resp_pipe_id, response_buffer, 255, PSP_MSGPIPE_NOWAIT, NULL, NULL);

    if (bytes_received > 0) {
        response_buffer[bytes_received] = '\0'; // Assicura lo zero terminale
        
        // CHIAMATA AL PARSER: analizza il contenuto del comando AT ricevuto
        parse_at_response(response_buffer);
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
    snprintf(ir_cmd, sizeof(ir_cmd), "IR+TX=%08X", code);
    send_ir_command(ir_cmd);
    state.current_code = code;
    
    // Add to history
    if (history_count < g_ir_config.HISTORY_SIZE) {
        history[history_index].code = code;
        history[history_index].protocol = state.ir_protocol;
        history[history_index].mode = IR_MODE_TX;
        history[history_index].timestamp = sceKernelGetSystemTimeLow();
        history_index = (history_index + 1) % g_ir_config.HISTORY_SIZE;
        if (history_count < g_ir_config.HISTORY_SIZE) history_count++;
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
    // Command: IR+PROTO=<name>,<carrier_freq>,<bits>
    // I send the protocol parameters to ESP32 even if the name is not used by ESP32, but in a future it could be useful for debugging or by having specific behaviors for certain protocols on ESP32 side like routing to different decoders? or different modulation schemes? or different hardware sending methods like for home automation? but it's not implemented now and can be deprecated in the future.
    if (protocol_idx >= 0 && protocol_idx < protocol_count) {
        IR_Protocol *proto = &protocols[protocol_idx];
        
        char cmd[96];
        snprintf(cmd, sizeof(cmd), "IR+PROTO=%d,%d", proto->carrier_freq, proto->bits);
        
        send_ir_command(cmd);
    }
}

void set_ir_protocol(int protocol_idx) {
    if (protocol_idx >= 0 && protocol_idx < protocol_count) {
        state.current_protocol_idx = protocol_idx;
        load_protocol_from_config(protocol_idx);
        
        // Send protocol parameters to ESP32
        send_protocol_parameters(protocol_idx);
        
        snprintf(state.status_msg, sizeof(state.status_msg), "Protocol: %s", protocols[protocol_idx].name);
    }
}

// --- KEYBOARD INPUT ---
// Simple on-screen keyboard for naming received codes
// Layout: the number of columns is defined by KEYBOARD_COLS in a .ini file
// Characters: A-Z only Uppercase, 0-9, space
// In the future, we could expand this to include more characters or symbols if needed and i feel the necessity, maibe by having it in the .ini file? .

void render_keyboard(void) {
    pspDebugScreenClear();
    debug_print(10, 10, "Enter Name for IR Code", 0xFFFFFF00);
    debug_print(10, 30, state.keyboard_input, 0xFF00FF00);
    debug_print(10, 50, "--------------------------------------------", 0xFF666666);
    
    int keyboard_size = strlen(keyboard_layout);
    int row = 0;
    
    for (int i = 0; i < keyboard_size; i++) {
        int col = i % g_ir_config.KEYBOARD_COLS;
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
    int max_rows = (keyboard_size + g_ir_config.KEYBOARD_COLS - 1) / g_ir_config.KEYBOARD_COLS;
    
    if (pad->Buttons & PSP_CTRL_UP) {
        state.keyboard_cursor_y = (state.keyboard_cursor_y > 0) ? state.keyboard_cursor_y - 1 : max_rows - 1;
        sceKernelDelayThread(150000);
    }
    if (pad->Buttons & PSP_CTRL_DOWN) {
        state.keyboard_cursor_y = (state.keyboard_cursor_y < max_rows - 1) ? state.keyboard_cursor_y + 1 : 0;
        sceKernelDelayThread(150000);
    }
    if (pad->Buttons & PSP_CTRL_LEFT) {
        state.keyboard_cursor_x = (state.keyboard_cursor_x > 0) ? state.keyboard_cursor_x - 1 : g_ir_config.KEYBOARD_COLS - 1;
        sceKernelDelayThread(150000);
    }
    if (pad->Buttons & PSP_CTRL_RIGHT) {
        state.keyboard_cursor_x = (state.keyboard_cursor_x < g_ir_config.KEYBOARD_COLS - 1) ? state.keyboard_cursor_x + 1 : 0;
        sceKernelDelayThread(150000);
    }
    
    if (pad->Buttons & PSP_CTRL_CROSS) {
        // Select character
        int index = state.keyboard_cursor_y * g_ir_config.KEYBOARD_COLS + state.keyboard_cursor_x;
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
        // int codes_per_page = 10; moved to the define section
        int total_pages = (current_code_count > 0) ? (current_code_count + g_ir_config.CODES_PER_PAGE - 1) / g_ir_config.CODES_PER_PAGE : 1;
        int page_start = state.current_page * g_ir_config.CODES_PER_PAGE;
        int page_end = (state.current_page + 1) * g_ir_config.CODES_PER_PAGE;
        if (page_end > current_code_count) page_end = current_code_count;
        
        if (pad->Buttons & PSP_CTRL_UP && state.selected_preset > 0) {
            state.selected_preset--;
            sceKernelDelayThread(150000);
        }else if (pad -> Buttons & PSP_CTRL_UP && state.selected_preset == 0 && state.current_page > 0)
        {
            state.current_page--;
            state.selected_preset = g_ir_config.CODES_PER_PAGE -1;
            sceKernelDelayThread(150000);
        }else if (pad -> Buttons & PSP_CTRL_UP && state.selected_preset == 0 && state.current_page == 0)
        {
            state.current_page = total_pages -1;
            int last_page_count = current_code_count % g_ir_config.CODES_PER_PAGE;
            if (last_page_count == 0)
                state.selected_preset = g_ir_config.CODES_PER_PAGE -1;
            else
                state.selected_preset = last_page_count -1;

            sceKernelDelayThread(150000);
        }

        if (pad->Buttons & PSP_CTRL_DOWN && state.selected_preset < page_end - page_start - 1) {
            state.selected_preset++;
            sceKernelDelayThread(150000);
        } else if (pad -> Buttons & PSP_CTRL_DOWN && state.selected_preset == page_end - page_start -1 && state.current_page < total_pages -1)
        {
            state.current_page++;
            state.selected_preset = 0;
            sceKernelDelayThread(150000);
        } else if (pad -> Buttons & PSP_CTRL_DOWN && state.selected_preset == page_end - page_start -1 && state.current_page == total_pages -1)
        {
            state.current_page = 0;
            state.selected_preset = 0;
            sceKernelDelayThread(150000);
        }
        
        if (pad->Buttons & PSP_CTRL_LEFT && state.current_page > 0) {
            state.current_page--;
            sceKernelDelayThread(200000);
        }
        if (pad->Buttons & PSP_CTRL_RIGHT && state.current_page < total_pages - 1) {
            state.current_page++;
            sceKernelDelayThread(200000);
        }
        
        if (pad->Buttons & PSP_CTRL_CROSS && current_code_count > 0 && state.ir_mode == IR_MODE_TX) {
            // Transmit code from current protocol
            unsigned int code = protocols[state.current_protocol_idx].codes[page_start + state.selected_preset];
            transmit_ir_code(code);
            sceKernelDelayThread(300000);
        }

        if (pad->Buttons & PSP_CTRL_CROSS && state.ir_mode == IR_MODE_RX) {
            set_ir_mode_tx();
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
                // need to resend the current code that is viewed ti verify if the code is correct
                unsigned int code = protocols[state.current_protocol_idx].codes[page_start + state.selected_preset];
                transmit_ir_code(code);
                sceKernelDelayThread(300000);
            }

            if (pad->Buttons & PSP_CTRL_CIRCLE && state.ir_mode == IR_MODE_RX && state.current_code != 0) {
                // Save last received code to current protocol
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

        //maybe square to delete protocol? to be implemented

        //maybe triangle to create a new protocolo?

        //maybe circle to copy protocol?
    }
    
    if (state.menu_state == 3) {
        // History view
        if (pad->Buttons & PSP_CTRL_UP || pad->Buttons & PSP_CTRL_DOWN) {
            // Scroll through history (can be enhanced) not implemented
            sceKernelDelayThread(150000);
        }

        //maybe cross to resend the code? idk

        if (pad->Buttons & PSP_CTRL_START) {
            state.menu_state = 0; // Back to main menu
            sceKernelDelayThread(300000);
        }
    }
}

void modify_ESP32_mode(){
    if(state.menu_state == 1){
        send_ir_command("IR+MODE+RX");
    }else
        send_ir_command("IR+MODE+TX");
}

// --- DISPLAY RENDERING ---
void render_main_menu(void) {
    pspDebugScreenClear();
    
    debug_print(10, 10, "ESP32 IR Controller v1.2", 0xFFFFFF00);
    debug_print(10, 60, "================================", 0xFF666666);

    if (state.ir_mode == IR_MODE_RX) {
        debug_print(10, 50, "Status: RECEIVE MODE", 0xFFCCCCCC);
        debug_print(10, 70, "Press X to enter transmit mode", 0xFFFFFFFF);
            
        debug_print(10, 235, "X: Enter TC Mode  |  L/R: Menu", 0xFF888888);

    } else {

        // Current code display
        if (state.current_code != 0) {
            char code_str[64];
            snprintf(code_str, sizeof(code_str), "Last Code: 0x%08X", state.current_code);
            debug_print(10, 75, code_str, 0xFF00FF00);
        }
        
        // Display codes from current protocol with pagination
        int current_code_count = protocols[state.current_protocol_idx].code_count;
        int g_ir_config.CODES_PER_PAGE = 10;
        int total_pages = (current_code_count > 0) ? (current_code_count + g_ir_config.CODES_PER_PAGE - 1) / g_ir_config.CODES_PER_PAGE : 1;
        int page_start = state.current_page * g_ir_config.CODES_PER_PAGE;
        int page_end = (state.current_page + 1) * g_ir_config.CODES_PER_PAGE;
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
}

// --- RECEIVE MENU --- still bugged
void render_receive_menu(void) {
    process_uart_responses();

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
                
                debug_print(10, 225, "Circle Save: Name & Save  |  SQUARE: Send code to test  |  L/R: Change Menu", 0xFF888888);
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
    
    debug_print(10, 235, "START: Back to Main Menu |  L/R: Menu", 0xFF888888);
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
    
    load_config();
    load_protocols();

    // Open message pipe to UART Manager
    state.cmd_pipe = sceKernelCreateMsgPipe("SioCmdPipe", 1, 0, 256, NULL);
    if (state.cmd_pipe < 0) {
        pspDebugScreenPuts("ERROR: Cannot open UART Manager pipe!");
        release_app_lock();
        sceKernelSleepThread();
        return -1;
    }

    state.resp_pipe = sceKernelCreateMsgPipe("UartRespPipe", 1, 0, 256, NULL);
    if (state.resp_pipe < 0) {
        pspDebugScreenPuts("ERROR: Cannot open UART Response pipe!");
        release_app_lock();
        sceKernelSleepThread();
        return -1;
    }
    

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
        modify_ESP32_mode();
        
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
    SceUID thid = sceKernelCreateThread("IRController", main_thread, 0x20, 0x10000, 0, NULL);
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
