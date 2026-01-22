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
PSP_MAIN_THREAD_ATTR(0x10); // High Priority

// --- CONFIGURAZIONE E STATO ---
#define MAX_BUFFER_SIZE 512
static unsigned char uart_rx_buffer[MAX_BUFFER_SIZE];
static int uart_rx_index = 0;

static SceUID cmd_pipe_out = -1;
static SceUID uart_resp_pipe = -1;
static SceUID app_lock_semaphore = -1;

typedef struct {
    int baudrate;
    int delay_us;
    int buffer_limit;
} Config;

Config g_config = {9600, 750, 255}; // Default values

void save_defaults() {
    // Open for writing, create if it doesn't exist, and truncate if it does
    SceUID fd = sceIoOpen(CONFIG_PATH, PSP_O_WRONLY | PSP_O_CREAT | PSP_O_TRUNC, 0777);
    if (fd >= 0) {
        char output[128];
        // Format the defaults into a string
        int len = sprintf(output, "baudrate = %d\ndelay_us = %d\nbuffer_limit = %d\n", 
                          g_config.baudrate, g_config.delay_us, g_config.buffer_limit);
        sceIoWrite(fd, output, len);
        sceIoClose(fd);
    }
}

void load_config() {
    // Open the file using PSP SDK file I/O
    SceUID fd = sceIoOpen("ms0:/seplugins/uart_config.ini", PSP_O_RDONLY, 0777);
    if (fd < 0) return; // Exit if file doesn't exist

    char file_buf[512];
    int bytesRead = sceIoRead(fd, file_buf, sizeof(file_buf) - 1);
    sceIoClose(fd);

    if (bytesRead > 0) {
        file_buf[bytesRead] = '\0';
        
        // Split the buffer into lines and parse each
        char *line = strtok(file_buf, "\r\n");
        while (line != NULL) {
            int val;
            // Use sscanf to match key = value pairs
            // The " %d" format automatically skips the '=' and spaces
            if (sscanf(line, "baudrate = %d", &val) == 1) {
                g_config.baudrate = val;
            } 
            else if (sscanf(line, "delay_us = %d", &val) == 1) {
                g_config.delay_us = val;
            } 
            else if (sscanf(line, "buffer_limit = %d", &val) == 1) {
                // Apply protection against overflow
                if (val < MAX_BUFFER_SIZE) g_config.buffer_limit = val;
                else g_config.buffer_limit = MAX_BUFFER_SIZE - 1;
            }
            
            line = strtok(NULL, "\r\n"); // Move to next line
        }
    } else save_defaults()
}

/// @brief TRasmettere dati alla uart
/// @param data 
/// @param len 
/// @return 
int UARTManagerSend(void* data, int len) {
    return sceKernelSendMsgPipe(cmd_pipe_out, data, len, PSP_MSGPIPE_WAIT, NULL, NULL);
}

int UARTManagerReceive(void* data, int max_len) {
    return sceKernelReceiveMsgPipe(uart_resp_pipe, data, max_len, PSP_MSGPIPE_WAIT, NULL, NULL);
}


// --- RICEZIONE UART ---
void process_uart_data(void) {
    int ch;
    
    while ((ch = pspDebugSioGetchar()) != -1) {
        unsigned char c = (unsigned char)ch;
        
        if (c == '\r') return; // Ignora Carriage Return (standard nei comandi AT)

        if (c == '\n') {
            // Chiudi la stringa nel buffer
            uart_rx_buffer[uart_rx_index] = '\0';
            
            // Invia la risposta all'App/Plugin tramite la Pipe di ritorno
            // Usiamo PSP_MSGPIPE_NOWAIT per non bloccare il kernel se l'app non legge
            if (uart_resp_pipe >= 0 && uart_rx_index > 0) {
                sceKernelSendMsgPipe(uart_resp_pipe, uart_rx_buffer, uart_rx_index, PSP_MSGPIPE_NOWAIT, NULL, NULL);
            }
            
            // Reset indice per il prossimo messaggio
            uart_rx_index = 0;
        } 
        else if (uart_rx_index < g_config.buffer_limit) {
            // Aggiungi carattere al buffer
            uart_rx_buffer[uart_rx_index++] = c;
        } 
        else {
            // Emergenza: Buffer saturo senza aver ricevuto \n
            // Inviamo quello che abbiamo finora per liberare spazio
            uart_rx_buffer[uart_rx_index] = '\0';
            if (uart_resp_pipe >= 0) {
                sceKernelSendMsgPipe(uart_resp_pipe, uart_rx_buffer, uart_rx_index, PSP_MSGPIPE_NOWAIT, NULL, NULL);
            }
            uart_rx_index = 0;
        }
    }
}


//Google respond of the control 
/*
La documentazione ufficiale Sony non è mai stata rilasciata al pubblico, ma la comunità homebrew (grazie al reverse engineering degli anni 2000) ha identificato esattamente come funziona il protocollo.
Il telecomando comunica a 4800 bps (molto lento) quando è in modalità standard, ma dato che tu userai la UART a 38.400 bps per il Bluetooth, dovrai campionare i byte binari che arrivano.
Ecco i dettagli tecnici per il tuo driver:
1. Struttura del Pacchetto
Il telecomando non invia un solo byte, ma un pacchetto di 5 byte ogni volta che un tasto viene premuto o rilasciato.
Byte 0: Header (fisso, solitamente 0xFF o 0x00 a seconda del jitter).
Byte 1: ID pacchetto.
Byte 2: Codice del Tasto (Quello che ti serve).
Byte 3: Checksum o bit di stato.
Byte 4: Footer.
2. Tabella dei Codici dei Tasti (Byte 2)
In base ai sorgenti storici di progetti come RemoteJoy e PSPLINK, i valori esadecimali che leggerai sulla UART per i tasti fisici sono i seguenti:
Tasto	Codice Esadecimale (Byte 2)	Azione Kernel PSP
Play/Pause	0x01	PSP_CTRL_PLAYPAUSE
Forward (Avanti)	0x02	PSP_CTRL_FORWARD
Back (Indietro)	0x04	PSP_CTRL_BACK
Volume +	0x08	PSP_CTRL_VOLUP
Volume -	0x10	PSP_CTRL_VOLDOWN
Hold	0x20	Blocca gli altri comandi
*/


//devo pensare se voglio gestire il telecomando della psp qui?
/*
void process_uart_data(void) {
    int ch;
    while ((ch = pspDebugSioGetchar()) != -1) {
        unsigned char c = (unsigned char)ch;

        // A. FILTRO TELECOMANDO ORIGINALE (Byte Binari < 32)
        if (c < 32 && c != '\r' && c != '\n' && c != '\t') {
            // Se il valore è molto basso, non è testo "AT+" o "IR+". 
            // È quasi certamente un byte del telecomando Sony.
            
            // Esempio: Iniezione nel sistema tramite le maschere tasti PSP
            if (c == 0x08) sceCtrlSetButtonMasks(PSP_CTRL_PLAYPAUSE, 0);
            if (c == 0x01) sceCtrlSetButtonMasks(PSP_CTRL_VOLUP, 0);
            // ... e così via per tutti i 6 tasti
            continue; 
        }

        // B. FILTRO STRINGHE ( evrything that is not the control)
        if (uart_rx_index < g_config.buffer_limit) {
            uart_rx_buffer[uart_rx_index++] = c;


            // GESTIONE COMANDI TESTUALI (in questo modo salto anche la verifica di hold+)
                // 1. GESTIONE VOLUME
                if      (strstr(uart_rx_buffer, "V+"))    inject_system_button(PSP_CTRL_VOLUP);
                else if (strstr(uart_rx_buffer, "V-"))    inject_system_button(PSP_CTRL_VOLDOWN);
                
                // 2. GESTIONE RIPRODUZIONE
                else if (strstr(uart_rx_buffer, "PLAY"))  inject_system_button(PSP_CTRL_PLAYPAUSE);
                else if (strstr(uart_rx_buffer, "PAUSE")) inject_system_button(PSP_CTRL_PLAYPAUSE);
                else if (strstr(uart_rx_buffer, "MUTE")) inject_system_button(PSP_CTRL_PLAYPAUSE);
                
                // 3. GESTIONE TRACCE
                else if (strstr(uart_rx_buffer, "NEXT"))  inject_system_button(PSP_CTRL_FORWARD);
                else if (strstr(uart_rx_buffer, "PREV"))  inject_system_button(PSP_CTRL_BACK);

                // AUTO-PAUSE SU DISCONNESSIONE (Opzionale ma professionale non ne sono sicuro perchè mi sembra un problema nel caso la musica sia in pausa, in quel caso la avvia)
                else if (strstr(uart_rx_buffer, "DISCON")) inject_system_button(PSP_CTRL_PLAYPAUSE);

            
            // Se troviamo un terminatore, spediamo la stringa intera alla pipe
            if (c == '\n') {
                uart_rx_buffer[uart_rx_index] = '\0';
                sceKernelSendMsgPipe(uart_resp_pipe, uart_rx_buffer, uart_rx_index, PSP_MSGPIPE_NOWAIT, NULL, NULL);
                uart_rx_index = 0;
            }
        }
    }
}
*/


// --- INVIO UART ---
void send_uart_command(const char *command) {
    if (command == NULL) return;
    const char *p = command;
    while (*p) {
        pspDebugSioPutchar(*p++);
    }
    pspDebugSioPutchar('\n');
}

// --- THREAD PRINCIPALE ---
int uart_manager_thread(SceSize args, void *argp) {
    load_config(); // Carica impostazioni da file .ini
    
    app_lock_semaphore = sceKernelCreateSema("app_lock", 0, 1, 1, NULL);
    
    pspDebugSioInit();
    pspDebugSioSetBaud(g_config.baudrate);

    unsigned char msg_buf[MAX_BUFFER_SIZE];

    while(1) {
       
        // 1. Legge dati in entrata dal dispositivo esterno (UART -> Buffer)
        process_uart_data();

        // 2. Legge comandi in uscita dai plugin (Pipe Unica -> UART)
        // Non importa se il comando viene dal plugin BT o IR, la pipe è la stessa
        int recv_size = sceKernelReceiveMsgPipe(cmd_pipe_out, msg_buf, g_config.buffer_limit, PSP_MSGPIPE_NOWAIT, NULL, NULL);
        
        if (recv_size > 0) {
            msg_buf[recv_size] = '\0';
            send_uart_command((const char*)msg_buf);
        }
        
        // Pausa dinamica
        sceKernelDelayThread(g_config.delay_us);
    }
    return 0;
}


// --- ENTRY POINTS DEL MODULO ---
int module_start(SceSize args, void *argp) {
    // Pipe per comandi in USCITA (App -> UART)
    cmd_pipe_out = sceKernelCreateMsgPipe("UartCmdPipe", 1, 0, MAX_BUFFER_SIZE, NULL);
    
    // Pipe per risposte in ENTRATA (UART -> App)
    uart_resp_pipe = sceKernelCreateMsgPipe("UartRespPipe", 1, 0, MAX_BUFFER_SIZE, NULL);
    
    SceUID thid = sceKernelCreateThread("UART_Manager", uart_manager_thread, 0x10, 0x2000, 0, NULL);
    if (thid >= 0) {
        sceKernelStartThread(thid, 0, NULL);
    }
    return 0;
}


int module_stop(SceSize args, void *argp) {
    if (cmd_pipe_out >= 0) sceKernelDeleteMsgPipe(cmd_pipe_out);
    if (uart_resp_pipe >= 0) sceKernelDeleteMsgPipe(uart_resp_pipe);
    if (app_lock_semaphore >= 0) sceKernelDeleteSema(app_lock_semaphore);
    return 0;
}
