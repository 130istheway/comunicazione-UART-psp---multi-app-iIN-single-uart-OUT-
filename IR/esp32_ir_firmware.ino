/*
 * ESP32 IR Controller Firmware
 * Communicates with PSP IR Controller via UART (9600 baud)
 * 
 * Pinout:
 * - TX: GPIO 1 (UART0)
 * - RX: GPIO 3 (UART0)
 * - IR TX: GPIO 12 (PWM for IR LED)
 * - IR RX: GPIO 14 (Input from IR receiver)
 * 
 * Protocol:
 * TX Commands: AT+IRTX=<protocol>,<code>
 * RX Mode: AT+IRRX
 * TX Mode: AT+IRTX
 * Reset: AT+RST
 */

#include <Arduino.h>
#include <IRremote.h>

// Pin definitions
#define IR_TX_PIN 12
#define IR_RX_PIN 14
#define BAUD_RATE 9600

// IR Protocol parameters structure
typedef struct {
  char name[32];
  uint16_t carrier_freq;
  uint8_t bits;
} IR_Protocol_Params;

// Current active protocol
IR_Protocol_Params current_protocol = {"NEC", 38000, 32};

// Global variables
volatile bool ir_rx_enabled = false;
volatile unsigned long last_ir_code = 0;
volatile uint8_t last_ir_protocol = 0;
volatile unsigned long ir_rx_timestamp = 0;

char uart_buffer[256];
int uart_idx = 0;

IRsend irsend(IR_TX_PIN);
IRrecv irrecv(IR_RX_PIN);
decode_results results;

// Function prototypes
void handle_at_command(char *cmd);
void send_ir_code(uint8_t protocol, unsigned long code);
void enable_ir_rx();
void disable_ir_rx();
void process_ir_reception();
void send_response(const char *response);

void setup() {
  Serial.begin(BAUD_RATE);
  
  // Initialize IR sender (must be done before receiver)
  irsend.begin();
  
  // Initialize IR receiver
  irrecv.enableIRIn();
  irrecv.disableIRIn(); // Start disabled, enable on command
  
  delay(100);
  send_response("AT+READY");
}

void loop() {
  // Check for incoming UART commands
  if (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (uart_idx > 0) {
        uart_buffer[uart_idx] = '\0';
        handle_at_command(uart_buffer);
        uart_idx = 0;
      }
    } else if (uart_idx < sizeof(uart_buffer) - 1) {
      uart_buffer[uart_idx++] = c;
    }
  }
  
  // Check for incoming IR codes (if RX enabled)
  if (ir_rx_enabled) {
    process_ir_reception();
  }
  
  delay(10);
}

void handle_at_command(char *cmd) {
  // Parse IR commands from PSP
  
  // IR+PROTO=<name>,<carrier_freq>,<bits>
  // Example: IR+PROTO=NEC,38000,32
  if (strncmp(cmd, "IR+PROTO=", 11) == 0) {
    char proto_name[32];
    uint16_t carrier = 0;
    uint8_t bits = 0;
    
    if (sscanf(cmd, "IR+PROTO=%31[^,],%hu,%hhu", proto_name, &carrier, &bits) == 3) {
      strcpy(current_protocol.name, proto_name);
      current_protocol.carrier_freq = carrier;
      current_protocol.bits = bits;
      
      char response[80];
      snprintf(response, sizeof(response), "AT+IRPROTO:SET:%s,%d,%d", 
               current_protocol.name, current_protocol.carrier_freq, current_protocol.bits);
      send_response(response);
    } else {
      send_response("AT+ERROR:PARSE");
    }
  }
  
  // IR+TX=<code>
  // Uses current protocol parameters
  else if (strncmp(cmd, "IR+TX=", 8) == 0) {
    unsigned long code = 0;
    
    // Parse: IR+TX=<code>
    if (sscanf(cmd, "IR+TX=%lX", &code) == 1) {
      send_ir_code(code);
      send_response("IR+OK");
    } else {
      send_response("IR+ERROR:PARSE");
    }
  }
  
  // IR+RX - Enable IR receive mode
  else if (strcmp(cmd, "IR+RX") == 0) {
    enable_ir_rx();
    send_response("IR+RX:ENABLED");
  }
  
  // IR+TX - Disable IR receive mode (switch to TX)
  else if (strcmp(cmd, "IR+TX") == 0) {
    disable_ir_rx();
    send_response("IR+TX:ENABLED");
  }
  
  // IR+RST - Reset
  else if (strcmp(cmd, "IR+RST") == 0) {
    send_response("IR+RESET");
    delay(1000);
    ESP.restart();
  }
  
  // IR+STATUS - Get status
  else if (strcmp(cmd, "IR+STATUS") == 0) {
    if (ir_rx_enabled) {
      send_response("IR+STATUS:RX_MODE");
    } else {
      send_response("IR+STATUS:TX_MODE");
    }
  }
  
  // Unknown command
  else {
    send_response("IR+ERROR:UNKNOWN");
  }
}

void send_ir_code(unsigned long code) {
  // Send IR code using current protocol parameters
  // Uses generic IR send with carrier frequency
  
  // IRremote library supports sending raw IR with custom parameters
  // For simplicity, we send using the detected protocol
  // In production, could use PWM directly for full control
  
  irsend.sendRaw(code, current_protocol.bits);
  
  // Log transmission with protocol info
  char response[80];
  snprintf(response, sizeof(response), "IR+TX:SENT:0x%08lX [%s]", code, current_protocol.name);
  send_response(response);
}

void enable_ir_rx() {
  irrecv.enableIRIn();
  ir_rx_enabled = true;
}

void disable_ir_rx() {
  irrecv.disableIRIn();
  ir_rx_enabled = false;
}

void process_ir_reception() {
  if (irrecv.decode(&results)) {
    // the protocol data come from the psp plugin
    // maube can be done in automatic but i dont think
    
    // Send received code to PSP
    char response[64];
    snprintf(response, sizeof(response), "IR+RX:CODE:0x%08lX,%d", code, protocol);
    send_response(response);
    
    // Resume receiver
    irrecv.resume();
  }
}

void send_response(const char *response) {
  Serial.print(response);
  Serial.print("\r\n");
  Serial.flush();
}

/*
 * UART Command Reference:
 * 
 * Set Protocol Parameters:
 *   AT+IRPROTO=<name>,<carrier_freq>,<bits>
 *   Example: AT+IRPROTO=NEC,38000,32
 *   Response: AT+IRPROTO:SET:NEC,38000,32
 * 
 * Transmit IR Code (uses current protocol):
 *   AT+IRTX=<code>
 *   Example: AT+IRTX=0x20DF40BF
 *   Response: AT+OK or AT+IRTX:SENT:0x20DF40BF [NEC]
 * 
 * Enable Receive Mode:
 *   AT+IRRX
 *   Response: AT+IRRX:ENABLED
 *   Then receives: AT+IRRX:CODE:0x20DF40BF
 * 
 * Switch to Transmit Mode:
 *   AT+IRTX
 *   Response: AT+IRTX:ENABLED
 * 
 * Get Status:
 *   AT+STATUS
 *   Response: AT+STATUS:RX_MODE or AT+STATUS:TX_MODE
 * 
 * Reset Device:
 *   AT+RST
 *   Response: AT+RESET (then reboot)
 */
