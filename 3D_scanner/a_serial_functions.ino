/*  SKYE-AUSTIN PROTOCOL v1
 *  This protocol specifies a standard format for interacting with the 3D scanner over Serial.
 *  This implementation is taylored to be stable on an embedded system, minimizing use of both the stack and heap.
 *  As a learning exercise, we intentionally avoided using Arduino's String class because, even though it would
 *  have made this a lot easier, it is prone to causing heap fragmentation and so is not good practice.
 * 
 *  To get this to work:
 *  1. call configure_serial() in void setup() [do not also call Serial.begin()]
 *  2. call update_rx() in each void loop()
 *
 *  To send the Arduino commands without arguments, use the format:
 *  [packet_initiator][command][packet_terminator]
 *  ex: @PING;
 *
 *  To send the Arduino commands with arguments, use the format:
 *  [packet_initiator][command][args_initiator][arg1][args_separator][arg2]....[argN][args_terminator][packet_terminator]
 *  ex: @GOTO{1234,5678};
 *  [do not include anything but the args_separator between arguments]
 *
 *  To send the computer messages from the Arduino, call send_message(header_text, body_text)
 *  [both message_header and message_body must be char arrays (also called c-strings)]
 *  
 *  The Arduino sends messages to the computer in a comparable format:
 *  [packet_initiator][header][packet_terminator]
 *  ex: @SCANSTART;
 *  OR
 *  [packet_initiator][header][args_initiator][body][args_terminator][packet_terminator]
 *  ex: @ACK{GOTO};
*/ 

// this is where you specify the protocol's syntax
const char packet_initiator = '@';
const char packet_terminator = ';';
const char args_initiator = '{';
const char args_terminator = '}';
const char args_separator = ',';

// this is where you specify various buffer sizes
// try to make these buffer sizes as small as possible given your intended commands and responses
const uint8_t max_rx_cmd_len = 8; // the maximum allowed length (in chars) of any incoming command name
const uint8_t max_rx_args_num = 4; // the maximum allowed number of arguments in any incoming packet
const uint8_t max_rx_args_len = 5; // the maximum allowed length (in chars) of any incoming argument
const uint8_t max_tx_header_len = 14; // the maximum allowed length (in chars) of the header of any outgoing packet
const uint8_t max_tx_body_len = 30; // the maximum allowed length (in chars) of the body of any outgoing packet

// this is where all the buffers get declared... no touchy
char rx_byte; // a buffer for storing the incoming byte from Serial
bool receiving_packet; // a flag to tell whether incoming bytes should be treated as part of a packet
char rx_cmd_buffer[max_rx_cmd_len + 1]; // a c-string buffer to hold the command portion of the incoming packet
uint8_t rx_cmd_buffer_len; // the length of the rx_cmd_buffer in bytes, NOT including the null terminator
uint8_t rx_cmd_buffer_pos; // a variable to keep track of the next available position in the rx_cmd_buffer
bool receiving_args; // a flag to tell whether incoming bytes should be treated as parts of arguments
char rx_args_buffer[max_rx_args_num][max_rx_args_len + 1]; // an array of c-string buffers to hold the incoming arguments
                                                           // each c-string buffer holds the chars that compose 1 argument
                                                           // when new chars are placed at the end of the c-string buffer,
                                                           // existing chars are pushed up one slot
                                                           // this makes parsing the c-string into an int much easier
uint8_t rx_args_buffer_arglen; // the number of columns/c-string buffers in the rx_args_buffer
uint8_t rx_args_buffer_diglen; // the length of the c-strings in rx_args_buffer in bytes, w/o null term.
uint8_t rx_args_buffer_argpos; // a variable to keep track of the next available column in rx_args_buffer
uint8_t num_rx_cmds; // the number of different incoming commands to listen for

char tx_header_buffer[max_tx_header_len + 1]; // a c-string buffer to hold the header text of the outgoing packet
char tx_body_buffer[max_tx_body_len + 1]; // a c-string buffer to hold the body text of the outgoing packet


// this is where you can specify some standard headers for messages you send to the computer
const char startup_header[] = "JOHNNY5ISALIVE";
const char ack_header[] = "ACK"; // required
const char error_header[] = "ERR"; // required
const char indv_reading_header[] = "READING";
const char scan_start_header[] = "SCANSTART";
const char scan_stop_header[] = "SCANSTOP";
const char scan_reading_header[] = "";

struct Cmd_Binding {
  char cmd_cstr[max_rx_cmd_len + 1];
  void (*handler_func)(void);
};

// this is where you specify what commands you are listening for from the computer,
// as well as what function you want to be called when each command is received
// we recommend using an intermediate handler function that calls the actual function after
// extracting arguments from rx_args_buffer 
const struct Cmd_Binding rx_cmd_bindings[] = {
  {"PING"    , handle_ping},
  {"GOTO"    , handle_goto},
  {"GOTOABS" , handle_gotoabs},
  {"MOVE"    , handle_gotorel},
  {"GOHOME"  , handle_gohome},
  {"SETHOME" , handle_sethome},
  {"CLRHOME" , handle_clearhome},
  {"RAW"     , handle_raw},
  {"SMOOTH"  , handle_smooth},
  {"READ"    , handle_read},
  {"DOSCAN"  , handle_doscan}
};

// and this is where you define those functions
void handle_ping() {};
void handle_goto() {goto_pos(atoi(rx_args_buffer[0]), atoi(rx_args_buffer[1]));}
void handle_gotoabs() {goto_abs_pos(atoi(rx_args_buffer[0]), atoi(rx_args_buffer[1]));}
void handle_gotorel() {goto_rel_pos(atoi(rx_args_buffer[0]), atoi(rx_args_buffer[1]));}
void handle_gohome() {gohome();}
void handle_sethome() {sethome();}
void handle_clearhome() {clearhome();}
void handle_raw() {disable_sensor_smoothing();}
void handle_smooth() {enable_sensor_smoothing();}
void handle_read() {
  static char temp_body_buffer[max_tx_body_len + 1];
  memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
  snprintf(temp_body_buffer, max_tx_body_len, "%d", read_sensor());
  send_message(indv_reading_header, temp_body_buffer);
}
void handle_doscan() {start_scan(atoi(rx_args_buffer[0]), atoi(rx_args_buffer[1]), atoi(rx_args_buffer[2]), atoi(rx_args_buffer[3]));}

bool busy = false;

// ^ global variables
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// v functions

void configure_serial(uint16_t baudrate) {
  rx_cmd_buffer_len = (sizeof(rx_cmd_buffer) / sizeof(rx_cmd_buffer[0])) - 1;
  rx_args_buffer_arglen = sizeof(rx_args_buffer) / sizeof(rx_args_buffer[0]);
  rx_args_buffer_diglen = (sizeof(rx_args_buffer[0]) / sizeof(rx_args_buffer[0][0])) - 1;
  num_rx_cmds = sizeof(rx_cmd_bindings) / sizeof(struct Cmd_Binding);

  reset_rx_buffers();
  reset_tx_buffers();

  receiving_packet = false;
  receiving_args = false;
  
  Serial.begin(baudrate);
}

void update_rx() {
  while (Serial.available()) {
    rx_byte = Serial.read();

    switch (receiving_packet) {
      case true:
        switch (receiving_args) {
          case true:
            // listen for args_separator and args_terminator, put everything else in rx_args_buffer
            switch (rx_byte) {
              case args_separator:
                rx_args_buffer_argpos++;
                if (rx_args_buffer_argpos >= rx_args_buffer_arglen) {
                  static char temp_body_buffer[max_tx_body_len + 1];
                  memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
                  snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("RX_ARGS_BUFFER OVERFLOW. Buffer len: %d, Write pos: %d"), rx_args_buffer_arglen, rx_args_buffer_argpos);
                  send_message(error_header, temp_body_buffer);
                  reset_rx_buffers();
                }
                break;
              case args_terminator:
                receiving_args = false;
                break;
              default:
                if ((!isDigit(rx_byte)) && rx_byte != '-') {
                  static char temp_body_buffer[max_tx_body_len + 1];
                  memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
                  snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Invalid char in args: %c"), rx_byte);
                  send_message(error_header, temp_body_buffer);
                  reset_rx_buffers();
                }
                for (int d=0; d < rx_args_buffer_diglen - 1; d++) {
                  rx_args_buffer[rx_args_buffer_argpos][d] = rx_args_buffer[rx_args_buffer_argpos][d+1];
                }
                rx_args_buffer[rx_args_buffer_argpos][rx_args_buffer_diglen - 1] = rx_byte;
                break;
            }
            break;
          default:
            // listen for args_initiator and packet_terminator, put everything else in rx_cmd_buffer
            switch (rx_byte) {
              case args_initiator:
                // set receiving_args flag
                receiving_args = true;
                break;
              case packet_terminator:
                // clear receiving_packet and receiving_args flags, kick off response to instruction
                receiving_packet = false;
                receiving_args = false;
                handle_rx_packet();
                break;
              default:
                // write the received char to the rx_cmd_buffer 
                if (!isUpperCase(rx_byte)) {
                  static char temp_body_buffer[max_tx_body_len + 1];
                  memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
                  snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Invalid char in cmd: %c"), rx_byte);
                  send_message(error_header, temp_body_buffer);
                  reset_rx_buffers();
                }
                if (rx_cmd_buffer_pos < rx_cmd_buffer_len) {
                  rx_cmd_buffer[rx_cmd_buffer_pos] = rx_byte;
                  rx_cmd_buffer_pos++;
                } else {
                  static char temp_body_buffer[max_tx_body_len + 1];
                  memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
                  snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("RX_CMD_BUFFER OVERFLOW. Buffer len: %d, Write pos: %d"), rx_cmd_buffer_len, rx_cmd_buffer_pos);
                  send_message(error_header, temp_body_buffer);
                  reset_rx_buffers();
                }
                break;
              }
              break;
        }
        break;
    }
    // listen for packet initiator
    switch (rx_byte) {
      case packet_initiator:
        reset_rx_buffers();
        receiving_packet = true;
        receiving_args = false;
        break;
    }
  
  }
}

void reset_rx_buffers() {
  memset(rx_cmd_buffer, NULL, sizeof(rx_cmd_buffer));
  memset(rx_args_buffer, ' ', sizeof(rx_args_buffer));
  rx_cmd_buffer[rx_cmd_buffer_len] = NULL;
  for (int a=0; a < rx_args_buffer_arglen; a++) {
    rx_args_buffer[a][rx_args_buffer_diglen] = NULL;
  }

  rx_cmd_buffer_pos = 0;
  rx_args_buffer_argpos = 0;

  receiving_packet=false;
  receiving_args=false;
  
  return;
}

void handle_rx_packet() {
  for (int c=0; c <= num_rx_cmds; c++) {
    if (c == num_rx_cmds) {
      static char temp_body_buffer[max_tx_body_len + 1];
      memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
      snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Unrecognized command: %s"), rx_cmd_buffer);
      send_message(error_header, temp_body_buffer);
    } else if (strcmp(rx_cmd_buffer, rx_cmd_bindings[c].cmd_cstr) == 0) {      
      if (!busy) {
        send_message(ack_header, rx_cmd_buffer);
        rx_cmd_bindings[c].handler_func();
      } else {
        static char temp_body_buffer[max_tx_body_len + 1];
        memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
        snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Scanner is busy."));
        send_message(error_header, temp_body_buffer);
      }
      break;
    }
  }
  return;
}

void send_message(const char *header_text, const char *body_text) {
  reset_tx_buffers();

  if (strlen(header_text) > max_tx_header_len) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Message header exceeds max allowed length: %d"), max_tx_header_len);
    send_message(error_header, temp_body_buffer);
    return;
  }
  
  if (strlen(body_text) > max_tx_body_len) {
    static char temp_body_buffer[max_tx_body_len + 1];
    memset(temp_body_buffer, NULL, sizeof(temp_body_buffer));
    snprintf_P(temp_body_buffer, max_tx_body_len, PSTR("Message body exceeds max allowed length: %d"), max_tx_body_len);
    send_message(error_header, temp_body_buffer);
    return;
  }

  strcpy(tx_header_buffer, header_text);
  strcpy(tx_body_buffer, body_text);

  send_tx_packet();
  return;
}

void reset_tx_buffers() {
  memset(tx_header_buffer, NULL, sizeof(tx_header_buffer));
  memset(tx_body_buffer, NULL, sizeof(tx_body_buffer));
  return;
}

void send_tx_packet() {
  Serial.print(packet_initiator);
  Serial.print(tx_header_buffer);
  if (strlen(tx_body_buffer) > 0) {
    Serial.print(args_initiator);
    Serial.print(tx_body_buffer);
    Serial.print(args_terminator);
  }
  Serial.print(packet_terminator);
  Serial.println();
  
  return;
}
