#ifndef GPIBNano_H
#define GPIBNano_H
#include <Arduino.h> // Include Arduino core library for types

// --- Pin Bitmasks & Argument Macros ---
// These defines are sorted by GPIB function. They provide the arguments for

//  --- Pin Mapping (Arduino Nano -> GPIB) ---
// --- GPIB Data Lines (DIO1-DIO8) ---
#define DIO1_PIN  12
#define DIO2_PIN  10
#define DIO3_PIN  14
#define DIO4_PIN  15
#define DIO5_PIN  11
#define DIO6_PIN   9
#define DIO7_PIN   8
#define DIO8_PIN   7

// --- GPIB Handshake Lines ---
#define DAV_PIN   17
#define NRFD_PIN   5
#define NDAC_PIN  18

// --- GPIB Management Lines ---
#define EOI_PIN   16
#define IFC_PIN    4
#define ATN_PIN    2
#define REN_PIN    6
#define SRQ_PIN    3

// --- Define the digitalWriteFast/igitalReadFast/pinModeFast macros ---
#define digitalPinToPortReg(P) (((P) >= 0 && (P) <= 7) ? &PORTD : (((P) >= 8 && (P) <= 13) ? &PORTB : &PORTC))
#define digitalPinToPINReg(P) (((P) >= 0 && (P) <= 7) ? &PIND : (((P) >= 8 && (P) <= 13) ? &PINB : &PINC))
#define digitalPinToBit(P) (((P) >= 0 && (P) <= 7) ? (P) : (((P) >= 8 && (P) <= 13) ? (P) - 8 : (P) - 14))
#define digitalPinToDDRReg(P) (((P) >= 0 && (P) <= 7) ? &DDRD : (((P) >= 8 && (P) <= 13) ? &DDRB : &DDRC))
#define pinModeFast(P, V) bitWrite(*digitalPinToDDRReg(P), digitalPinToBit(P), (V))
#define digitalWriteFast(P, V) bitWrite(*digitalPinToPortReg(P), digitalPinToBit(P), (V))
#define digitalReadFast(P) bitRead(*digitalPinToPINReg(P), digitalPinToBit(P))


/* GPIB uses open collector outputs/inputs.  This means pulling a pin LOW (asserting) is active or 1, and 
releasing a pin will allow it to go HIGH or 0.  This negative logic can be confusing so we use the 
Assert/Release nomenclature to avoid confusion.
*/
#define isAsserted(pin) (digitalReadFast(pin) == LOW) // LOW means asserted

#define assertPin(pin) { \
    digitalWriteFast(pin, LOW); \
    pinModeFast(pin, OUTPUT); \
}

#define releasePin(pin) { \
    pinModeFast(pin, INPUT); \
    digitalWriteFast(pin, HIGH); \
}

// --- Bit position definitions for the packed 16-bit bus state variable ---
// These map a pin name to its bit number (0-15) in the state variable.
#define DIO1_BIT  0
#define DIO2_BIT  1
#define DIO3_BIT  2
#define DIO4_BIT  3
#define DIO5_BIT  4
#define DIO6_BIT  5
#define DIO7_BIT  6
#define DIO8_BIT  7
#define DAV_BIT   8
#define NRFD_BIT  9
#define NDAC_BIT 10
#define EOI_BIT  11
#define IFC_BIT  12
#define ATN_BIT  13
#define REN_BIT  14
#define SRQ_BIT  15

// --- Helper macros for checking the state of a specific pin ---
// These macros use the 16bit currentPinStates global which is set using readGpibPins().
// They return a non-zero value (true) if the pin is asserted, and 0 (false) if not.
#define getDIO1 (currentPinStates & (1 << DIO1_BIT))
#define getDIO2 (currentPinStates & (1 << DIO2_BIT))
#define getDIO3 (currentPinStates & (1 << DIO3_BIT))
#define getDIO4 (currentPinStates & (1 << DIO4_BIT))
#define getDIO5 (currentPinStates & (1 << DIO5_BIT))
#define getDIO6 (currentPinStates & (1 << DIO6_BIT))
#define getDIO7 (currentPinStates & (1 << DIO7_BIT))
#define getDIO8 (currentPinStates & (1 << DIO8_BIT))
#define getDAV  (currentPinStates & (1 << DAV_BIT))
#define getNRFD (currentPinStates & (1 << NRFD_BIT))
#define getNDAC (currentPinStates & (1 << NDAC_BIT))
#define getEOI  (currentPinStates & (1 << EOI_BIT))
#define getIFC  (currentPinStates & (1 << IFC_BIT))
#define getATN  (currentPinStates & (1 << ATN_BIT))
#define getREN  (currentPinStates & (1 << REN_BIT))
#define getSRQ  (currentPinStates & (1 << SRQ_BIT))


#define LISTEN_TIMEOUT_MS 3000
// note, we use conservative buffer sizes because they are static in RAM.
// some applications may need larger buffers.
#define MAX_COMMAND_LENGTH 32 // Define a maximum command length
#define MAX_WRITE_STRING_LENGTH (MAX_COMMAND_LENGTH - strlen("*WRITE ")) //max parameter for *WRITE command
#define MAX_RECEIVE_LENGTH 32 // length of receive buffer for *LISTEN
#define QUEUE_SIZE MAX_WRITE_STRING_LENGTH // --- Send Queue (FIFO) ---

// --- State Machine Definitions ---
enum TalkerState {
  T_IDLE,
  T_WAIT_NDAC_ASSERTED,
  T_WAIT_NRFD_RELEASED,
  T_WAIT_NDAC_RELEASED
};

enum GpibState {
// LISTEN states (must come first to simplify timeout)
  // Phase 1: Configure the bus
  LISTEN_SETUP_ADDRESSES, // 0
  LISTEN_BEGIN_HANDSHAKE, // 1
  // Phase 2: Perform the data reception handshake loop
  LISTEN_READY_FOR_DATA, // 2
  LISTEN_WAIT_FOR_DAV, // 3
  LISTEN_DATA_RECEIVED, // 4
  LISTEN_WAIT_FOR_DAV_RELEASE, // 5
  LISTEN_FINISH_BYTE_HANDSHAKE, // 6
  // Phase 3: Unaddress the talker to clean up the bus
  LISTEN_UNADDRESS_START_ATN, // 7
  // New states for the self-handshake to prevent deadlock
  LISTEN_UNADDRESS_WAIT_FOR_DAV, // 8
  LISTEN_UNADDRESS_ACK, // 9
  LISTEN_UNADDRESS_WAIT_FOR_IDLE, // 10
  LISTEN_UNADDRESS_FINISH, // 11
// WRITE states
  WRITE_SETUP_ADDRESSES, // 12
  WRITE_SEND_BODY, // 13
  WRITE_SEND_FINAL_CHAR, // 14
  WRITE_FINISH, // 15
// INIT states
  INIT_PULSE_IFC_START, // 16
  INIT_PULSE_IFC_WAIT, // 17
  INIT_PULSE_IFC_END, // 18
  INIT_ASSERT_REN_ATN, // 19
  INIT_SEND_UNL, // 20
  INIT_SEND_UNT, // 21
  INIT_FINISH, // 22
  GPIB_COMPLETE, // 23
  GPIB_IDLE // 24
};

class GPIBnano {
public:
    void begin(uint8_t ctrlAddress = 0);
    void processGPIB();
    bool isResult();
    const char* result();
private:
    void updateTalkerFSM(uint16_t currentPinStates);
    void gpibFSM(uint16_t currentPinStates);
    void setTalkerListener(uint8_t talkerAddress, uint8_t listenerAddress);
    void toUpperCase(char* str);
    void executeHighLevelCommand(char* cmdLine);
    void reportPinStates(uint16_t currentPinStates);
    void queueByte(uint8_t data, bool isHex = false);
    uint16_t readGpibPins();
    void setDioPins(uint8_t data);
    void setControlPins(uint8_t data);
    void handleSerialInput();
    };

extern GPIBnano gpibNano;

#endif
