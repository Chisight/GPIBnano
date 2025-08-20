#include "GPIBnano.h"
// Define the global instance
GPIBnano gpibNano;

//#define GPIB_DEBUG //enable debugging

// --- Send Queue (FIFO) for individual bytes ---
uint8_t sendQueue[QUEUE_SIZE];
bool sendQueueIsHex[QUEUE_SIZE];
uint8_t queueHead = 0;
uint8_t queueTail = 0;
uint8_t queueCount = 0;

uint8_t controllerAddress = 0;

GpibState gpibState = GPIB_IDLE;
TalkerState talkerState = T_IDLE;

unsigned long listenTimeoutTimestamp = 0;

uint8_t initTargetAddress = 255;
unsigned long ifcPulseTimestamp = 0;

uint8_t lastTalker = 255;
uint8_t lastListener = 255;

char writeString[MAX_WRITE_STRING_LENGTH];

// --- Data Buffers for Display ---
#ifdef GPIB_DEBUG
  char sentData[MAX_WRITE_STRING_LENGTH];
  uint8_t sentDataIndex = 0;
#endif

char receivedData[MAX_RECEIVE_LENGTH];
int receivedDataIndex = 0; // Index to track where to write next in the array

bool resultReady = false;

/* --- main function to do everything but output --- */
void GPIBnano::processGPIB() {
  handleSerialInput(); // queue input, parse commands, and dispatch them.
  uint16_t currentPinStates = readGpibPins();
  gpibFSM(currentPinStates);
  updateTalkerFSM(currentPinStates);
}

bool GPIBnano::isResult() {
  return resultReady;
}

const char* GPIBnano::result() {
    if (resultReady) {
        resultReady = false;
        receivedDataIndex = 0; // Reset index for the next read
#ifdef GPIB_DEBUG
        Serial.print(F("LISTEN: Received from instrument: "));
#endif
        return receivedData; // Return the character array
    } else {
        const char* dummy = "";
        return dummy; // Return an empty string
    }
}

/**
 * @brief State machine to manage the low-level Talker handshake.
 * This version now correctly appends ASCII characters to the sentData buffer.
 */
void GPIBnano::updateTalkerFSM(uint16_t currentPinStates) {
  static uint8_t currentSendingByte = 0;
#ifdef GPIB_DEBUG
  static bool currentIsHex = false;
#endif
  switch (talkerState) {
    case T_IDLE: // 0
      if (queueCount > 0) {
        currentSendingByte = sendQueue[queueHead];
#ifdef GPIB_DEBUG
        currentIsHex = sendQueueIsHex[queueHead];
#endif
        queueHead = (queueHead + 1) % QUEUE_SIZE;
        queueCount--;
        setDioPins(currentSendingByte);
        talkerState = T_WAIT_NDAC_ASSERTED;
      }
      break;
    case T_WAIT_NDAC_ASSERTED: // 1
      if (getNDAC) {
        talkerState = T_WAIT_NRFD_RELEASED;
      }
      break;
    case T_WAIT_NRFD_RELEASED: // 2
      if (!getNRFD) {
        assertPin(DAV_PIN);
        talkerState = T_WAIT_NDAC_RELEASED;
      }
      break;
    case T_WAIT_NDAC_RELEASED: // 3
      if (!getNDAC) {
        releasePin(DAV_PIN);
#ifdef GPIB_DEBUG
        if (currentIsHex) {
          size_t currentLength = strlen(sentData);
          if (currentLength + 3 < MAX_WRITE_STRING_LENGTH) {
            sentData[currentLength] = (currentSendingByte >> 4) + ((currentSendingByte >> 4) < 10 ? '0' : 'A' - 10); //append high nibble in hex
            sentData[currentLength + 1] = (currentSendingByte & 0x0F) + ((currentSendingByte & 0x0F) < 10 ? '0' : 'A' - 10); //append low nibble in hex
            sentData[currentLength + 2] = ' '; //pad
            sentData[currentLength + 3] = '\0'; //terminate
          } else {
            Serial.print(F("Out of space in: sentData."));
          }
        } else {
          uint8_t len=strlen(sentData);
          sentData[len] = (char)currentSendingByte;
          sentData[len+1] = '\0';
        }
#endif
        talkerState = T_IDLE; // Handshake complete
      }
      break;
  }
}

/**
 * @brief Queues the standard sequence of GPIB command bytes to unaddress all
 *        devices and then assign a new Talker and a new Listener.
 * @note This function is now state-aware. It will do nothing if the requested
 *       talker and listener are already the active ones.
 * @note The ATN line must be asserted by the caller BEFORE this function is
 *       called and released AFTER the queued bytes have been sent.
 * @param talkerAddress The address (0-30) of the target Talker device.
 * @param listenerAddress The address (0-30) of the target Listener device.
 */
void GPIBnano::setTalkerListener(uint8_t talkerAddress, uint8_t listenerAddress) {
  // CHANGE: Check if the bus is already configured as requested.
  // If so, do nothing and return early to prevent redundant commands.
  if (talkerAddress == lastTalker && listenerAddress == lastListener) {
    return;
  }

  // If the state is different, proceed with re-addressing the bus.
  queueByte(0x3F, true); // UNL (Unlisten)
  queueByte(0x5F, true); // UNT (Untalk)

  uint8_t mta = 0x40 | talkerAddress;
  queueByte(mta, true); // MTA (My Talk Address)

  uint8_t mla = 0x20 | listenerAddress;
  queueByte(mla, true); // MLA (My Listen Address)

  // CHANGE: After queuing the new commands, update the stored state
  // to remember the new configuration.
  lastTalker = talkerAddress;
  lastListener = listenerAddress;
}

void GPIBnano::gpibFSM(uint16_t currentPinStates) {
  static bool eoi_was_detected = false;
  // talkerReady is true when the low-level Talker FSM is idle AND the send queue is empty.
  bool talkerReady = (talkerState == T_IDLE && queueCount == 0);
  
  // The timeout check now excludes all final cleanup states.
  if (gpibState < LISTEN_UNADDRESS_START_ATN && (millis() - listenTimeoutTimestamp > LISTEN_TIMEOUT_MS)) {
    Serial.println(F("ERROR: *LISTEN timed out after 3 seconds."));
    releasePin(ATN_PIN);
    gpibState = LISTEN_UNADDRESS_FINISH; // Force cleanup
    return;
  }

  switch (gpibState) {
// LISTEN state#endif

    // --- Phase 1: Configure the bus ---
    case LISTEN_SETUP_ADDRESSES:
      if (talkerReady) {
#ifdef GPIB_DEBUG
        Serial.println(F("LISTEN: Asserting ATN and setting addresses."));
#endif
        assertPin(ATN_PIN);
        setTalkerListener(initTargetAddress, controllerAddress);
        gpibState = LISTEN_BEGIN_HANDSHAKE;
      }
      break;
    case LISTEN_BEGIN_HANDSHAKE:
      if (talkerReady) {
#ifdef GPIB_DEBUG
        Serial.println(F("LISTEN: Releasing ATN and preparing handshake lines."));
#endif
        releasePin(ATN_PIN);
        assertPin(NRFD_PIN);
        assertPin(NDAC_PIN);
        eoi_was_detected = false;
        gpibState = LISTEN_READY_FOR_DATA;
      }
      break;
    // --- Phase 2: Perform the actual listening handshake ---
    case LISTEN_READY_FOR_DATA:
      releasePin(NRFD_PIN);
      gpibState = LISTEN_WAIT_FOR_DAV;
      break;
    case LISTEN_WAIT_FOR_DAV:
      if (getDAV) {
        gpibState = LISTEN_DATA_RECEIVED;
      }
      break;
    case LISTEN_DATA_RECEIVED:
    {
        uint8_t data = (currentPinStates & 0xff);
        eoi_was_detected = getEOI;
        if (receivedDataIndex < MAX_RECEIVE_LENGTH - 1) { // Check for buffer overflow
            receivedData[receivedDataIndex++] = (char)data; // Append data
            receivedData[receivedDataIndex] = '\0'; // Null-terminate the string
        }
        assertPin(NRFD_PIN);
        releasePin(NDAC_PIN);
        gpibState = LISTEN_WAIT_FOR_DAV_RELEASE;
    }
      break;
    case LISTEN_WAIT_FOR_DAV_RELEASE:
      if (!getDAV) {
        gpibState = LISTEN_FINISH_BYTE_HANDSHAKE;
      }
      break;
    case LISTEN_FINISH_BYTE_HANDSHAKE:
      assertPin(NDAC_PIN);
      if (eoi_was_detected) {
        gpibState = LISTEN_UNADDRESS_START_ATN;
      } else {
        gpibState = LISTEN_READY_FOR_DATA;
      }
      break;
    // --- Phase 3: Unaddress the talker ---
    case LISTEN_UNADDRESS_START_ATN:
      if (talkerReady) {
#ifdef GPIB_DEBUG
        Serial.println(F("LISTEN: Unaddressing talker."));
#endif
        assertPin(ATN_PIN);
        queueByte(0x5F, true); // UNT command
        gpibState = LISTEN_UNADDRESS_WAIT_FOR_DAV;
      }
      break;
    case LISTEN_UNADDRESS_WAIT_FOR_DAV:
      releasePin(NRFD_PIN);  // We signal we are ready for the UNT command byte.
      if (getDAV) { // Wait for our TalkerFSM to assert DAV.
        gpibState = LISTEN_UNADDRESS_ACK;
      }
      break;
    case LISTEN_UNADDRESS_ACK:
      releasePin(NDAC_PIN); // We acknowledge the UNT command byte. This un-sticks the TalkerFSM from T:3.
      if (!getDAV) { // Wait for our TalkerFSM to see the acknowledgement and release DAV.
        gpibState = LISTEN_UNADDRESS_WAIT_FOR_IDLE;
      }
      break;
    case LISTEN_UNADDRESS_WAIT_FOR_IDLE:
      if (talkerReady) {  // Now we wait for the TalkerFSM to fully complete its cycle and go idle.
        gpibState = LISTEN_UNADDRESS_FINISH;
      }
      break;
    case LISTEN_UNADDRESS_FINISH:
      if (talkerReady) {
#ifdef GPIB_DEBUG
        Serial.println(F("LISTEN: Releasing ATN. Sequence complete."));
#endif
        resultReady = true;
        releasePin(ATN_PIN);
        releasePin(NRFD_PIN);
        releasePin(NDAC_PIN);
        gpibState = GPIB_COMPLETE;
      }
      break;
// INIT states
    case INIT_PULSE_IFC_START:
#ifdef GPIB_DEBUG
      Serial.println(F("INIT: Asserting IFC."));
#endif
      assertPin(IFC_PIN);
      ifcPulseTimestamp = millis();
      gpibState = INIT_PULSE_IFC_WAIT;
      break;
    case INIT_PULSE_IFC_WAIT: // 2
      if (millis() - ifcPulseTimestamp >= 1) {
        gpibState = INIT_PULSE_IFC_END;
      }
      break;
    case INIT_PULSE_IFC_END:
#ifdef GPIB_DEBUG
      Serial.println(F("INIT: Releasing IFC."));
#endif
      releasePin(IFC_PIN);
      gpibState = INIT_ASSERT_REN_ATN;
      break;
    case INIT_ASSERT_REN_ATN:
#ifdef GPIB_DEBUG
      Serial.println(F("INIT: Asserting REN and ATN."));
#endif
      assertPin(REN_PIN);
      assertPin(ATN_PIN);
      gpibState = INIT_SEND_UNL;
      break;
    case INIT_SEND_UNL:
      if (talkerReady) {
#ifdef GPIB_DEBUG
        Serial.println(F("INIT: Sending UNL (0x3F)."));
#endif
        queueByte(0x3F, true);
        gpibState = INIT_SEND_UNT;
      }
      break;
    case INIT_SEND_UNT:
      if (talkerReady) {
#ifdef GPIB_DEBUG
        Serial.println(F("INIT: Sending UNT (0x5F)."));
#endif
        queueByte(0x5F, true);
        // CHANGE: After sending UNT, the command phase is done. Go to FINISH.
        gpibState = INIT_FINISH;
      }
      break;
    case INIT_FINISH:
      if (talkerReady) {
#ifdef GPIB_DEBUG
        Serial.println(F("INIT: Releasing ATN and DIO bus. Sequence complete."));
#endif
        releasePin(ATN_PIN);
        setDioPins(0x00);
        gpibState = GPIB_COMPLETE;
      }
      break;
// WRITE states
    case WRITE_SETUP_ADDRESSES:
      if (talkerReady) {
#ifdef GPIB_DEBUG
        Serial.println(F("WRITE: Asserting ATN and setting addresses."));
#endif
        assertPin(ATN_PIN);
        setTalkerListener(controllerAddress, initTargetAddress);
        gpibState = WRITE_SEND_BODY;
      }
      break;
    case WRITE_SEND_BODY:
      if (talkerReady) {  // This state waits for the address commands to be sent.
#ifdef GPIB_DEBUG
        Serial.println(F("WRITE: Releasing ATN and queuing string body."));
#endif
        releasePin(ATN_PIN);
        for (uint8_t i = 0; i < strlen(writeString) - 1; i++) { // Queue all characters except for the last one.
            queueByte((uint8_t)writeString[i]);
        }
        gpibState = WRITE_SEND_FINAL_CHAR;
      }
      break;
    case WRITE_SEND_FINAL_CHAR:
      if (talkerReady) {  // This state waits for the body of the string to be sent.
#ifdef GPIB_DEBUG
        Serial.println(F("WRITE: Asserting EOI and queuing final character."));
#endif
        assertPin(EOI_PIN);
        queueByte((uint8_t)writeString[strlen(writeString) - 1]);
        gpibState = WRITE_FINISH;
      }
      break;
    case WRITE_FINISH:
      if (talkerReady) {  // This state waits for the final character to be sent.
#ifdef GPIB_DEBUG
        Serial.println(F("WRITE: Releasing EOI and DIO bus. Sequence complete."));
#endif
        releasePin(EOI_PIN);
        setDioPins(0x00);
        gpibState = GPIB_COMPLETE;
      }
      break;
    case GPIB_COMPLETE:
#ifdef GPIB_DEBUG
      Serial.println(F("INFO: Command complete. Ready for next command."));
#endif
      gpibState = GPIB_IDLE;
      break;
    case GPIB_IDLE:
      break;
  }
#ifdef GPIB_DEBUG
  reportPinStates(currentPinStates);
#endif
}

/**
 * @brief Reads the current bus state and prints it to the Serial monitor ONLY if
 *        any state has changed since the last print. This prevents a constant
 *        stream of identical status lines, making the output much cleaner.
 *
 * @param gpibState The current state of the primary FSM.
 * @param talkerState The current state of the low-level Talker FSM.
 */
#ifdef GPIB_DEBUG
void GPIBnano::reportPinStates(uint16_t currentPinStates) {
  // --- Static variables to hold the state from the previous print ---
  static uint16_t previousPinStates = 0xFFFF;
  static uint8_t previousGpibState = 255;
  static uint8_t previousTalkerState = 255;
  static int previousSentDataLen = -1;
  static int previousRecvDataLen = -1;

  int currentSentDataLen = strlen(sentData);
  int currentRecvDataLen = receivedDataIndex; 

  // --- Compare current state to the previously printed state ---
  // The talkerState is now included in the comparison.
  if (currentPinStates == previousPinStates &&
      gpibState == previousGpibState &&
      talkerState == previousTalkerState && // New comparison
      currentSentDataLen == previousSentDataLen &&
      currentRecvDataLen == previousRecvDataLen)
  {
    return; // State is unchanged, do nothing.
  }

  // --- If we've reached this point, something has changed. Print the new state. ---
  Serial.print(F("DIO: "));
  Serial.print(getDIO8 ? '1' : '0'); // DIO8
  Serial.print(getDIO7 ? '1' : '0'); // DIO7
  Serial.print(getDIO6 ? '1' : '0'); // DIO6
  Serial.print(getDIO5 ? '1' : '0'); // DIO5
  Serial.print(getDIO4 ? '1' : '0'); // DIO4
  Serial.print(getDIO3 ? '1' : '0'); // DIO3
  Serial.print(getDIO2 ? '1' : '0'); // DIO2
  Serial.print(getDIO1 ? '1' : '0'); // DIO1

  Serial.print(F(" | HS:"));
  Serial.print(getDAV ? F(" DAV") : F(" dav"));
  Serial.print(getNRFD ? F(" NRFD") : F(" nrfd"));
  Serial.print(getNDAC ? F(" NDAC") : F(" ndac"));

  Serial.print(F(" | MGMT:"));
  Serial.print(getEOI ? F(" EOI") : F(" eoi"));
  Serial.print(getIFC ? F(" IFC") : F(" ifc"));
  Serial.print(getATN ? F(" ATN") : F(" atn"));
  Serial.print(getREN ? F(" REN") : F(" ren"));
  Serial.print(getSRQ ? F(" SRQ") : F(" srq"));

  // Append the Talker FSM state, since it's always relevant.
  Serial.print(F(" (T:"));
  Serial.print(talkerState);
  Serial.print(F(")"));


  if (currentSentDataLen > 0) {
    Serial.print(F(" | SENT: "));
    Serial.print(sentData);
  }
  if (currentRecvDataLen > 0) {
    Serial.print(F(" | RECV: "));
    Serial.print(receivedData);
  }
  Serial.println();

  previousPinStates = currentPinStates;
  previousGpibState = gpibState;
  previousTalkerState = talkerState; // New state to save
  previousSentDataLen = currentSentDataLen;
  previousRecvDataLen = currentRecvDataLen;
}
#endif

/**
 * @brief Queues a byte for sending via the Talker FSM.
 * @param data The 8-bit value to send.
 * @param isHex If true, the byte will be displayed as hex in the status monitor.
 *              If false (default), it will be displayed as an ASCII character.
 */
void GPIBnano::queueByte(uint8_t data, bool isHex __attribute__((unused))) {
  if (queueCount < QUEUE_SIZE) {
#ifdef GPIB_DEBUG
    if (talkerState == T_IDLE && queueCount == 0) { sentData[0] = '\0'; }
    sendQueueIsHex[queueTail] = isHex;
    Serial.print(F("CMD: Queued 0x")); 
    if (data < 0x10) Serial.print('0');
    Serial.println(data, HEX);
#endif
    sendQueue[queueTail] = data;
    queueTail = (queueTail + 1) % QUEUE_SIZE;
    queueCount++;
  } else { 
    Serial.println(F("ERR: Send queue is full!"));
  }
}

void GPIBnano::setDioPins(uint8_t data) {
    if (data & (1 << DIO8_BIT)) { assertPin(DIO8_PIN); } else { releasePin(DIO8_PIN); }
    if (data & (1 << DIO7_BIT)) { assertPin(DIO7_PIN); } else { releasePin(DIO7_PIN); }
    if (data & (1 << DIO6_BIT)) { assertPin(DIO6_PIN); } else { releasePin(DIO6_PIN); }
    if (data & (1 << DIO5_BIT)) { assertPin(DIO5_PIN); } else { releasePin(DIO5_PIN); }
    if (data & (1 << DIO4_BIT)) { assertPin(DIO4_PIN); } else { releasePin(DIO4_PIN); }
    if (data & (1 << DIO3_BIT)) { assertPin(DIO3_PIN); } else { releasePin(DIO3_PIN); }
    if (data & (1 << DIO2_BIT)) { assertPin(DIO2_PIN); } else { releasePin(DIO2_PIN); }
    if (data & (1 << DIO1_BIT)) { assertPin(DIO1_PIN); } else { releasePin(DIO1_PIN); }
}

void GPIBnano::setControlPins(uint8_t data) {
  if (data & (1 << DAV_BIT)) { assertPin(DAV_PIN); } else { releasePin(DAV_PIN); }
  if (data & (1 << NRFD_BIT)) { assertPin(NRFD_PIN); } else { releasePin(NRFD_PIN); }
  if (data & (1 << NDAC_BIT)) { assertPin(NDAC_PIN); } else { releasePin(NDAC_PIN); }
  if (data & (1 << EOI_BIT)) { assertPin(EOI_PIN); } else { releasePin(EOI_PIN); }
  if (data & (1 << IFC_BIT)) { assertPin(IFC_PIN); } else { releasePin(IFC_PIN); }
  if (data & (1 << ATN_BIT)) { assertPin(ATN_PIN); } else { releasePin(ATN_PIN); }
  if (data & (1 << REN_BIT)) { assertPin(REN_PIN); } else { releasePin(REN_PIN); }
  if (data & (1 << SRQ_BIT)) { assertPin(SRQ_PIN); } else { releasePin(SRQ_PIN); }
}


uint16_t GPIBnano::readGpibPins() {
    uint16_t data = 0;
    data |= isAsserted(DIO1_PIN) ? (1 << DIO1_BIT) : 0;
    data |= isAsserted(DIO2_PIN) ? (1 << DIO2_BIT) : 0;
    data |= isAsserted(DIO3_PIN) ? (1 << DIO3_BIT) : 0;
    data |= isAsserted(DIO4_PIN) ? (1 << DIO4_BIT) : 0;
    data |= isAsserted(DIO5_PIN) ? (1 << DIO5_BIT) : 0;
    data |= isAsserted(DIO6_PIN) ? (1 << DIO6_BIT) : 0;
    data |= isAsserted(DIO7_PIN) ? (1 << DIO7_BIT) : 0;
    data |= isAsserted(DIO8_PIN) ? (1 << DIO8_BIT) : 0;
    data |= isAsserted(DAV_PIN)  ? (1 << DAV_BIT) : 0;
    data |= isAsserted(NRFD_PIN) ? (1 << NRFD_BIT) : 0;
    data |= isAsserted(NDAC_PIN) ? (1 << NDAC_BIT): 0;
    data |= isAsserted(EOI_PIN)  ? (1 << EOI_BIT): 0;
    data |= isAsserted(IFC_PIN)  ? (1 << IFC_BIT): 0;
    data |= isAsserted(ATN_PIN)  ? (1 << ATN_BIT): 0;
    data |= isAsserted(REN_PIN)  ? (1 << REN_BIT): 0;
    data |= isAsserted(SRQ_PIN)  ? (1 << SRQ_BIT): 0;
    return data;
}

void GPIBnano::toUpperCase(char* str) {
    for (int i = 0; str[i]; i++) {
        str[i] = toupper(str[i]);
    }
}

/**
 * @brief Parses a complete command line and starts the appropriate state machine.
 */
void GPIBnano::executeHighLevelCommand(char* cmdLine) {
    cmdLine++; // Skip the leading '*'
    while (isspace(*cmdLine)) { cmdLine++; } // Trim leading spaces
#ifdef GPIB_DEBUG
    Serial.print(F("CMD EXECUTING: "));
    Serial.println(cmdLine);
#endif
    if (cmdLine[0] == '\0') { // Check if the command is empty after trimming
        Serial.println(F("ERROR: Command must not be empty."));
        return;
    }
    const char* argument = ""; // Initialize to an unwritable dummy empty string
    for (int i = 0; cmdLine[i] != '\0'; i++) { // Loop through the command line to find the first space
        if (cmdLine[i] == ' ') {
            cmdLine[i] = '\0'; // Replace space with null terminator
            argument = cmdLine + i + 1; // Point argument just past the null
            break;
        }
    }
    toUpperCase(cmdLine); // Convert command to upper case
    // Process the command
    if (strcmp_P(cmdLine, PSTR("INIT")) == 0) {
        int addr = atoi(argument);
        if (addr > 0 && addr <= 30) {
            initTargetAddress = (uint8_t)addr;
#ifdef GPIB_DEBUG
            sentData[0] = '\0'; // Clear sentData
#endif
            gpibState = INIT_PULSE_IFC_START;
        } else {
            Serial.println(F("ERROR: Invalid GPIB address for *INIT."));
        }
    } else if (strcmp_P(cmdLine, PSTR("WRITE")) == 0) {
        if (strlen(argument) > 0) {
            strncpy(writeString, argument, MAX_WRITE_STRING_LENGTH - 1);
            writeString[MAX_WRITE_STRING_LENGTH - 1] = '\0'; // Ensure null-termination
#ifdef GPIB_DEBUG
            sentData[0] = '\0'; // Clear sentData
#endif
            gpibState = WRITE_SETUP_ADDRESSES;
        } else {
            Serial.println(F("ERROR: *WRITE command received with no string."));
        }
    } else if (strcmp_P(cmdLine, PSTR("LISTEN")) == 0) {
        if (initTargetAddress > 30) {
            Serial.println(F("ERROR: Must run *INIT <addr> before *LISTEN."));
            return;
        }
        receivedData[0] = '\0'; // Clear receivedData
        receivedDataIndex = 0;
        resultReady = false;
        gpibState = LISTEN_SETUP_ADDRESSES;
        listenTimeoutTimestamp = millis();
    } else {
        Serial.print(F("ERROR: Unknown command: "));
        Serial.println(cmdLine);
    }
}

/**
 * @brief Reads from the serial port until a complete line or comma is received.
 */
void GPIBnano::handleSerialInput() {
  static char serialCommandBuffer[MAX_COMMAND_LENGTH]; // Static buffer to retain data
  static int commandIndex = 0; // Static index to retain state

  while (Serial.available() > 0 && gpibState == GPIB_IDLE) {
    char receivedChar = Serial.read();
    
    if (receivedChar == '\n' || receivedChar == '\r' || receivedChar == ',') {
      if (commandIndex > 0) {
        serialCommandBuffer[commandIndex] = '\0'; // Null-terminate the string
        
        if (strncmp(serialCommandBuffer, "*", 1) == 0) {
          executeHighLevelCommand(serialCommandBuffer);
        } else {
          Serial.println(F("ERROR: All commands must start with '*'."));
        }
        
        commandIndex = 0; // Reset the index for the next command
      }
    } else {
      if (commandIndex < MAX_COMMAND_LENGTH - 1) { // Ensure there's space for the null terminator
        serialCommandBuffer[commandIndex++] = receivedChar; // Add the character to the buffer
      } else {
        Serial.println(F("ERROR: Command too long."));
        commandIndex = 0; // Reset the index if the command is too long
      }
    }
  }
}

void GPIBnano::begin(uint8_t ctrlAddress) {
  controllerAddress = ctrlAddress;
  setDioPins(0x00);
  setControlPins(0x00);
#ifdef GPIB_DEBUG
  Serial.println(F("\n--- GPIB Low-Level Protocol Driver ---"));
  Serial.println(F("Ready for commands (e.g., *INIT 22)."));
  Serial.println(F("------------------------------------------------------------------"));
#endif
}
