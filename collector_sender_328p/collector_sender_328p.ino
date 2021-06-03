/*
 * pin assignments:
 * stepper step --> PD2
 * stepper dir --> PD3
 * encoder A --> PD4
 * encoder B --> PD5
 */

#define STEP_PIN       PIND & (1 << PIND2)
#define DIR_PIN        PIND & (1 << PIND3)
#define ENCODER_A_PIN  PIND & (1 << PIND4)
#define ENCODER_B_PIN  PIND & (1 << PIND5)

#define BUFFER_SIZE         128// valid values are 2,4,8,16,32, and 64

#define GOOD_ACK_BYTE       0xAA
#define BREAK_MSG           0xAA,0x7F,0xFF,0x7F,0xFF,0xAA
#define CRC_POLYNOMIAL      0x83

//variables for interrupts
volatile int encoderBuffer[BUFFER_SIZE];
volatile int stepperBuffer[BUFFER_SIZE];
volatile byte dataBufferTracker[BUFFER_SIZE];
volatile bool encoderALastState;
volatile int encoderPos = 0;
volatile int stepperPos = 0;
volatile bool bufferOverflow = 0;
volatile bool encoderOverflow = 0;
volatile bool stepperOverflow = 0;
volatile byte dataIndex = 0;
byte resendOnError = 0;

//byte crcLookup[256];

void setup() {
  Serial.begin(250000);
  while(Serial.available()) {
    Serial.read();
  }
  Serial.write(0xAA);
  while (Serial.available() < 2) {
    //do nothing
  }

  byte timerInterval = Serial.read();
  timerInterval = constrain(timerInterval, 1, 250);
  Serial.write(timerInterval);
  
  resendOnError = Serial.read();
  
  while (Serial.available()) {
    Serial.read();
  }
  delay(500);

  //initialize buffers
  for (byte i = 0; i < BUFFER_SIZE; i++) {
    encoderBuffer[i] = 0;
    stepperBuffer[i] = 0;
    dataBufferTracker[i] = 0;
  }

  //set up timer 1
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 250 * timerInterval; // set compare match register
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10); //set bits for prescaler: clk/64

  // set up pin interrupts
  DDRD   = B00000000;  //port D pins as inputs
  PORTD  = B00000000;  // no pullups
  EICRA  = B00000011;  //rising edge decection on PD2 (INT0)
  PCICR  |= B00000100;  //enable pin change interrupts on PORTD
  sei();  

  //initialize encoder state
  encoderALastState = ENCODER_A_PIN;
  
  //mask inturrupts
  TCNT1 = 0;  //reset timer
  TIMSK1 |= (1 << OCIE1A);  //mask timer interrupt
  EIMSK  |= (1 << INT0);  //mask external interrupt on D2
  PCMSK2 |= (1 << PCINT20)| (1 << PCINT21); //mask pin change interrupt on PD4 and PD5
}

void loop() {
  static byte txIndex = 0;
  static byte rxIndex= 0;
  static byte msgBuilt = 0;
  static byte msg[6] = {0, 0, 0, 0, 0, 0};

//check for buffer overflow
  if (bufferOverflow || encoderOverflow || stepperOverflow) OverflowHandler();

//check for acks
if (resendOnError) {
    if (Serial.available()) {
      byte ack = Serial.read();
      if (ack == GOOD_ACK_BYTE) {
        cli();
        dataBufferTracker[rxIndex] = 0;
        sei();
        if (rxIndex == BUFFER_SIZE - 1) rxIndex = 0;
        else rxIndex++;
      }
      else {
        msgBuilt = 0;  //will need to rebuild previous message
        txIndex = rxIndex;
      }
    }
  }

//send message if message is ready
  if (msgBuilt) {
    Serial.write(msg,6);
    msgBuilt = 0;
    
    if (!resendOnError) {
      cli();
      dataBufferTracker[txIndex] = 0;
      sei();
    }
    
    if (txIndex == (BUFFER_SIZE - 1)) txIndex = 0;
    else txIndex++;
  }

// build the message
  if (!msgBuilt) {
    cli();
    byte msgIndex = dataIndex;
    sei();
    if (txIndex != msgIndex) {
      msg[0] = txIndex;
      msg[1] = (unsigned int)encoderBuffer[txIndex] >> 8,  //cast to unsigned int to prevent sign extension on shifting
      msg[2] = encoderBuffer[txIndex],
      msg[3] = (unsigned int)stepperBuffer[txIndex] >> 8,
      msg[4] = stepperBuffer[txIndex];
      byte crc = 0;
      for (byte currByte = 0; currByte < 5; currByte++) {
        crc ^= msg[currByte];
        for (byte i = 0; i < 8; i++) {
          if ((crc & 0x80) != 0) {
            crc <<= 1;
            crc ^= CRC_POLYNOMIAL;
          }
          else {
            crc <<= 1;
          }
        }
      }


// build crc lookup table
//  for (byte i = 0; i < 256; i++) {
//    byte currentByte = i;
//    for (byte k = 0; k < 8; k++) {
//      if ((currentByte & 0x80) != 0) {
//        currentByte <<= 1;
//        currentByte ^= CRC_POLYNOMIAL;
//      } else {
//        currentByte <<= 1;
//      }
//    }
//    crcLookup[i] = currentByte & 0xff;
//  }
      
      msg[5] = crc & 0xff;
      msgBuilt = 1;
    }
  }
}


ISR(INT0_vect) {
  // handle step/dir signals
  if (DIR_PIN) stepperPos++;
  else stepperPos--;
  if ((stepperPos == 32767) || (stepperPos == -32768)) stepperOverflow = 1;
}

ISR(PCINT2_vect) {
  //handle encoder counts
  bool encoderAState = ENCODER_A_PIN;
  bool encoderBState = ENCODER_B_PIN;
  if (encoderAState != encoderALastState) { //then a state change on A is what brought us here
    if (encoderAState == encoderBState) encoderPos++;
    else encoderPos--;
  }else {  //then a state change on B brought us here
    if (encoderAState == encoderBState) encoderPos--;
    else encoderPos++;
  }
  if ((encoderPos == 32767) || (encoderPos == -32768)) encoderOverflow = 1;
  encoderALastState = encoderAState;
}

ISR(TIMER1_COMPA_vect) {
  if (resendOnError) {
    if (dataBufferTracker[dataIndex]) bufferOverflow = 1; //don't overwrite data for which a receipt hasn't been acknowledged
    dataBufferTracker[dataIndex] = 1; //mark the tracker for this index for the buffer overflow tracking
  }
  
  encoderBuffer[dataIndex] = encoderPos; //put encoder and stepper positions in the buffer
  stepperBuffer[dataIndex] = stepperPos;
  encoderPos=0; //reset encoder/stepper positions
  stepperPos=0;
  
  if (dataIndex == BUFFER_SIZE - 1) dataIndex = 0; //increment with artificial overflow
  else dataIndex++;
}

void OverflowHandler() {  //TODO handle overflows for encoder/stepper data
  TIMSK1 = 0;  //mask the timer output connected to built-in LED
  Serial.flush();
  byte msg[6] = {BREAK_MSG};
  Serial.write(msg, 6);
  if (bufferOverflow) {
    Serial.println(F("BUFFER OVERFLOW -- TRY:"));
    Serial.println(F("- LONGER TIME INTERVAL"));
    Serial.println(F("- WRITING A BETTER SERIAL PROTOCOL"));
  }
  if (encoderOverflow) {
    Serial.println(F("ENCODER DATA OVERFLOW -- TRY:"));
    Serial.println(F("- SHORTER TIME INTERVAL"));
    Serial.println(F("- SLOWER EXTRUDER SPEED"));
    Serial.println(F("- LOWER ENCODER RESOLUTION"));
  }
  if (stepperOverflow) {
    Serial.println(F("STEPPER DATA OVERFLOW -- TRY:"));
    Serial.println(F("- SHORTER TIME INTERVAL"));
    Serial.println(F("- SLOWER EXTRUDER SPEED"));
    Serial.println(F("- LOWER MICROSTEPPING"));
  }

  Serial.end();
  bool ledState = 1;
  while (true) {
    digitalWrite(LED_BUILTIN, ledState);
    delay(1000);
    ledState = !ledState;
  }
}
