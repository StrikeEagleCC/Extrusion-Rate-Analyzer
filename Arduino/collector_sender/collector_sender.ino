/*
 * pin assignments:
 * encoder A --> D2
 * encoder B --> D3
 * stepper step --> D4
 * stepper dir --> D5
 */

#define ENCODER_A_PIN            PIND2
#define ENCODER_B_PIN            PIND3
#define STEP_PIN                 PIND4
#define DIR_PIN                  PIND5

#define DATA_BUFFER_SIZE         16  // valid values are 2,4,8,16,32, and 64
#define TIMER_INTERVAL           250 // how often (in milliseconds) to store stepper and encoder counts. valid range is 1-250

//header byte prefixes (just the two MSBs)
#define ARD_NORMAL_BYTE          0x00
#define ARD_ERROR_BYTE           0x40
#define ARD_SYNC_BYTE            0x80
#define ARD_MESSAGE_BYTE         0xC0
#define GOOD_ACK_BYTE            0x00
#define BAD_ACK_BYTE             0x40
#define BUFFER_EMPTY_BYTE        0x80 

//variables for interrupts
volatile int encoderBuffer[DATA_BUFFER_SIZE];
volatile int stepperBuffer[DATA_BUFFER_SIZE];
volatile bool dataBufferTracker[DATA_BUFFER_SIZE];
volatile int encoderPos = 0;
volatile int stepperPos = 0;
volatile bool bufferOverflow = 0;
volatile bool encoderOverflow = 0;
volatile bool stepperOverflow = 0;
volatile byte dataIndex = 0;


void setup() {

  Serial.begin(250000);
  Serial.write(DATA_BUFFER_SIZE);  //tell python the data buffer size so it can set its index size. TODO: get a response verifying good receipt
  bool ledState = 1;
  while (Serial.available() < 1){
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, ledState);
    delay(500);
    ledState = !ledState;
  }
  digitalWrite(LED_BUILTIN, LOW);

  while (Serial.available()) Serial.read(); //clear any remaining data in the recieve buffer (there shouldn't be any)

  //initialize buffers
  for (int i = 0; i < DATA_BUFFER_SIZE; i++) {
    encoderBuffer[i] = 0;
    stepperBuffer[i] = 0;
    dataBufferTracker[i] = 0;
  }

  //set up timer 1
  cli();
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 250 * TIMER_INTERVAL; // set compare match register
  TCCR1B |= (1 << WGM12); // turn on CTC mode
  TCCR1B |= (1 << CS11) | (1 << CS10); //set bits for prescaler: clk/64

  // set up pin interrupts
  DDRD   |= B00111100;  //enable pins D2,3,4 and 5 as inputs
  PORTD  |= B00001100;  //enable pull-ups on D2,3
  EICRA   = B00000101;  //set interrupt type to logic change for pins D2,3
  PCICR  |= B00000100;  //enable pin change interrupts on PORTD
  sei();  
  
  //mask inturrupts
  TCNT1 = 0;  //reset timer
  TIMSK1 |= (1 << OCIE1A);  //mask timer interrupt
  EIMSK  |= (1 << INT0);  //mask external interrupt on D2
  EIMSK  |= (1 << INT1);  //mask external interrupt on D3
  PCMSK2 |= (1 << PCINT20); //mask pin change interrupt on D4
}

void loop() {
  static byte txIndex = 0;
  static byte rxIndex = 0;
  static bool txReady = 1;
  
  //check for buffer overflow
  if (bufferOverflow || encoderOverflow || stepperOverflow) OverflowHandler();
  // TODO make this with a software inturrupt on the overflow bits?

  // handle acks
  byte rxBufferSize = Serial.available();
  static bool lastAckBad = 0;
  if (rxBufferSize){
    for (int i = 0; i < rxBufferSize; i++) {
      int ack = Serial.read();
      if (ack ^ 0xc0 == BUFFER_EMPTY_BYTE) {
        if (lastAckBad) sendSyncPackets(rxIndex);
        txReady = 1;
        continue;        
      }else if (ack == GOOD_ACK_BYTE | (rxIndex + 1)) {
        cli();
        dataBufferTracker[rxIndex] = 0;
        sei();
        rxIndex++;
        continue;
      }else {
      txReady = 0;
      lastAckBad = 1;
      sendSyncPackets(rxIndex);
      txIndex = rxIndex + 1;
      }
    }
  }
  
  //build and send message
  if (txReady) {
    cli();
    byte dataIndex2 = dataIndex;  //prevent interrupts from changing dataIndex during checking (maybe not necessary?)
    sei();
    if (txIndex != dataIndex2) {  //only send if there's new data
      //build message
      byte msg[5] = {
        ARD_NORMAL_BYTE | txIndex,
        (unsigned int)encoderBuffer[txIndex] >> 8,  //cast to unsigned int to prevent sign extension on shifting
        encoderBuffer[txIndex],
        (unsigned int)stepperBuffer[txIndex] >> 8,
        stepperBuffer[txIndex],
        };
      Serial.write(msg, 5);
      if (txIndex == DATA_BUFFER_SIZE - 1) txIndex = 0; //increment transmit index with artificial overflow
      else txIndex++;
    }
  }
}

ISR(INT0_vect) {
  //trigger on encoder channel A state change
  if (ENCODER_A_PIN == ENCODER_B_PIN) encoderPos++; //check state of other channel
  else encoderPos--;
  if ((encoderPos == 32767) || (encoderPos == -32768)) encoderOverflow = 1;
}

ISR(INT1_vect) {
  //trigger on encoderchannel B state change
  if (ENCODER_A_PIN == ENCODER_B_PIN) encoderPos--; //check state of other channel
  else encoderPos++;
  if ((encoderPos == 32767) || (encoderPos == -32768)) encoderOverflow = 1;
}

ISR(PCINT2_vect) {
  if (STEP_PIN) {//increment on step signal rising edge only
    if (DIR_PIN) stepperPos++;
    else stepperPos--;
    if ((stepperPos == 32767) || (stepperPos == -32768)) stepperOverflow = 1;
  }
}

ISR(TIMER1_COMPA_vect) {
  if (dataBufferTracker[dataIndex]) bufferOverflow = 1; //don't overwrite data for which a receipt hasn't been acknowledged
  encoderBuffer[dataIndex] = encoderPos; //put encoder and stepper positions in the buffer
  stepperBuffer[dataIndex] = stepperPos;
  dataBufferTracker[dataIndex] = 1; //mark the tracker for this index for the buffer overflow tracking
  encoderPos = 0; //reset encoder/stepper positions
  stepperPos = 0;
  if (dataIndex == DATA_BUFFER_SIZE - 1) dataIndex = 0; //increment with artificial overflow
  else dataIndex++;
}

void sendSyncPackets(byte index) {
  byte msg[5] = {ARD_SYNC_BYTE ^ index, 0xff, 0xff, 0xff, 0xff};
  Serial.write(msg, 5);
  Serial.write(msg, 5);
}

void OverflowHandler() {  //TODO handle overflows for encoder/stepper data
  TIMSK1 = 0;  //mask the timer output connected to built-in LED
  Serial.flush();
  byte msg[5] = {ARD_MESSAGE_BYTE, 0x00, 0x00, 0x00, 0x00};
  Serial.write(msg, 5);
  if (bufferOverflow) {
    Serial.println("BUFFER OVERFLOW -- TRY:");
    Serial.println("- LONGER TIME INTERVAL");
    Serial.println("- WRITING A BETTER SERIAL PROTOCOL");
  }
  if (encoderOverflow) {
    Serial.println("ENCODER DATA OVERFLOW -- TRY:");
    Serial.println("- SHORTER TIME INTERVAL");
    Serial.println("- SLOWER EXTRUDER SPEED");
    Serial.println("- LOWER ENCODER RESOLUTION");
  }
  if (stepperOverflow) {
    Serial.println("STEPPER DATA OVERFLOW -- TRY:");
    Serial.println("- SHORTER TIME INTERVAL");
    Serial.println("- SLOWER EXTRUDER SPEED");
    Serial.println("- LOWER MICROSTEPPING");
  }
  Serial.end();
  bool ledState = 1;
  while (true) {
    digitalWrite(LED_BUILTIN, ledState);
    delay(1000);
    ledState = !ledState;
  }
}
