/* 
 * Visible light communication library
 * For INF471C - Connected Objects and the Internet of Things (IoT) at École polytechnique
 * Ilja Tihhanovski <ilja.tihhanovski@gmail.com>
 *
 * Inspired by 1-Wire protocol https://en.wikipedia.org/wiki/1-Wire

SYMBOLS
START
0
1
FINISH

 */

#include <Arduino.h>

#ifndef VLC_DEBUG
  #define VLC_DEBUG 0
#endif

// Receiver could be in two states
#define VLC_RECEIVER_STATE_IDLE 0
#define VLC_RECEIVER_STATE_INCOMING 1

// variables for interrupt handler
volatile bool started = false;
volatile unsigned long startedTime = 0;
volatile unsigned long impulseLength = 0;
volatile unsigned long impulseNo = -1;

// Interrupt handler on LDR pin
// TODO - works only with PIN2
// Fired on CHANGE event on receiver pin
void _vlc_reactToInterrupt() {
  unsigned long t = millis();

  //TODO works only with PIN2
  // Check if pin is HIGH or LOW
  bool ledOn = (PIND & (1<<PD2));      // See https://www.avrfreaks.net/forum/pin-change-interrupt-low
  //bool ledOn = digitalRead(vlc.getInputPin());
  if(ledOn && !started)
  {
    // Save incoming pulse start moment (LOW -> HIGH)
    started = true;
    startedTime = t;
    impulseLength = 0;
  }

  if(started && !ledOn)
  {
    // pulse is over (HIGH -> LOW), calculate pulse length, save pulse number
    impulseLength = t - startedTime;
    started = false;
    impulseNo++;
  }  
}


class VLC {

  // Delays - all delays based on delayScale.
  // Works fine with 2 and more
  // TODO Does not work when delayScale = 1, maybe because of "delayReceiver1 = delayScale / 2;"?
  unsigned long delayScale;

  // Sender delays, during the symbol LED will be ON
  unsigned long delaySenderStart;         // transmission START symbol duration
  unsigned long delaySenderFinish;        // transmission FINISH symbol duration
  unsigned long delaySender1;             // SYMBOL 1 duration
  unsigned long delaySender0;             // SYMBOL 2 duration
  unsigned long delaySenderPause;         // pause between symbols. During the pause LED is OFF
  
  // Receiver delays.
  // We cant expect that the durations of symbols on the receiver will be exactly the same as on the transmitter
  // Nonoptimal software and physical properties of LED and LDR can modify the period when the receiver gets HIGH level on its input pin
  // So we expect the duration of symbol to be longer of period that is slightly shorter than corresponding period on the sender side
  // For example if sender START symbol lasts for 20 msec, then the receiver will consider incoming symbol as START when it lasts between 14 and 26 msec
  // Receiver does not measure intervals between symbols. They just must be long enough
  unsigned long delayReceiverStart;
  unsigned long delayReceiverFinish;
  unsigned long delayReceiver1;
  unsigned long delayReceiver0;

  // Calculates symbols durations.
  void setDelays()
  {
    // Coeficients are quite random.
    // TODO experiment with different coeficients to gain faster transmissions
    delaySenderStart = delayScale * 10;
    delaySenderFinish = delayScale * 15;
    delaySender1 = delayScale;
    delaySender0 = delayScale * 3;
    delaySenderPause = delayScale * 3;

    delayReceiverStart = delayScale * 7;
    delayReceiverFinish = delayScale * 13;
    delayReceiver1 = delayScale / 2;
    delayReceiver0 = delayScale * 2;
    //delayReceiverPause = delayScale * 3;
  }

  // Callbacks for receiver
  void (*onMessageReceived)(const String& s) = nullptr;     // Message is completely received (after FINISH symbol received)
  void (*onMessageStarted)() = nullptr;                     // Called (if set up) after START symbol received
  void (*onByteReceived)(uint8_t b) = nullptr;              // Called after one byte of information received (i.e. every 8 bits)

  // To distinguish between pulses
  unsigned long processedimpulseNo = 0;
  uint8_t receiverState = VLC_RECEIVER_STATE_IDLE;          // By default receiver is in IDLE state
  
  // digital pins
  uint8_t inputPin;
  uint8_t outputPin;

  String buffer = "";                                       // Buffer where received data is stored between START and FINISH of transmission
  uint8_t currentByte;                                      // Current byte being received is stored here
  uint8_t currentBit;                                       // Current bit number being received

  // ===== Private receiver methods =============

  // Initialize byte buffer
  void startByte()
  {
    currentByte = 0;
    currentBit = 0;
  }

  // Byte is received:
  void addByte()
  {
    buffer = buffer + (char)currentByte;                    // Add the byte to buffer
    if(onByteReceived != nullptr)                           // Call the listener if listener exist
      onByteReceived(currentByte);
    startByte();                                            // Prepare to receive next byte
  }

  // Bit of data is received (i.e. symbol for 0 or 1 is received)
  void addBit(bool b)
  {
    if(receiverState == VLC_RECEIVER_STATE_INCOMING)        // Receiver must be in certain state to consider incoming data as a part of a message
    {
      #if VLC_DEBUG > 1
        Serial.print("\t");
        Serial.print(b);
      #endif

      if(b)                                                 // If 1 was received
        currentByte |= 1 << (7 - currentBit);               // Save it. We transmit bits starting from the highest bit of the byte (i.e. 2^7 will be sent first, 2^0 will be the last)
      currentBit++;                                         // Move the "bit pointer" one position further

      #if VLC_DEBUG > 1
        Serial.print("\t --> ");
        Serial.print(currentByte, BIN);
      #endif

      if(currentBit > 7)                                    // 8 bits will make 1 byte
      {
        #if VLC_DEBUG > 1
          Serial.print("\t --> ");  
        #endif
        #if VLC_DEBUG > 0
          Serial.print((char)currentByte);
        #endif
        addByte();                                          // Save this byte
      }
    }
    #if VLC_DEBUG > 1
      Serial.println("");
    #endif
  }

  // ===== Private sender methods ============


  // Send out an impulse of given(in milliseconds) duration
  inline void impulse(int iDelay) {
    digitalWrite(outputPin, 1);     // turn ON the transmitter LED
    delay(iDelay);                  // wait
    digitalWrite(outputPin, 0);     // turn OFF the transmitter LED
    delay(delaySenderPause);        // pause between impulsees
  }

  // Methods for sending symbols
  void sendStart() {
    impulse(delaySenderStart);
  }

  inline void sendFinish() {
    impulse (delaySenderFinish);
  }

  void sendBit(bool b) {
    impulse (b ? delaySender1 : delaySender0);
    #if VLC_DEBUG > 1
      Serial.print(b ? 1 : 0);
    #endif
  }

public:

  // Event callbacks registration methods
  
  void setOnMessageReceived(void (*callbackFunc)(const String& s)) {
    onMessageReceived = callbackFunc;
  }

  void setOnMessageStarted(void (*callbackFunc)()) {
    onMessageStarted = callbackFunc;
  }

  void setOnByteReceived(void (*callbackFunc)(uint8_t b)) {
    onByteReceived = callbackFunc;
  }

  // Initialize
  void begin(unsigned long delayScale, uint8_t outputPin, uint8_t inputPin)
  {
    this->inputPin = inputPin;
    this->outputPin = outputPin;

    if(inputPin > 0)
      pinMode(inputPin, INPUT);
    if(outputPin > 0)
      pinMode(outputPin, OUTPUT);
    
    this->delayScale = delayScale;
    setDelays();

    
    if(inputPin > 0)    //TODO Currently library works only with input pin 2
    {
      noInterrupts();
      attachInterrupt(digitalPinToInterrupt(inputPin), _vlc_reactToInterrupt, CHANGE);
      interrupts();
    }
  }

  // Returns buffer contents and empties the buffer
  // TODO test in some example
  String read()
  {
    String ret = buffer;
    buffer = "";
    return ret;
  }

  // Should be called from every loop iteration on receiver to handle incoming data.
  // If more than one bit was received between two consequent loop calls, data will be lost!
  // TODO Maybe interrupt handler itself could save decode incoming impulses and save data bits and loop could do the heavy stuff like debug and callbacks
  void loop() {
    if(processedimpulseNo != impulseNo)                 // To avoid handling the same incoming impulse twice, every impulse has unique number
    {
      #if VLC_DEBUG > 1
        Serial.print(impulseNo);
        Serial.print(":\t");
        Serial.print(impulseLength);
        Serial.print("\t");
      #endif
      processedimpulseNo = impulseNo;

      // Decode impulse length:
      // Compare last received impulse with possible durtions starting from longest
      // First suitable duration will give us the symbol
      
      if(impulseLength > delayReceiverFinish)
      {
        // FINISH symbol received
        receiverState = VLC_RECEIVER_STATE_IDLE;        // Receiver state is IDLE now
        #if VLC_DEBUG > 1
          Serial.println("FINISH");
          Serial.print("\nMessage: '");
          Serial.print(buffer);
          Serial.println("'");
        #endif

        if(onMessageReceived != nullptr)                // Notify the client that message is reveived  if callback is set up
          onMessageReceived(buffer);

        // Prepare to receive a next message
        buffer = "";
        startByte();
        impulseNo = 0;
        processedimpulseNo = 0;
      }
      else if(impulseLength > delayReceiverStart)
      {
        // START symbol received
        receiverState = VLC_RECEIVER_STATE_INCOMING;   // Receiver state is INCOMING - ready to receive data bits
        buffer = "";                                   // Prepare the buffers
        startByte();

        if(onMessageStarted != nullptr)                // Notify the client if callback is set up
          onMessageStarted();
      }
      else if (impulseLength > delayReceiver0)         // Data bit 0 received
        addBit(0);
      else if (impulseLength > delayReceiver1)         // Data bit 1 received
        addBit(1);
    }
  }

  // Send given message
  void sendMessage(String s)
  {
    sendStart();                                        // Send START symbol
    for(uint8_t c : s)                                  // Repeat for every byte in message
    {
      #if VLC_DEBUG > 0
        Serial.print((char)c);
        Serial.print("\t");
        Serial.print(c, BIN);
        Serial.print("\t");
      #endif
      for(uint8_t i = 128; i > 0; i = i >> 1)           // Repeat for every bit in byte starting with bit #7 (i.e. 2^7 = 128)
        sendBit(c & i);                                 // Sent the bit symbol
      #if VLC_DEBUG > 1
        Serial.println("");
      #endif
    }
    sendFinish();                                       // Send FINISH symbol
  }
};

VLC vlc;