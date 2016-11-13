#include "arduinolin.h"

ArduinoLin::ArduinoLin(HardwareSerial &ser, uint8_t txPin): serial(ser), txPin(txPin)
{
}

void ArduinoLin::serialFlush()
{
    serial.flush();
}

// Generate a BREAK signal (a low signal for longer than a byte) across the serial line
void ArduinoLin::serialBreak(void)
{
  if (serialOn) serial.end();

  pinMode(txPin, OUTPUT);
  digitalWrite(txPin, LOW);  // Send BREAK
  unsigned long int brkend = (1000000UL/((unsigned long int)serialSpd));
  unsigned long int brkbegin = brkend*LIN_BREAK_DURATION;
  if (brkbegin > 16383) delay(brkbegin/1000);  // delayMicroseconds unreliable above 16383 see arduino man pages
  else delayMicroseconds(brkbegin);

  digitalWrite(txPin, HIGH);  // BREAK delimiter

  if (brkend > 16383) delay(brkend/1000);  // delayMicroseconds unreliable above 16383 see arduino man pages
  else delayMicroseconds(brkend);

  serial.begin(serialSpd);
  serialOn = 1;
}

void ArduinoLin::serialWrite(uint8_t b)
{
    serial.write(b);
}

void ArduinoLin::serialWrite(const uint8_t *data, uint8_t length)
{
    serial.write(data, length);
}

size_t ArduinoLin::serialAvailable()
{
    return serial.available();
}

uint8_t ArduinoLin::serialRead()
{
    return serial.read();
}

void ArduinoLin::enableTx()
{
    pinMode(txPin, OUTPUT);
}

void ArduinoLin::disableTx()
{
    pinMode(txPin, INPUT);
    digitalWrite(txPin, LOW);  // don't pull up
}

uint32_t ArduinoLin::millis()
{
    return ::millis();
}

void ArduinoLin::begin(int speed)
{
    AbstractLin::begin(speed);
    serial.begin(serialSpd);
    serialOn  = 1;
}
