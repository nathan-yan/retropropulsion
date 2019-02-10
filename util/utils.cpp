class Pyro{
  private:
    int timer = 0;
    int limit = 0;
    int pin;

  public:
    Pyro(int pin);

    void fire(int milliseconds);
    void check();
};

Pyro::Pyro(int pin){
  this->pin = pin;
  pinMode(pin, OUTPUT);
}

void Pyro::fire(int milliseconds){
  timer = millis();
  limit = timer + milliseconds;

  digitalWrite(pin, HIGH);
}

void Pyro::check(){
  if (millis() > limit){
    digitalWrite(pin, LOW);
  }
}

float clip(float x, float minimum, float maximum){
    if (x > maximum) {
        return maximum;
    }else if (x < minimum) {
        return minimum;
    }

    return x;
}

double RToDegree(double r)
{
  return r * 180 / PI;
}

double DegreeToR(double d)
{
  return d * PI / 180;
}
double servoAngle(double gimbalAngle)
{
  double x = 4; // in cm
  double y = 0.7; // in cm

  return RToDegree(asin(x / y * sin(DegreeToR(gimbalAngle))));
}

void dataToBytes(float* data, uint8_t* buf, int dataLen, int bufLen, bool isUnsigned = false){
  // Converts a list of floats into a list of characters

  if (bufLen <= dataLen){    // It is impossible for bufLen to be less than dataLen because each data point is 13 bits
    return;
  }

  int buffer = 0;
  int bits = 0;
  int c = 0;

  int total_bits = 0;

  for (int i = 0; i < dataLen; i++){
    int data_;

    if (isUnsigned){
      if (data[i] < 0){
        return ;
      }

      data_ = (int) clip((data[i] * 10), 0, 5000);
    } else {
      data_ = (int) clip((data[i] * 10), -2500, 2500) + 2500;
    }

    buffer += data_ << bits;  // Shift data_ up `bits` number of bits because we don't want to overwrite the left over bits from the previous number
    bits += 13;

    while (bits >= 8){
      char write = buffer & 0xff;     // Get first byte (in little endian)
      buffer >>= 8;                   // Shift down a byte
      bits -= 8;

      if (c >= bufLen){   // Don't overwrite the buffer
            return;
      }
      
      buf[c] = write;
      total_bits += 8;
      c++;
    }
  }

  buf[c] = buffer;
  total_bits += 8;
  //SerialUSB.print("WRITTEN ");
  //SerialUSB.println(total_bits);
}

void editConfigs(SPIFlash flash, int configSize, uint8_t* values, uint8_t* addresses, int len){
  uint8_t configValues[configSize];
  flash.readBytes(0xfa0, configValues, configSize);

  flash.blockErase4K(0xfa0);    // Erase from the second 4k block
  delay(500); 

  for (int i = 0; i < len; i++){
    configValues[addresses[i]] = values[i];  
  }

  flash.writeBytes(0xfa0, configValues, configSize);
}