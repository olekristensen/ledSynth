
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int numDigits(int number)
{
  int digits = 0;
  if (number < 0) {
    digits = 1;
  }
  do {
    number /= 10;
    digits++;
  } while (number != 0);
  return digits;
}

