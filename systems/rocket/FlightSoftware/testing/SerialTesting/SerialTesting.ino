
uint8_t data;
uint8_t x=20;

void setup()
{
    Serial.begin(9600);
    x=20;
}

void loop()
{
    if (Serial.available() > 0) {
        data = Serial.read();
    }
    Serial.println(x);
    x++;
    delay(10000);
}
