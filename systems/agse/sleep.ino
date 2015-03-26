
int pin = 13;


void wakeUpNow()
{
}


void setup()
{
  pinMode(pin, OUTPUT);
  attachInterrupt(2, sleepNow, FALLING); //triggers when pb is pressed
}

void sleepNow()
{
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    sleep_enable();     
    detachInterrupt(2);
    attachInterrupt(2, wakeUpNow, FALLING); 
    sleep_mode();  // here the device is actually put to sleep!!
    
    sleep_disable(); //first thing after waking up
    detachInterrupt(2);
    attachInterrupt(2, sleepNow, FALLING);
    
}
void loop()
{
  digitalWrite(pin, HIGH);
}


