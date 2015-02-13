
 int ledPin=13;
 int i = 0;
   
void setup() {
   Serial.begin(9600);
 }

void loop() {
   while(!Serial.available()) {
     delay(200);
   }
   int buf[4];
   while(Serial.available()) {
     buf[i] = Serial.read();
     i++;
   }
   for(i = 0; i<4; i++) {
     Serial.print(buf[i]);
     Serial.print(" ");
   }  
   Serial.print("\n");
   i = 0;
   delay(200);
 }


