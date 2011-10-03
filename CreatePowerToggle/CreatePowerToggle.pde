#define powerPin 13
int inByte = 0;

void setup ()
{
 pinMode(powerPin, OUTPUT); // toggle DD to toggle power on the Create
 Serial.begin(9600);
}

void loop()
{
  if (Serial.available() > 0) // if there's feedback from the commander, take it 
  { 
    inByte = Serial.read(); // read the next available character
    switch (inByte)
    {
       case 'y':
          digitalWrite(powerPin, LOW);
          delay(500);
          digitalWrite(powerPin, HIGH);
          delay(500);
    }
  }  
}
