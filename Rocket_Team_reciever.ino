//com 3 is reciever 
#include <chibi.h>

void setup()
{
  Serial.begin(115200);
  chibiInit();
  chibiSetShortAddr(5);
}

void loop()
{
// Check if any data was received from the radio. If so, then handle it.
  if (chibiDataRcvd() == true)
  {
    byte len, buf[100];
    
    len = chibiGetData(buf);
    if (len == 0) return; // if no len, its a dupe packet. discard.
    
    Serial.print("Transmission: " );
    Serial.println((char *)buf);
  }

  
}


