//Grove - 315(433) RF link kit Demo v1.0
//by :http://www.seeedstudio.com/
//connect the receive module to D2 to use ..
#include <VirtualWire.h>

int RF_RX_PIN = 3;

void setup()
{
  Serial.begin(9600);
  Serial.println("setup");
  vw_set_rx_pin(RF_RX_PIN);  // Setup receive pin.
  vw_setup(3000); // Transmission speed in bits per second.
  vw_rx_start(); // Start the PLL receiver.
}

void loop()
{
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  if(vw_get_message(buf, &buflen)) // non-blocking I/O
  {
    int i;
    // Message with a good checksum received, dump HEX
    Serial.print("Received: ");
    String received = ((char*)buf);
    received.remove((sizeof(received)/received[0])-1);
    /*for(i = 0; i < buflen; ++i)
    {
      Serial.print(buf[i], HEX);
      Serial.print(" ");
      Serial.print(buf[i]);
    }*/
    Serial.println(received);
  }
}
