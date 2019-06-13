#include <VirtualWire.h>

//Grove - 315(433) RF link kit Demo v1.0
//by :http://www.seeedstudio.com/
//connect the sent module to D2 to use  
#include <VirtualWire.h>

int RF_TX_PIN = 11;

void setup(){
  vw_set_tx_pin(RF_TX_PIN); // Setup transmit pin
  vw_setup(3000); // Transmission speed in bits per second.
}

void loop(){
  const char *msg = "hello";
  vw_send((uint8_t *)msg, strlen(msg));  // Send 'hello' every 400ms.
  delay(400);

}
