#include <SPI.h>
#include <Ethernet.h>
#include <EEPROM.h>
#include <RobotOpen.h>

boolean winding = false;
boolean wound = false;
boolean ready = true;

/* I/O Setup */
ROJoystick usb1(1);         // Joystick #1
ROPWM pwm0(0);
ROPWM pwm1(1);
ROPWM pwm2(2);
ROPWM pwm3(3);

ROSolenoid sol0(0);
ROSolenoid sol1(1);

ROTimer kickerReset;

void setup()
{
  /* Initiate comms */
  RobotOpen.begin(&enabled, &disabled, &timedtasks);
  
  pinMode(SIDECAR_DIGITAL4, OUTPUT);
  digitalWrite(4,LOW);
  pinMode(SIDECAR_DIGITAL5, OUTPUT);
  digitalWrite(5,LOW);
  pinMode(SIDECAR_DIGITAL3, OUTPUT);
  digitalWrite(3,LOW);
  
  pinMode(SIDECAR_DIGITAL1, INPUT);
  pinMode(SIDECAR_DIGITAL2, INPUT);
}


/* This is your primary robot loop - all of your code
 * should live here that allows the robot to operate
 */
void enabled() {
  // get desired translation and rotation, scaled to [-127..128] (0 neutral)
  int x = usb1.leftX() - 127;
  int y = (255 - usb1.leftY()) - 127;
  int rotate = (255 - usb1.rightX()) - 127;

  // calculate wheel throttles
  int lf = x + y - rotate;
  int rf = x - y - rotate;
  int lr = -x + y - rotate;
  int rr = -x - y - rotate;

  // normalize wheel throttles
  int maximum = max(max(abs(lf), abs(rf)), max(abs(lr), abs(rr)));
  if (maximum > 127) {
    lf = (lf / maximum) * 127;
    rf = (rf / maximum) * 127;
    lr = (lr / maximum) * 127;
    rr = (rr / maximum) * 127;
  }

  // Set PWMs, shifted back to [0..255]
  pwm0.write((lf*1) + 127);
  pwm1.write((rf*1) + 127);
  pwm2.write((lr*1) + 127);
  pwm3.write((rr*1) + 127);
  
  if(!digitalRead(SIDECAR_DIGITAL1)){
    digitalWrite(4,HIGH);
  } else {
    digitalWrite(4,LOW);
  }
  
  if(usb1.btnX()){
    digitalWrite(5,HIGH);
  }else{
    digitalWrite(5,LOW);
  }
  
  if(usb1.btnA()){
    if(wound){
      sol1.on();
      sol0.off();
      wound = false;
      kickerReset.queue(250);
    } else if(ready) winding = true;
  }
  
  if(digitalRead(SIDECAR_DIGITAL2)) { winding=false; wound=true; }
  
  if(winding){
    sol0.on();
    sol1.off();
    digitalWrite(3, HIGH);
  } else digitalWrite(3, LOW);
  
  if(kickerReset.ready()) ready = true;
}


/* This is called while the robot is disabled
 * PWMs and Solenoids are automatically disabled
 */
void disabled() {
  digitalWrite(4,LOW);
  digitalWrite(5,LOW);
  digitalWrite(3,LOW);
}


/* This loop ALWAYS runs - only place code here that can run during a disabled state
 * This is also a good spot to put driver station publish code
 */
void timedtasks() {
  RODashboard.publish("Wound", digitalRead(SIDECAR_DIGITAL2));
}


// !!! DO NOT MODIFY !!!
void loop() {
  RobotOpen.syncDS();
}
