#include <ros.h>
#include <std_msgs/String.h>

#define in1 4
#define in2 5
#define in3 6
#define in4 7 
#define enA 9 
#define enB 11 

//the amount of time for movement on a single key press
const int DELAY_TIME = 400;

//speed for the wheels
const int SPEED = 190;

//Node handler for subscribing to the topic '/keypress'
ros::NodeHandle nh;

void callback_message(const std_msgs::String& pressed_key){
  nh.loginfo(pressed_key.data);
  String message = String(pressed_key.data);
  
  if(message.equals("Key.up")) 
    move_forward();
  if(message.equals("Key.down"))
    move_backward();
  if(message.equals("Key.left")) 
    move_left();
  if(message.equals("Key.right")) 
    move_right();
}

//Subscribing to the topic "/keypress" to receive the input from keyboard
ros::Subscriber<std_msgs::String> sub("/keypress", &callback_message);

void setup() {
  // Setting up the pins to output
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  //initial direction of the motors
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  nh.spinOnce();
  delay(10);
}


void move_backward() {
  // Set Motor A backward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Set Motor B backward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  //Move backward for the given DELAY_TIME, then stop
  analogWrite(enA, SPEED);
  analogWrite(enB, SPEED);
  delay(DELAY_TIME);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void move_forward() {
  // Set Motor A forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // Set Motor B forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  //Move forward for the given DELAY_TIME, then stop
  analogWrite(enA, SPEED);
  analogWrite(enB, SPEED);
  delay(DELAY_TIME);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}

void move_left() {
  // Set Motor A forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // Set Motor B backward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);

  //Move left for the given DELAY_TIME, then stop
  analogWrite(enA, SPEED);
  analogWrite(enB, SPEED);
  delay(DELAY_TIME);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}


void move_right() {
  // Set Motor A backward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Set Motor B forward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);

  //Move right for the given DELAY_TIME, then stop
  analogWrite(enA, SPEED);
  analogWrite(enB, SPEED);
  delay(DELAY_TIME);
  analogWrite(enA, 0);
  analogWrite(enB, 0);
}
