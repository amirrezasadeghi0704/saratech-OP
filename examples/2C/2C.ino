#include <Saratech.h>

Saratech robot;

void print() {
  robot.display.clearDisplay();
  robot.display.setCursor(0, 0);
  robot.display.print("x :");
  robot.display.println(robot.ball_x);
  robot.display.print("y :");
  robot.display.println(robot.ball_y);
  robot.display.print("gy :");
  robot.display.println(robot.gy);  
  robot.display.display();
}

void setup() {
  robot.setup();  
}

void loop() {
  robot.read_mv();
  robot.read_gyro();
  print();
  robot.move(-robot.ball_x * 5.0f, -robot.ball_y * 5.0f);
}
