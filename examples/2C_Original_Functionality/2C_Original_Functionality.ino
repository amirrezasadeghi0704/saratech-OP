#include <Saratech.h>

// One object replaces the separate display, driver, I2C, and gyro declarations.
Saratech robot;

void print() {
  robot.display.clearDisplay();
  robot.display.setCursor(0, 0);
  robot.display.print("x :");
  robot.display.println(robot.ball_x);
  robot.display.print("y :");
  robot.display.println(robot.ball_y);
  robot.display.print("gy :");
  robot.display.println(robot.gy);  // Raw TDAxis12 value, as in the original sketch.
  robot.display.display();
}

void setup() {
  robot.setup();  // Initializes Serial1, I2C, motors, and OLED.
}

void loop() {
  robot.read_mv();
  robot.read_gyro();
  print();

  // Two-axis movement. The original mecanum wheel mix is done inside Saratech.
  robot.move(-robot.ball_x * 5.0f, -robot.ball_y * 5.0f);

  // Scaled gyro value is also available as robot.gyu (robot.gy * 300).
  // The raw value remains available as robot.gy.
}
