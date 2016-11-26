/* Copyright (c) 2014, 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Test: Motor Encoder", group = "Concept")
@TeleOp(name = "Test: Line Sensor Alpha", group = "Linear Opmode")
public class LineSensorAlpha extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime speedTimer = new ElapsedTime();

  //Initialize Variables
  DcMotor leftMotor = null;
  DcMotor rightMotor = null;
  ColorSensor colorSensor = null;
  Servo servo = null;

  //All units here is inches
  private final int ticksPerRotation = 1120;
  private int motorTarget = 3 * ticksPerRotation;
  private int realTimeTicks = 0;

  private double time = 0;
  private double wheelDiameter = 2.75;

  private double olderTick = 0;
  private double currentTick = 0;
  private double lastSecondsTick = 0;
  private double motorSpeed = 0;

  private boolean reached = false;

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    leftMotor = hardwareMap.dcMotor.get("left motor");
    leftMotor.setDirection(DcMotor.Direction.REVERSE);
    rightMotor = hardwareMap.dcMotor.get("right motor");
    colorSensor = hardwareMap.colorSensor.get("color sensor");
    colorSensor.setI2cAddress(I2cAddr.create8bit(0x10));
    colorSensor.enableLed(false);
    servo = hardwareMap.servo.get("servo");

  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {
    colorSensor.enableLed(true);

  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    //Test for color sensor
    colorSensor.enableLed(true);

  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  //The code for the robot while running driver control
  //This method acts as a while loop
  @Override
  public void loop() {
    //Driver Controller
    if(!reached){
      testForLine();
    }
  }

  //Test for line
  public void testForLine() {
    if(colorSensor.alpha() >= 5) {
      stopRobot();
      reached = true;
      try {
        Thread.sleep(100);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    } else {
      moveForwardNonStop();
    }
  }

  //Test for blue
  public boolean testForBlue() {
    //Stop
    try {
      Thread.sleep(2000);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }

    //Stop servo
    if ((colorSensor.blue() >= 3) && (colorSensor.red() < 2)){
      servo.setPosition(1);
      try {
        Thread.sleep(2000);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
      servo.setPosition(0.5);
      return true;
    }
    return false;
  }

  //Move robot
  public void moveRobotForward(int seconds){
    leftMotor.setPower(0.2);
    rightMotor.setPower(0.2);
    try {
      Thread.sleep(seconds * 100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    leftMotor.setPower(0);
    rightMotor.setPower(0);
  }

  //Move robot none stop
  public void moveForwardNonStop(){
    leftMotor.setPower(0.15);
    rightMotor.setPower(0.15);
  }

  //Stop robot
  public void stopRobot(){
    leftMotor.setPower(0);
    rightMotor.setPower(0);
  }

}
