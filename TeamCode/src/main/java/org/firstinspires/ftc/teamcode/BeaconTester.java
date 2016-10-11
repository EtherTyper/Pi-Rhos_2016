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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Test: Motor Encoder", group = "Concept")
@TeleOp(name = "Test: Color Sensor", group = "Linear Opmode")
public class BeaconTester extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime speedTimer = new ElapsedTime();

  //Initialize Variables
  DcMotor leftMotor = null;
  DcMotor rightMotor = null;
  ColorSensor rightColorSensor = null;
  ColorSensor leftColorSensor = null;
  Servo rightServo = null;
  Servo leftServo = null;

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

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
  }

  /*
     * Code to run when the op mode is first enabled goes here
     * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#start()
     */
  @Override
  public void init_loop() {

    leftMotor = hardwareMap.dcMotor.get("left motor");
    rightMotor = hardwareMap.dcMotor.get("right motor");
    rightColorSensor = hardwareMap.colorSensor.get("color sensor 1");
    leftColorSensor = hardwareMap.colorSensor.get("color sensor 2");

    leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    //Test for color sensor
    telemetry.addData("Blue value: ", rightColorSensor.blue());

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

    telemetry.addData("Right color sensor blue: ", rightColorSensor.blue());
    telemetry.addData("Right color sensor red: ", rightColorSensor.red());
    telemetry.addData("Left color sensor blue: ", leftColorSensor.blue());
    telemetry.addData("Left color sensor red: ", leftColorSensor.red());

    testForRed(rightColorSensor.blue(), rightColorSensor.red(), leftColorSensor.blue(), leftColorSensor.red());

  }

  //Test for red
  public void testForRed(int rightBlueValue, int rightRedValue, int leftBlueValue, int leftRedValue){
    //Stop motor
    if(rightRedValue >= 10 && leftRedValue >= 10){
      rightServo.setPosition(0);
      leftServo.setPosition(0);
    }

    //Press buttond
    if(rightRedValue >= 10 && leftRedValue <= 10){
      rightServo.setPosition(1);
    }
    else if (leftRedValue >= 10 && rightRedValue <= 10){
      leftServo.setPosition(1);
    }

  }

}
