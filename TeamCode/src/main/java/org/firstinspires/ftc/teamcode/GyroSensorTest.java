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

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Test: Motor Encoder", group = "Concept")
@TeleOp(name = "Test: Gyro Sensor Test", group = "Linear Opmode")
public class GyroSensorTest extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime speedTimer = new ElapsedTime();

  //Initialize Variables
  DcMotor leftMotor = null;
  DcMotor rightMotor = null;
  GyroSensor robotGyroSensor = null;
  //All units here is inches
  private final int ticksPerRotation = 1120;
  private int motorTarget = 3 * ticksPerRotation;
  private int realTimeTicks = 0;
  public int threshold = 5;

  private double time = 0;
  private double wheelDiameter = 2.75;

  private double olderTick = 0;
  private double currentTick = 0;
  private double lastSecondsTick = 0;
  private double motorSpeed = 0;

  private boolean LEDStatus = false;
  public boolean goalReached = false;


  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");
    robotGyroSensor = hardwareMap.gyroSensor.get("gyro sensor");
    leftMotor = hardwareMap.dcMotor.get("left motor");
    rightMotor = hardwareMap.dcMotor.get("right motor");
    robotGyroSensor.calibrate();
    leftMotor.setDirection(DcMotor.Direction.REVERSE);
  }
  @Override
  public void init_loop() {

  }

  @Override
  public void start() {

    robotGyroSensor.calibrate();
    telemetry.addData("Is calibrating?", robotGyroSensor.isCalibrating());
  }

  public void turnRightNintyAndStop() {
    if (!robotGyroSensor.isCalibrating()){
      if (robotGyroSensor.getHeading() > (90 - threshold) && robotGyroSensor.getHeading() < (90 + threshold)) {
        goalReached = true;
      }
    if (goalReached == true) {
      leftMotor.setPower(0);
      rightMotor.setPower(0);
    } else if (goalReached == false) {
      leftMotor.setPower(1);
      rightMotor.setPower(-1);
    }
  }
  }

  public void turnLeftNintyAndStop() {
    if (!robotGyroSensor.isCalibrating()){
      if (robotGyroSensor.getHeading() > (270 - threshold) && robotGyroSensor.getHeading() < (270 + threshold)) {
        goalReached = true;
      }
      if (goalReached == true) {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
      } else if (goalReached == false) {
        leftMotor.setPower(-1);
        rightMotor.setPower(1);
      }
    }
  }

  public void turnRightVariableAndStop(double power, int heading)
  {
    if (!robotGyroSensor.isCalibrating()){
      if (robotGyroSensor.getHeading() > (heading - threshold) && robotGyroSensor.getHeading() < (heading + threshold)) {
        goalReached = true;
      }
      if (goalReached == true) {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
      } else if (goalReached == false) {
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
      }
    }
  }

  public void turnLeftVariableAndStop(double power, int heading)
  {
    if (!robotGyroSensor.isCalibrating()){
      if (robotGyroSensor.getHeading() > ((360-heading) - threshold) && robotGyroSensor.getHeading() < ((360-heading) + threshold)) {
        goalReached = true;
      }
      if (goalReached == true) {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
      } else if (goalReached == false) {
        leftMotor.setPower(-power);
        rightMotor.setPower(power);
      }
    }
  }

  public void recalibrateGyro()
  {
    robotGyroSensor.calibrate();
  }

  //The code for the robot while running driver control
  //This method acts as a while loop
  @Override
  public void loop() {
    telemetry.addData("Gyro X Value", robotGyroSensor.rawX());
    telemetry.addData("Gyro Y Value", robotGyroSensor.rawY());
    telemetry.addData("Gyro Z Value", robotGyroSensor.rawZ());
    telemetry.addData("Heading!", robotGyroSensor.getHeading());
    telemetry.addData("Sensor Status: ", robotGyroSensor.status());
    turnRightVariableAndStop(0.25, 90);
    //recalibrateGyro();
    //turnLeftNintyAndStop();
    //recalibrateGyro();
    //turnRightVariableAndStop(0.5, 45);
  }
}
 