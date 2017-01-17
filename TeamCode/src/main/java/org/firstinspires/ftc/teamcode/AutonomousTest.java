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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 Init Robot - reset gyro, reset encoders, turn on color sensor LEDs
 Move Foward x inches
 Turn x degrees toward Center Vortex
 -Check Gyro to confirm, correct and check again
 Shoot
 Reload
 Shoot
 Turn x degrees towards second beacon
 -Check Gyro to confirm, correct and check again
 Move Foward toward second beacon
 Turn to parallel with wall
 -Check Gyro to confirm, correct and check again
 Move Foward slowly to detect white line
 Detect the beacon colors
 Adjust according to color detected on beacon
 Press beacon, and retract
 Move Backward
 Detect White line
 Detect the beacon colors
 Adjust according to color detected on beacon
 Press beacon, and retract
 Turn to face field
  */
//@Autonomous(name = "Autonomous Test", group = "Concept")
public class AutonomousTest extends LinearOpMode {
  private HardwareConfiguration3rdEdition robot = new HardwareConfiguration3rdEdition();
  int threshold = 2;
  //Initialization
  public void init_hardware() {
    telemetry.addData("Status", "Initialized");

    robot.init(hardwareMap);

    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.gyro.calibrate();
    //robot.lineColorSensor.enableLed(true);
  }

  //Stop Function
  public void delay(long mils){
    try {
      Thread.sleep(mils);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  //Turning
  public void turnRightVariableAndStop(double power, int heading)
  {
      int i = 0;

          while (i<10000/*absHeading() < (heading - threshold) || absHeading() > heading + threshold*/) {
              robot.frontLeftMotor.setPower(power);
              robot.backLeftMotor.setPower(power);
              robot.frontRightMotor.setPower(-power);
              robot.backRightMotor.setPower(-power);
              i++;
          }
          /*robot.frontLeftMotor.setPower(0);
          robot.backLeftMotor.setPower(0);
          robot.frontRightMotor.setPower(0);
          robot.backRightMotor.setPower(0);*/
      
  }

  @Override
  public void runOpMode() throws InterruptedException {
      telemetry.addData("Stage: ", "Called runOpMode");
      telemetry.update();
      //Initialize Robot
      init_hardware();

      waitForStart();

      //Move the Robot
      turnRightVariableAndStop(1,90);
      telemetry.addData("Step 2: ", "turned");
      telemetry.addData("Current Heading", robot.gyro.getHeading());
      telemetry.update();

  }
}