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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.FinalHardwareConfiguration;

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Test: Motor Encoder", group = "Concept")
//@TeleOp(name = "Test: Motor Encoder", group = "Linear Opmode")
public class PiRhoAutoMaster extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime speedTimer = new ElapsedTime();
  private HardwareConfigurationMax robot = new HardwareConfigurationMax();

  //All units here is inches
  private final int ticksPerRotation = 1120;

  private int frontRightTarget = 0;
  private int frontLeftTarget = 0;
  private int backRightTarget = 0;
  private int backLeftTarget = 0;
  private int shooterTarget = 0;

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
  }

  @Override
  public void init_loop() {

  }

  @Override
  public void start() {
    /*Test for color sensor*/

  }

  //This method acts as a while loop
  @Override
  public void loop() {
    /*//Driver Controller



    telemetry.addData("Status", "Run Time :" + runtime.toString());
    telemetry.addData("Left Encoder", " :" + leftMotor.getCurrentPosition());
    //telemetry.addData("Current Ticks", " :" + currentTick);
    //telemetry.addData("Older Ticks", " :" + olderTick);
    telemetry.addData("Last second ticks", " :" + lastSecondsTick);
    telemetry.addData("Left Speed in inches per second", " :" + motorSpeed);
    telemetry.addData("Updating...", " " + runtime.seconds());

    //realTimeTicks = leftMotor.getCurrentPosition() + realTimeTicks;
    //calcMotorSpeed(wheelDiameter ,leftMotor.getCurrentPosition());

    //For the first second
    if (speedTimer.seconds() >= 1){
      //For the following Seconds
      currentTick = leftMotor.getCurrentPosition();
      lastSecondsTick = getLastSecondTick(olderTick, currentTick);
      motorSpeed = calcMotorSpeed(wheelDiameter ,lastSecondsTick,speedTimer.seconds());
      olderTick = leftMotor.getCurrentPosition();
      speedTimer.reset();
    }

    if(leftMotor.getCurrentPosition() >= motorTarget){
      //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      leftMotor.setPower(0);
    }*/


  }


  public void haltDrive(){
    robot.frontLeftMotor.setPower(0);
    robot.frontRightMotor.setPower(0);
    robot.backLeftMotor.setPower(0);
    robot.backRightMotor.setPower(0);
  }

  public void haltALL(){
    haltDrive();
    robot.intakeMotor.setPower(0);
    robot.shooterMotor.setPower(0);
    robot.elevatorMotor.setPower(0);
  }

  public void moveForwardTo(int moveInRotations){
    robot.frontLeftMotor.setTargetPosition(frontLeftTarget + moveInRotations * ticksPerRotation);
    robot.frontLeftMotor.setPower(1);
    robot.frontRightMotor.setTargetPosition(frontRightTarget + moveInRotations * ticksPerRotation);
    robot.frontRightMotor.setPower(1);
    robot.backLeftMotor.setTargetPosition(backLeftTarget + moveInRotations * ticksPerRotation);
    robot.backLeftMotor.setPower(1);
    robot.backRightMotor.setTargetPosition(backRightTarget + moveInRotations * ticksPerRotation);
    robot.backRightMotor.setPower(1);
  }

  public void moveBackTo(int moveInRotations){
    robot.frontLeftMotor.setTargetPosition(frontLeftTarget - moveInRotations * ticksPerRotation);
    robot.frontLeftMotor.setPower(-1);
    robot.frontRightMotor.setTargetPosition(frontRightTarget - moveInRotations * ticksPerRotation);
    robot.frontRightMotor.setPower(-1);
    robot.backLeftMotor.setTargetPosition(backLeftTarget - moveInRotations * ticksPerRotation);
    robot.backLeftMotor.setPower(-1);
    robot.backRightMotor.setTargetPosition(backRightTarget - moveInRotations * ticksPerRotation);
    robot.backRightMotor.setPower(-1);
  }

  public void turnLeftTo(int moveInRotations){
    robot.frontLeftMotor.setTargetPosition(frontLeftTarget + moveInRotations * ticksPerRotation);
    robot.frontLeftMotor.setPower(1);
    robot.backLeftMotor.setTargetPosition(backLeftTarget + moveInRotations * ticksPerRotation);
    robot.backLeftMotor.setPower(1);
    robot.frontRightMotor.setTargetPosition(frontRightTarget - moveInRotations * ticksPerRotation);
    robot.frontRightMotor.setPower(-1);
    robot.backRightMotor.setTargetPosition(backRightTarget - moveInRotations * ticksPerRotation);
    robot.backRightMotor.setPower(-1);
  }

  public void turnRightTo(int moveInRotations){
    robot.frontLeftMotor.setTargetPosition(frontLeftTarget - moveInRotations * ticksPerRotation);
    robot.frontLeftMotor.setPower(-1);
    robot.backLeftMotor.setTargetPosition(backLeftTarget - moveInRotations * ticksPerRotation);
    robot.backLeftMotor.setPower(-1);
    robot.frontRightMotor.setTargetPosition(frontRightTarget + moveInRotations * ticksPerRotation);
    robot.frontRightMotor.setPower(1);
    robot.backRightMotor.setTargetPosition(backRightTarget + moveInRotations * ticksPerRotation);
    robot.backRightMotor.setPower(1);
  }

  public void moveElevator(int rotationTime){

    robot.elevatorMotor.setPower(1);
    try {
      Thread.sleep(rotationTime * 100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    robot.elevatorMotor.setPower(0);

  }

  public void shootBall(){
    robot.shooterMotor.setTargetPosition(shooterTarget + ticksPerRotation);
    robot.shooterMotor.setPower(1);
  }

  public void pushButton(int pushCase){
    //Left is 1 and right is 2
    switch (pushCase) {
      case 1: robot.leftServo.setPosition(1);
        break;
      case 2: robot.rightServo.setPosition(1);
        break;
    }
  }

  public boolean findLeftColor(String findColor){
    return true;
  }

  public boolean findRightColor(String findColor){
    return true;
  }

}
 