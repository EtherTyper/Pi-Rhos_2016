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

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Linear Drive Test", group = "Concept")
//TeleOp(name = "Test: Motor Encoder", group = "Linear Opmode")
public class CompetitionAutoRed extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime speedTimer = new ElapsedTime();
  private HardwareConfigMaxLinearTest robot = new HardwareConfigMaxLinearTest();

  //All units here is inches
  private final int ticksPerRotation = 1120;
  private int frontRightTarget = 0;
  private int frontLeftTarget = 0;
  private int backRightTarget = 0;
  private int backLeftTarget = 0;
  private int shooterTarget = 0;
  int step = 0;
  double MOTOR_CPR = 1120;
  double GEAR_RATIO = 27.0/40.0;
  double WHEEL_DIAMETER = 3.5;
  double distance = 10;

    double CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
    double ROTATIONS = distance/CIRCUMFERENCE;
  int counts = (int)(MOTOR_CPR*ROTATIONS*GEAR_RATIO);



  public void my_init() {
    telemetry.addData("Status", "Initialized");

    robot.init(hardwareMap);

    /*robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

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


  /*public void delay(long mils){
    try {
      Thread.sleep(mils);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }*/

  //Will be uncommented once confirmation of LinearOpMode function is present.
  /*public void haltDrive(){
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
  }*/

 public void shootBall(int ticks){
    while(robot.shooterMotor.getCurrentPosition()<=ticks){
        robot.shooterMotor.setTargetPosition(ticks);
        robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.shooterMotor.setPower(1);
    }
  }

  public void moveScrewUp(){
      while(robot.elevatorMotor.getCurrentPosition()<=ticksPerRotation * 3) {
          robot.elevatorMotor.setTargetPosition(ticksPerRotation * 3);
          robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          robot.elevatorMotor.setPower(1);
      }
      /*if(robot.elevatorMotor.getCurrentPosition()>=ticksPerRotation*3){
        step++;
      }*/
  }

  public void calcDrive(double dist){
      this.distance = dist;
      CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
      ROTATIONS = distance/CIRCUMFERENCE;
      counts = (int)(MOTOR_CPR*ROTATIONS*GEAR_RATIO);
  }
  public void halfForward(double dist){
      calcDrive(dist);
      while(robot.frontLeftMotor.getCurrentPosition()<=counts-100){
          robot.frontLeftMotor.setTargetPosition(counts);
          robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          robot.frontLeftMotor.setPower(.5);

          robot.backLeftMotor.setTargetPosition(counts);
          robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          robot.backLeftMotor.setPower(.5);

          robot.frontRightMotor.setTargetPosition(counts);
          robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          robot.frontRightMotor.setPower(.5);

          robot.backRightMotor.setTargetPosition(counts);
          robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          robot.backRightMotor.setPower(.5);
      }
  }

    public void fullForward(double dist){
        calcDrive(dist);
        while(robot.frontLeftMotor.getCurrentPosition()<=counts){
            robot.frontLeftMotor.setTargetPosition(counts);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setPower(1);

            robot.backLeftMotor.setTargetPosition(counts);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setPower(1);

            robot.frontRightMotor.setTargetPosition(counts);
            robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightMotor.setPower(1);

            robot.backRightMotor.setTargetPosition(counts);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setPower(1);
        }
    }

  public void turnLeft(double angle){
      double dist = (30*Math.PI)*(angle/360);
      calcDrive(dist);
      while(robot.frontRightMotor.getCurrentPosition()<=counts-100){
          robot.frontRightMotor.setTargetPosition(counts);
          robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          robot.frontRightMotor.setPower(.5);

          robot.backRightMotor.setTargetPosition(counts);
          robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          robot.backRightMotor.setPower(.5);
      }
  }

    public void turnRight(double angle){
        double dist = (30*Math.PI)*(angle/360);
        telemetry.addData("Counts",counts);
        calcDrive(dist);
        while(robot.frontRightMotor.getCurrentPosition()<=counts-100){
            telemetry.addData("fl Pos",robot.frontLeftMotor.getCurrentPosition());
            telemetry.update();
            robot.frontLeftMotor.setTargetPosition(counts);
            robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontLeftMotor.setPower(.5);

            robot.backLeftMotor.setTargetPosition(counts);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setPower(.5);
        }
    }
  public void resetEncoders()
  {
    robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  }

  public boolean findLeftColor(String findColor){
    return true;
  }

  public boolean findRightColor(String findColor){
    return true;
  }

  @Override
  public void runOpMode() throws InterruptedException {
      my_init();
      waitForStart();
      shootBall(ticksPerRotation);
      sleep(500);
      moveScrewUp();
      sleep(500);
      shootBall(ticksPerRotation*2);
      sleep(500);
      turnRight(90);
      telemetry.addData("Turned R","");
      telemetry.update();
      resetEncoders();
      turnLeft(90);
      telemetry.addData("Turned L","");
      telemetry.update();
      resetEncoders();
      halfForward(2);
      resetEncoders();
      fullForward(10);
      resetEncoders();
      halfForward(2);
      resetEncoders();

  }

}
