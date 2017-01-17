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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Turn Auto Test", group = "Concept")
public class TurnTest extends OpMode {

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
  int step = 0;
  int MOTOR_CPR = 1120;
  double GEAR_RATIO = 27/40;
  double WHEEL_DIAMETER = 3.5;
  double distance = 4.45;
  int counts = (int)(((MOTOR_CPR*GEAR_RATIO)/WHEEL_DIAMETER*Math.PI)*distance);
  int flCounts = 0;
  int blCounts = 0;
  int frCounts = 0;
  int brCounts = 0;

  @Override
  public void init() {
    telemetry.addData("Status", "Initialized");

    robot.init(hardwareMap);

    robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

  public void delay(long mils){
    try {
      Thread.sleep(mils);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
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

  public void moveForwardTo(double distance){
    if(step==6) {
      this.distance = distance;
      int targPosLF = robot.frontLeftMotor.getCurrentPosition() + counts;
        robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() - counts);
        robot.frontLeftMotor.setPower(1);
        robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + counts);
        robot.frontRightMotor.setPower(1);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - counts);
        robot.backLeftMotor.setPower(1);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + counts);
        robot.backRightMotor.setPower(1);
        if(robot.frontLeftMotor.getCurrentPosition()>=robot.frontLeftMotor.getCurrentPosition() - counts){
          step++;
      }
    }
  }

  public void calculateForward(double dist, int stp){
    if(step==stp){
      distance = dist;
      blCounts = robot.backLeftMotor.getCurrentPosition() - counts;
      brCounts = robot.backRightMotor.getCurrentPosition() - counts;
      step++;
    }
  }
  public void calculateTurnRight(double dist, int stp){
    if(step==stp){
      distance = dist;
      blCounts = robot.backLeftMotor.getCurrentPosition() - counts;
      brCounts = robot.backRightMotor.getCurrentPosition() - counts;
      step++;
    }
  }

  public void moveBackTo(double distance){
    this.distance = distance;
    robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() - counts);
    robot.frontLeftMotor.setPower(-1);
    robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() - counts);
    robot.frontRightMotor.setPower(-1);
    robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + counts);
    robot.backLeftMotor.setPower(-1);
    robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + counts);
    robot.backRightMotor.setPower(-1);
  }

  public void turnLeftTo(double distance){
    this.distance = distance;
    robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + counts);
    robot.frontLeftMotor.setPower(1);
    robot.backLeftMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + counts);
    robot.backLeftMotor.setPower(1);
    robot.frontRightMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + counts);
    robot.frontRightMotor.setPower(-1);
    robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + counts);
    robot.backRightMotor.setPower(-1);
  }

  public void turnRightTo(double distance){
    if(step == 4) {
      this.distance = distance;
      robot.frontLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - counts);
      robot.frontLeftMotor.setPower(1);
      robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - counts);
      robot.backLeftMotor.setPower(1);
      if(robot.frontLeftMotor.getCurrentPosition() <= robot.frontRightMotor.getCurrentPosition() - counts){
        step++;
      }
    }
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
    if(step==0) {
      robot.shooterMotor.setTargetPosition(ticksPerRotation);
      robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.shooterMotor.setPower(1);
      if (robot.shooterMotor.getCurrentPosition() >= ticksPerRotation) {
        step++;
      }
    }
  }
  public void shoot2ndBall(){
    if(step==2) {
      robot.shooterMotor.setTargetPosition(ticksPerRotation * 2);
      robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.shooterMotor.setPower(1);
      if(robot.shooterMotor.getCurrentPosition()>=ticksPerRotation * 2){
        step++;
      }
    }
  }
  public void moveScrewUp(){
    if(step==1) {
      robot.elevatorMotor.setTargetPosition(ticksPerRotation * 3);
      robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.elevatorMotor.setPower(1);
      if(robot.elevatorMotor.getCurrentPosition()>=ticksPerRotation*3){
        step++;
      }
    }
  }


  public void pushButton(int pushCase){
    //Left is 1 and right is 2
    switch (pushCase) {
      //case 1: robot.leftServo.setPosition(1);
        //break;
      //case 2: robot.rightServo.setPosition(1);
        //break;
    }
  }

  public boolean findLeftColor(String findColor){
    return true;
  }

  public boolean findRightColor(String findColor){
    return true;
  }

  @Override
  public void loop() {
    shootBall();
    moveScrewUp();
    shoot2ndBall();
    calculateTurnRight(4.45,3);
    turnRightTo(4.45);
    calculateForward(100,5);
    moveForwardTo(100);

    telemetry.addData("Position",robot.shooterMotor.getCurrentPosition());
    telemetry.addData("Step", step);

    //moveForwardTo(3);
    //turnRightTo(2);

  }


}
