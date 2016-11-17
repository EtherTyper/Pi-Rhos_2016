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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Test: Motor Encoder", group = "Concept")
//@TeleOp(name = "Test: Motor Encoder", group = "Linear Opmode")
public class PiRhoAutoMasterTest extends LinearOpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime speedTimer = new ElapsedTime();
 // private HardwareConfigurationMax robot = new HardwareConfigurationMax();

  //All units here is inches
  private final int ticksPerRotation = 1120;

  DcMotor frontLeftMotor   = null;
  DcMotor frontRightMotor  = null;
  DcMotor backLeftMotor = null;
  DcMotor backRightMotor = null;
  DcMotor shooterMotor = null;
  DcMotor intakeMotor = null;
  DcMotor elevatorMotor = null;
  Servo leftServo = null;
  Servo rightServo = null;

  private int frontRightTarget = 0;
  private int frontLeftTarget = 0;
  private int backRightTarget = 0;
  private int backLeftTarget = 0;
  private int shooterTarget = 0;


  public  void my_init() {
    telemetry.addData("Status", "About to initialize");
    frontLeftMotor = hardwareMap.dcMotor.get("front left motor");
    frontRightMotor = hardwareMap.dcMotor.get("front right motor");
    backLeftMotor = hardwareMap.dcMotor.get("back left motor");
    backRightMotor = hardwareMap.dcMotor.get("back right motor");
    shooterMotor = hardwareMap.dcMotor.get("shooter motor");
    intakeMotor = hardwareMap.dcMotor.get("intake motor");
    elevatorMotor = hardwareMap.dcMotor.get("elevator motor");

    leftServo = hardwareMap.servo.get("left beacon");
    rightServo = hardwareMap.servo.get("right beacon");

    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);




    frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

    intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    elevatorMotor.setDirection(DcMotor.Direction.REVERSE);
    rightServo.setDirection(Servo.Direction.REVERSE);

    frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    telemetry.addData("Status", "Initialized");
  }

  @Override
  public void runOpMode() throws InterruptedException {

  }
  //@Override
  //public void init_loop() {

  //}

  //@Override
  //public void start() {
    /*Test for color sensor*/

 // }

  //This method acts as a while loop
  //@Override
  //public void loop() {
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


  //}

  public void delay(int time)
  {
    try {
      Thread.sleep(time);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  public void resetEncoder()
  {shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);}


  public void haltDrive(){
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
  }

  public void haltALL(){
    haltDrive();
    intakeMotor.setPower(0);
    shooterMotor.setPower(0);
    elevatorMotor.setPower(0);
  }

  public void moveForwardTo(int moveInRotations){
    frontLeftMotor.setTargetPosition(frontLeftTarget + moveInRotations * ticksPerRotation);
    frontLeftMotor.setPower(1);
    frontRightMotor.setTargetPosition(frontRightTarget + moveInRotations * ticksPerRotation);
    frontRightMotor.setPower(1);
    backLeftMotor.setTargetPosition(backLeftTarget + moveInRotations * ticksPerRotation);
    backLeftMotor.setPower(1);
    backRightMotor.setTargetPosition(backRightTarget + moveInRotations * ticksPerRotation);
    backRightMotor.setPower(1);
  }

  public void moveBackTo(int moveInRotations){
    frontLeftMotor.setTargetPosition(frontLeftTarget - moveInRotations * ticksPerRotation);
    frontLeftMotor.setPower(-1);
    frontRightMotor.setTargetPosition(frontRightTarget - moveInRotations * ticksPerRotation);
    frontRightMotor.setPower(-1);
    backLeftMotor.setTargetPosition(backLeftTarget - moveInRotations * ticksPerRotation);
    backLeftMotor.setPower(-1);
    backRightMotor.setTargetPosition(backRightTarget - moveInRotations * ticksPerRotation);
    backRightMotor.setPower(-1);
  }

  public void turnLeftTo(int moveInRotations){
    frontLeftMotor.setTargetPosition(frontLeftTarget + moveInRotations * ticksPerRotation);
    frontLeftMotor.setPower(1);
    backLeftMotor.setTargetPosition(backLeftTarget + moveInRotations * ticksPerRotation);
    backLeftMotor.setPower(1);
    frontRightMotor.setTargetPosition(frontRightTarget - moveInRotations * ticksPerRotation);
    frontRightMotor.setPower(-1);
    backRightMotor.setTargetPosition(backRightTarget - moveInRotations * ticksPerRotation);
    backRightMotor.setPower(-1);
  }

  public void turnRightTo(int moveInRotations){
    frontLeftMotor.setTargetPosition(frontLeftTarget - moveInRotations * ticksPerRotation);
    frontLeftMotor.setPower(-1);
    backLeftMotor.setTargetPosition(backLeftTarget - moveInRotations * ticksPerRotation);
    backLeftMotor.setPower(-1);
    frontRightMotor.setTargetPosition(frontRightTarget + moveInRotations * ticksPerRotation);
    frontRightMotor.setPower(1);
    backRightMotor.setTargetPosition(backRightTarget + moveInRotations * ticksPerRotation);
    backRightMotor.setPower(1);
  }

  public void moveElevator(int rotationTime){

   elevatorMotor.setPower(1);
    try {
      Thread.sleep(rotationTime * 100);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
    elevatorMotor.setPower(0);

  }

  public void shootBall(){
    shooterMotor.setTargetPosition(ticksPerRotation);
    shooterMotor.setPower(1);
  }

  public void pushButton(int pushCase){
    //Left is 1 and right is 2
    switch (pushCase) {
      case 1: leftServo.setPosition(1);
        break;
      case 2: rightServo.setPosition(1);
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
 