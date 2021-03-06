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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.FinalHardwareConfiguration;

//import static java.lang.thread;

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Test: Motor Encoder", group = "Concept")
//@TeleOp(name = "Final: General Drive Control", group = "Linear Opmode")
public class FinalGeneralDriverControl extends OpMode {

  //Objects setup
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime speedTimer = new ElapsedTime();
  FinalHardwareConfiguration robot = new FinalHardwareConfiguration();

  //Integer Varibles (All units here is inches)
  private final int ticksPerRotation = 1120;
  private int shooterTarget = 0;
  private int motorTarget = 3 * ticksPerRotation;
  private int realTimeTicks = 0;

  //Double Variables
  private double time = 0;
  private double wheelDiameter = 2.75;
  private double olderTick = 0;
  private double currentTick = 0;
  private double lastSecondsTick = 0;
  private double motorSpeed = 0;
  private double left;
  private double right;

  //Boolean Variables
  private boolean reverseMode = false;
  private boolean preciseMode = false;
  private boolean shooterActive = false;
  private boolean intakeActive = false;
  private boolean elevatorActive = false;
  private boolean leftServoActive = false;
  private boolean rightServoActive = false;

  //Runs once when init is pressed
  @Override
  public void init() {
    robot.init(hardwareMap);
    telemetry.addData("Status", "(Initialized) Setup");

    robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
  }

  //Runs after init is pressed (loop)
  @Override
  public void init_loop() {
    telemetry.addData("Status", "(Initialized) Loop");

  }

  //Runs once when start button is pressed
  @Override
  public void start() {
    telemetry.addData("Status", "(Running) Setup");
    runtime.reset();
    speedTimer.reset();

  }

  //          ===Robot Functions===        //

  //Test Shooter System
  void testShooterSystem(){

    robot.shooterMotor.setPower(gamepad2.right_stick_y);

  }

  //Shooter System
  void shooterSystem(){
    if(gamepad2.a && !shooterActive){
      shooterTarget = shooterTarget + ticksPerRotation;
      robot.shooterMotor.setTargetPosition(shooterTarget);
      robot.shooterMotor.setPower(1);
      this.shooterActive = true;
    }

    if(robot.shooterMotor.getCurrentPosition() == shooterTarget){
      robot.shooterMotor.setPower(0);
      this.shooterActive = false;
    }
  }

  //Drive System
  void driveSystem(){

    left = -gamepad1.left_stick_y;
    right = -gamepad1.right_stick_y;


    if(preciseMode)
    {
      left*=0.25;
      right*=0.25;

    }

    if(reverseMode)
    {
      robot.frontLeftMotor.setPower(-right);
      robot.backLeftMotor.setPower(-right);

      robot.frontRightMotor.setPower(-left);
      robot.backRightMotor.setPower(-left);

    }
    else
    {
      robot.frontLeftMotor.setPower(left);
      robot.backLeftMotor.setPower(left);

      robot.frontRightMotor.setPower(right);
      robot.backRightMotor.setPower(right);
    }

    if(gamepad1.x)
    {
      reverseMode = !reverseMode;
      try {
        Thread.sleep(400);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
    if(gamepad1.y)  //lower power for more precise movement
    {
      preciseMode = !preciseMode;
      try {
        Thread.sleep(400);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }
  }

  //Beacon Pusher
  void beaconPusher(){

    leftServoActive = false;
    rightServoActive = false;

    if(gamepad1.right_bumper){
      robot.rightServo.setPosition(0);
      leftServoActive = true;
    } else if (gamepad1.right_trigger >= 0.5){
      robot.rightServo.setPosition(1);
    }

    if(gamepad1.left_bumper){
      robot.leftServo.setPosition(0);
      rightServoActive = true;
    } else if (gamepad1.left_trigger >= 0.5){
      robot.leftServo.setPosition(-1);
    }

  }

  //Intake System
  void intakeSystem(){

    //Intake
    if(!intakeActive && gamepad2.x) {

      robot.intakeMotor.setPower(1);
      intakeActive = true;

    }

    if(!intakeActive && gamepad2.b) {

      robot.intakeMotor.setPower(-1);
      intakeActive = true;

    }

    if (intakeActive && (gamepad2.x || gamepad2.b)){

      robot.intakeMotor.setPower(0);
      intakeActive = false;

    }

    else
    {
      robot.intakeMotor.setPower(0);
      intakeActive = false;
    }
  }

  //Elevator System
  void elevatorSystem(){

    if(gamepad2.dpad_up){
      robot.elevatorMotor.setPower(-1);

    } else if (gamepad2.dpad_down){

      robot.elevatorMotor.setPower(1);

    } else {

      robot.elevatorMotor.setPower(0);

    }
  }

  //        ===Main Loop===        //
  //Runs when start is pressed (loop)
  @Override
  public void loop() {

    telemetry.addData("Status", "(Running) Main Loop");

    //Method Calls
    //shooterSystem();
    testShooterSystem();
    driveSystem();
    beaconPusher();
    elevatorSystem();
    intakeSystem();

  }
}
