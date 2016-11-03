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

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Test: Motor Encoder", group = "Concept")
//@TeleOp(name = "Test: Motor Encoder", group = "Linear Opmode")
public class PiRhoAutoMaster extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime speedTimer = new ElapsedTime();

  //Initialize Variables
  DcMotor frontLeftMotor = null;
  DcMotor frontRightMotor = null;
  DcMotor backLeftMotor = null;
  DcMotor backRightMotor = null;
  DcMotor intakeSpinnerMotor = null;
  DcMotor shooterMotor = null;
  DcMotor screwMotor = null;

  ColorSensor leftColorSensor = null;
  ColorSensor rightColorSensor = null;

  Servo leftServo = null;
  Servo rightServo = null;

  //All units here is inches
  private final int ticksPerRotation = 1120;
  private int motorTarget = 3 * ticksPerRotation;
  private int realTimeTicks = 0;
  private final int GEAR_RATIO = 1;
  private double distance = 12; //in inches, so far this is test code

  private double time = 0;
  private final double wheelDiameter = 3;
  private final  double CIRCUMFERENCE = Math.PI * wheelDiameter;
  private double ROTATIONS = distance / CIRCUMFERENCE;
  public double COUNTS = ticksPerRotation * ROTATIONS * GEAR_RATIO;

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

    frontLeftMotor = hardwareMap.dcMotor.get("front left motor");
    frontRightMotor = hardwareMap.dcMotor.get("front right motor");
    backLeftMotor = hardwareMap.dcMotor.get("back left motor");
    backRightMotor = hardwareMap.dcMotor.get("back right motor");
    //shooterMotor = hwMap.dcMotor.get("shooter motor");
    //intakeSpinnerMotor = hwMap.dcMotor.get("intake motor");
    //screwMotor = hwMap.dcMotor.get("screw motor");

    leftServo = hardwareMap.servo.get("left servo");
    rightServo = hardwareMap.servo.get("right servo");


    frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
    backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

    frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


  }

  /*
   * This method will be called ONCE when start is pressed
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  @Override
  public void start() {
    /*//Test for color sensor

    runtime.reset();

    leftMotor.getCurrentPosition();
    leftMotor.setTargetPosition(this.motorTarget);
    leftMotor.setPower(1);*/

  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  //The code for the robot while running driver control
  //This method acts as a while loop
  //@Override
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


  public void halt()
  {
    frontLeftMotor.setPower(0);
    frontRightMotor.setPower(0);
    backLeftMotor.setPower(0);
    backRightMotor.setPower(0);
  }

  public void haltALL()
  {
    halt();
    intakeSpinnerMotor.setPower(0);
    shooterMotor.setPower(0);
    screwMotor.setPower(0);
  }
  //Gets ticks from last second
  public double getLastSecondTick(double lastTick, double currentTick){

    return (currentTick - lastTick);

  }

  //Calculates speed
  public double calcMotorSpeed(double diameter, double ticksLastSecond, double currentTime){

    return (diameter * Math.PI * (ticksLastSecond / this.ticksPerRotation))/currentTime;

  }

  public void switchDirection(){

  }
}
 