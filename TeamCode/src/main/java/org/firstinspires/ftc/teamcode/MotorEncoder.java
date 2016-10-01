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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Test: Motor Encoder", group = "Concept")
@TeleOp(name = "Test: Motor Encoder", group = "Linear Opmode")
public class MotorEncoder extends OpMode {

  private ElapsedTime runtime = new ElapsedTime();

  //Initialize Variables
  DcMotor leftMotor = null;
  DcMotor rightMotor = null;

  //All units here is inches
  private final int ticksPerRotation = 1120;
  private int motorTarget = 21 * ticksPerRotation;
  private int realTimeTicks = 0;

  private double time = 0;
  private double wheelDiameter = 3.5;
  private double olderTick = 0;
  private double currentTick = 0;
  private double lastSecondsTick = 0;

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
    runtime.reset();

    leftMotor.getCurrentPosition();
    leftMotor.setTargetPosition(this.motorTarget);
    leftMotor.setPower(1);

  }

  /*
   * This method will be called repeatedly in a loop
   * @see com.qualcomm.robotcore.eventloop.opmode.OpMode#loop()
   */
  //The code for the robot while running driver control
  //This method acts as a while loop
  @Override
  public void loop() {
    telemetry.addData("Status", "Run Time: " + runtime.toString());
    telemetry.addData("Left Encoder", ":" + leftMotor.getCurrentPosition());

    //realTimeTicks = leftMotor.getCurrentPosition() + realTimeTicks;
    //calcMotorSpeed(wheelDiameter ,leftMotor.getCurrentPosition());

    //Runs after a whole second, starts from one
    if((((runtime.seconds()) % 1) == 0)){
      //For the first second
      if(runtime.seconds() <= 0.01) {
        currentTick = leftMotor.getCurrentPosition();
        lastSecondsTick = getLastSecondTick(0, currentTick);
        olderTick = leftMotor.getCurrentPosition();
        calcMotorSpeed(wheelDiameter ,lastSecondsTick);
        telemetry.addData("Current Ticks: ", ":" + currentTick);
        telemetry.addData("Older Ticks", ":" + olderTick);
        telemetry.addData("Last second ticks: ", ":" + lastSecondsTick);
      } else if ((runtime.seconds()) % 1 <= 0.01){
        //For the following Seconds
        currentTick = leftMotor.getCurrentPosition();
        lastSecondsTick = getLastSecondTick(olderTick, currentTick);
        olderTick = leftMotor.getCurrentPosition();
        calcMotorSpeed(wheelDiameter ,lastSecondsTick);
        telemetry.addData("Current Ticks: ", ":" + currentTick);
        telemetry.addData("Older Ticks", ":" + olderTick);
        telemetry.addData("Last second ticks: ", ":" + lastSecondsTick);
      }
    }

    if(leftMotor.getCurrentPosition() >= motorTarget){
      //leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      leftMotor.setPower(0);
    }


  }

  public double getLastSecondTick(double lastTick, double currentTick){

    return (currentTick - lastTick);

  }

  //Custom Methods
  public void calcMotorSpeed(double diameter, double ticksLastSecond){

    double motorSpeed = diameter * Math.PI * ticksLastSecond / 1120;
    telemetry.addData("Left Speed in inches per second", ":" + motorSpeed);


  }

}
