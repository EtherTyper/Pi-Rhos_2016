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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

//import static java.lang.thread
/**
 * Demonstrates empty OpMode
 */
//@Autonomous(name = "Test: Motor Encoder", group = "Concept")
//@TeleOp(name = "Servo Crap", group = "Linear Opmode")
public class ServoTester extends OpMode {

  //Objects setup
  private ElapsedTime runtime = new ElapsedTime();
  private ElapsedTime speedTimer = new ElapsedTime();
  HardwareMap hwMap = hardwareMap;

  //Servo rightServo = hardwareMap.servo.get("right servo");
  Servo rightServo = null;
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
    telemetry.addData("Status", "(Initialized) Setup");
  }

  //Runs after init is pressed (loop)
  @Override
  public void init_loop() {
    telemetry.addData("Status", "(Initialized) Loop");
    rightServo = hardwareMap.servo.get("right servo");
  }

  //Runs once when start button is pressed
  @Override
  public void start() {
    telemetry.addData("Status", "(Running) Setup");
    runtime.reset();
    speedTimer.reset();

  }

  //          ===Robot Functions===        //

  void MoveTheDamServo(double motorPosition) {

    rightServo.setPosition(motorPosition);

  }

  void ControlTheDangServo(double motorPosition) {
    if (gamepad2.a) {


      rightServo.setPosition(motorPosition);
    } else
      rightServo.setPosition(0.5);
  }

    //        ===Main Loop===        //
    //Runs when start is pressed (loop)
    @Override
    public void loop () {

      telemetry.addData("Status", "(Running) Main Loop");
      //MoveTheDamServo(0);
      ControlTheDangServo(1);
      telemetry.addData("Motor position", rightServo.getPosition());

    }

}
