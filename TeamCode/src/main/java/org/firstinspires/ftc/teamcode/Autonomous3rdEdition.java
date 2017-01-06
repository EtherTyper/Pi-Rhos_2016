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
@Autonomous(name = "Autonomous 3rd Edition", group = "Concept")
public class Autonomous3rdEdition extends LinearOpMode {
  private HardwareConfiguration3rdEdition robot = new HardwareConfiguration3rdEdition();

  //All units here is inches
  private final int ticksPerRotation = 1120;
  private final int TOLERANCE = 100;
  int step = 0;

  //Calculation Variables
  double MOTOR_CPR = 1120;
  double GEAR_RATIO = 27.0/40.0;
  double WHEEL_DIAMETER = 3.5;
  double distance = 0;
  final double radius = 7.44;
  final double driveSpeed = 0.2;
  double CIRCUMFERENCE = 0;
  double ROTATIONS = 0;
  int counts = 0;
  int threshold = 2;


  //Initialization
  public void init_hardware() {
    telemetry.addData("Status", "Initialized");

    robot.init(hardwareMap);

    //robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    //robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    //robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    robot.gyro.calibrate();
    robot.lineColorSensor.enableLed(true);
    robot.frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    robot.frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
  }

  //Stop Function
  public void delay(long mils){
    try {
      Thread.sleep(mils);
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }

  public int absHeading(){
      return Math.abs(robot.gyro.getHeading());
  }

  public void haltDrive(){
        //robot.frontLeftMotor.setPower(0);
        //robot.frontRightMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

  public void haltALL(){
        haltDrive();
        robot.intakeMotor.setPower(0);
        robot.shooterMotor.setPower(0);
        robot.elevatorMotor.setPower(0);
    }

  public void resetEncoders() {
        robot.shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        delay(300);
    }

  public void shootBall(){

      robot.shooterMotor.setTargetPosition(robot.shooterMotor.getCurrentPosition() + ticksPerRotation);
      robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.shooterMotor.setPower(1);

  }

  public void moveScrewUp(){

      robot.elevatorMotor.setTargetPosition(ticksPerRotation * 2);
      robot.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.elevatorMotor.setPower(1);

  }

  //Robot Movement

    public void goFoward(double power){
        robot.backLeftMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        telemetry.addData("Motor Power", robot.backLeftMotor.getPower());
        telemetry.addData("Motor Variable", power);
    }

  public void moveBackTo(int moveInRotations){
      //robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() - moveInRotations);
      //robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() - moveInRotations);
      robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - moveInRotations);
      robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() - moveInRotations);
      //robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while(!((robot.backLeftMotor.getCurrentPosition()<= robot.backLeftMotor.getTargetPosition()+TOLERANCE) &&
              (robot.backRightMotor.getCurrentPosition()<= robot.backRightMotor.getTargetPosition()+TOLERANCE /*&&
              robot.backRightMotor.getCurrentPosition()>=robot.backRightMotor.getTargetPosition()-TOLERANCE*/))){

        //robot.frontLeftMotor.setPower(-driveSpeed);

        //robot.frontRightMotor.setPower(-driveSpeed);

        robot.backLeftMotor.setPower(-driveSpeed);

        robot.backRightMotor.setPower(-driveSpeed);

    }
      haltDrive();
  }

  public void moveForwardTo(int moveInRotations){
      //robot.frontLeftMotor.setTargetPosition(robot.frontLeftMotor.getCurrentPosition() + moveInRotations);
      //robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + moveInRotations);
      robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + moveInRotations);
      robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + moveInRotations);
      //robot.frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      while(!((robot.backLeftMotor.getCurrentPosition()>= robot.backLeftMotor.getTargetPosition()-TOLERANCE) &&
                (robot.backRightMotor.getCurrentPosition()>= robot.backRightMotor.getTargetPosition()-TOLERANCE /*&&
                 robot.backRightMotor.getCurrentPosition()<=robot.backRightMotor.getTargetPosition()+TOLERANCE*/))){

          //robot.frontLeftMotor.setPower(driveSpeed);

          //robot.frontRightMotor.setPower(driveSpeed);

          robot.backLeftMotor.setPower(driveSpeed);

          robot.backRightMotor.setPower(driveSpeed);
      }
      haltDrive();
    }

  //Turning

  public void correctTurn(double power, double headingGoal){
        if(absHeading() < (headingGoal - threshold)){
            calcTurn(headingGoal-absHeading());
            //robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() - counts);
            robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + counts);
            robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() - counts);
            //robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(!((robot.backLeftMotor.getCurrentPosition()<= robot.backLeftMotor.getTargetPosition()-TOLERANCE) &&
                    (robot.backRightMotor.getCurrentPosition()>=robot.backRightMotor.getTargetPosition()+TOLERANCE))){

                //robot.frontLeftMotor.setPower(power);

               //robot.frontRightMotor.setPower(-power);

                robot.backLeftMotor.setPower(power);

                robot.backRightMotor.setPower(-power);
            }
        }
        else if(absHeading() > headingGoal + threshold){
            calcTurn(absHeading()-headingGoal);
            //robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + counts);
            robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - counts);
            robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + counts);
            //robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(!((robot.backLeftMotor.getCurrentPosition()>= robot.backLeftMotor.getTargetPosition()+TOLERANCE) &&
                    (robot.backRightMotor.getCurrentPosition()<=robot.backRightMotor.getTargetPosition()-TOLERANCE))){

                //robot.frontLeftMotor.setPower(-power);

                //robot.frontRightMotor.setPower(power);

                robot.backLeftMotor.setPower(-power);

                robot.backRightMotor.setPower(power);
            }
        }
        haltDrive();
    }
    public void turnLeftVariableAndStop(double power, int degrees) {

          /*while (true/*absHeading() < (heading - threshold) || absHeading() > heading + threshold) {
              robot.frontLeftMotor.setPower(power);
              robot.backLeftMotor.setPower(power);
              robot.frontRightMotor.setPower(-power);
              robot.backRightMotor.setPower(-power);
          }
          robot.frontLeftMotor.setPower(0);
          robot.backLeftMotor.setPower(0);
          robot.frontRightMotor.setPower(0);
          robot.backRightMotor.setPower(0);*/
        calcTurn(degrees);
        //robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() - counts);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + counts);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() - counts);
        //robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!((robot.backLeftMotor.getCurrentPosition()<= robot.backLeftMotor.getTargetPosition()-TOLERANCE) &&
                (robot.backRightMotor.getCurrentPosition()>=robot.backRightMotor.getTargetPosition()+TOLERANCE))){

            //robot.frontLeftMotor.setPower(power);

            //robot.frontRightMotor.setPower(-power);

            robot.backLeftMotor.setPower(power);

            robot.backRightMotor.setPower(-power);
        }
        haltDrive();
        //correctTurn(power,heading);
    }


    public void turnRightVariableAndStop(double power, int degrees) {
        calcTurn(degrees);
        //robot.frontRightMotor.setTargetPosition(robot.frontRightMotor.getCurrentPosition() + counts);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - counts);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + counts);
        //robot.frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!((robot.backLeftMotor.getCurrentPosition()<= robot.backLeftMotor.getTargetPosition()+TOLERANCE) &&
                (robot.backRightMotor.getCurrentPosition()>=robot.backRightMotor.getTargetPosition()-TOLERANCE))){

            //robot.frontLeftMotor.setPower(-power);

            //robot.frontRightMotor.setPower(power);

            robot.backLeftMotor.setPower(-power);

            robot.backRightMotor.setPower(power);
        }
        haltDrive();

        //correctTurn(power,degrees);

    }

  public void turnLeftTest(double power, int heading) {
        while (absHeading() > ((360 - heading) - threshold)) {

            //robot.frontLeftMotor.setPower(-power);
            robot.backLeftMotor.setPower(-power);
            //robot.frontRightMotor.setPower(power);
            robot.backRightMotor.setPower(power);
        }

        //robot.frontLeftMotor.setPower(0);
        robot.backLeftMotor.setPower(0);
        //robot.frontRightMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }

  public void turnRightTest(double power, int heading) {

            while (absHeading() > ((360 - heading) - threshold)) {

                //robot.frontLeftMotor.setPower(-power);
                robot.backLeftMotor.setPower(-power);
                //robot.frontRightMotor.setPower(power);
                robot.backRightMotor.setPower(power);
            }

            //robot.frontLeftMotor.setPower(0);
            robot.backLeftMotor.setPower(0);
            //robot.frontRightMotor.setPower(0);
            robot.backRightMotor.setPower(0);

    }

    //Color Sensors
    public boolean onWhiteLine(){
        if(robot.lineColorSensor.alpha()>=40){
            return true;
        }
        return false;
    }

    public void stopOnLine(double power){
        telemetry.addData("loop","not init");
        telemetry.update();
        while(!(robot.lineColorSensor.alpha()>=40)){
            telemetry.addData("loop","init");
            goFoward(power);
            telemetry.addData("alpha",robot.lineColorSensor.alpha());
            telemetry.update();
        }
        robot.backLeftMotor.setPower(0);
        robot.backRightMotor.setPower(0);
    }
    public boolean rightBeaconIsRed(){
        if(robot.rightColorSensor.red()>220){
            return true;
        }
        else{
            return false;
        }

    }
    public void pressBeacon(){
        if(rightBeaconIsRed()){
            //move and extend beacon presser
        }
        else{
            //move and extend beacon presser
        }
    }


  //Calc Rotations -> Converts inches into ticks
  public int calcDrive(double dist){
    CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
    ROTATIONS = dist/CIRCUMFERENCE;
    counts = (int)(ROTATIONS*GEAR_RATIO*MOTOR_CPR);
    return counts;
  }

  public int calcTurn(double deg){
      double dist = deg*2.0*radius*Math.PI/360.0;
      CIRCUMFERENCE = WHEEL_DIAMETER*Math.PI;
      ROTATIONS = dist/CIRCUMFERENCE;
      counts = (int)(ROTATIONS*GEAR_RATIO*MOTOR_CPR);
      return counts;
  }

  @Override
  public void runOpMode() throws InterruptedException {
      telemetry.addData("Stage: ", "Called runOpMode");
      telemetry.update();
      //Initialize Robot
      //robot.lineColorSensor.enableLed(true);
      init_hardware();
      resetEncoders();
      waitForStart();
      haltALL();

      /*delay(1000);
      shootBall();
      //Shoot Ball One
      delay(1000);
      moveScrewUp();
      delay(1000);

      //Shoot Ball Two
      shootBall();
      delay(1000);*/

      //Move the Robot
      //moveForwardTo(calcDrive(10));
      //waitForGyroToCalibrate();
      /*telemetry.addData("FL power",robot.frontLeftMotor.getPower());
      telemetry.addData("BL power",robot.backLeftMotor.getPower());
      telemetry.addData("FR power",robot.frontRightMotor.getPower());
      telemetry.addData("BR power",robot.backRightMotor.getPower());
      telemetry.update();*/


      //Working Drive Code
      /*
      moveForwardTo(calcDrive(24));
      haltDrive();
      moveBackTo(calcDrive(24));
      haltDrive();
      turnLeftVariableAndStop(.2,90);
      */
      //stopOnLine(0.5);
      turnLeftVariableAndStop(0.3,90);
      delay(5000);
      turnRightVariableAndStop(0.3,90);


      /*
      resetEncoders();
      moveBackTo(calcDrive(24));
      resetEncoders();
      delay(2000);
      telemetry.addData("Step 2: ", "turn init");
      telemetry.update();
      turnRightTest(1,90);
      telemetry.addData("Current Heading", robot.gyro.getHeading());*/

      //Stop All Movement
      //resetEncoders();


  }
}
