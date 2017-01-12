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

/**
 * Demonstrates empty OpMode
 */
@Autonomous(name = "Autonomous 4th edition Blue", group = "Concept")
public class Autonomous4thEditionBlue extends Autonomous4thEdition {

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


      extendBeaconPresser(0.5,1000);
      retractBeaconPresser(0.5,1000);
      //pressBeacon();
      //extendBeaconPresser();
      //delay(2000);
      //CURRENT AUTONOMOUS CODE
      //move to shooting position
      /*moveForwardTo(calcDrive(5),0.2, 0.2);
      turnLeftVariableAndStop(0.3, 50);
      moveForwardTo(calcDrive(20),0.2, 0.2);

      //Shoot Ball One
      delay(1000);
      shootBall();

      //Load ball two
      delay(1000);
      moveScrewUp();
      delay(1000);

      //Shoot Ball Two
      shootBall();
      delay(1000);

      //drive to far beacon
      turnLeftVariableAndStop(0.3, 30);
      moveForwardTo(calcDrive(82), 0.7, 0.7);
      turnRightVariableAndStop(0.3, 59);
      moveForwardTo(calcDrive(24),0.3, 0.3);
      stopOnLineFoward(0.3);

      //correct for color

      //press beacon

      //move back to close beacon
      moveBackTo(calcDrive(24), 0.28, 0.3);
      stopOnLineBackward(0.3);
      //END OF WORKING AUTONOMOUS CODE*/

      //Beacon sensing







      //Stop All Movement
      //resetEncoders();


  }
}
