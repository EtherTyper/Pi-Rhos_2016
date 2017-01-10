/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * This OpMode uses the common HardwareK9bot class to define the devices on the robot.
 * All device access is managed through the HardwareK9bot class. (See this class for device names)
 * The code is structured as a LinearOpMode
 *
 * This particular OpMode executes a basic Tank Drive Teleop for the K9 bot
 * It raises and lowers the arm using the Gampad Y and A buttons respectively.
 * It also opens and closes the claw slowly using the X and B buttons.
 *
 * Note: the configuration of the servos is such that
 * as the arm servo approaches 0, the arm position moves up (away from the floor).
 * Also, as the claw servo approaches 0, the claw opens up (drops the game element).
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name="Test Servo Resting Pos", group="Linear Opmode")
public class TeleOpTestServo extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareConfiguration3rdEdition robot = new HardwareConfiguration3rdEdition();              // Use a K9'shardware
    //double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    //double          clawPosition    = robot.CLAW_HOME;                  // Servo safe position
    //final double    CLAW_SPEED      = 0.01 ;                            // sets rate to move servo
    //final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

    //Other Vaiables
    private boolean shooterActive;
    final int ticksPerRotation = 1120;
    private int shooterTarget = ticksPerRotation;
    @Override
    public void runOpMode() throws InterruptedException {
        //Gamepad 1 Variables
        boolean one;                //left drivebase speed
        boolean two;               //right drivebase speed
        boolean three;  //toggles the orientation of the drivebase
        boolean four;       //extends left beacon presser
        boolean five;       //retracts left beacon presser
        boolean six;      //retracts right beacon presser
        boolean seven;
        boolean eight;
        boolean nine;
        boolean ten;



        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (/*opModeIsActive()*/true) {

            telemetry.addData("Say","Op mode Active.");
            //Gamepad 1 controls
            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            one = gamepad1.y;
            two = gamepad1.b;
            three = gamepad1.a;
            four = gamepad1.x;
            five = gamepad1.left_bumper;
            six = gamepad1.right_bumper;
            seven = gamepad1.dpad_up;
            eight = gamepad1.dpad_right;
            nine = gamepad1.dpad_down;
            ten = gamepad1.dpad_left;

            if(one) {
                robot.beaconServo.setPosition(0);
            }
            else if(two){
                robot.beaconServo.setPosition(0.1);
            }
            else if(three){
                robot.beaconServo.setPosition(0.2);
            }
            else if(four){
                robot.beaconServo.setPosition(0.3);
            }
            else if(five){
                robot.beaconServo.setPosition(0.4);
            }
            else if(six){
                robot.beaconServo.setPosition(0.5);
            }
            else if(seven){
                robot.beaconServo.setPosition(0.6);
            }
            else if(eight){
                robot.beaconServo.setPosition(0.7);
            }
            else if(nine){
                robot.beaconServo.setPosition(0.8);
            }
            else if(ten){
                robot.beaconServo.setPosition(0.9);
            }
            telemetry.addData("servoPos",  robot.beaconServo.getPosition());
            telemetry.update();

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
