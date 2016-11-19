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
//@TeleOp(name="Max Shooter Test ControllerByPass", group="Linear Opmode")
public class TeleOpMax5 extends LinearOpMode {

    /* Declare OpMode members. */
    HardwareConfigurationMax robot = new HardwareConfigurationMax();              // Use a K9'shardware
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
        double left;                //left drivebase speed
        double right;               //right drivebase speed
        boolean toggleDriveOrient;  //toggles the orientation of the drivebase
        double beaconLeftOut;       //extends left beacon presser
        boolean beaconLeftIn;       //retracts left beacon presser
        double beaconRightOut;      //extends right beacon presser
        boolean beaconRightIn;      //retracts right beacon presser

        //Gamepad 2 Variables
        boolean shoot;              //shoots the ball
        boolean screwUp;            //screw spirals up
        boolean screwDown;          //screw spirals down
        boolean flapsIn = false;    //flaps bring ball in
        boolean toggleFlapsIn;
        boolean flapsOut = false;   //flaps push ball out
        boolean toggleFlapsOut;

        boolean reverseMode = false;
        boolean preciseMode = false;
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
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;
            toggleDriveOrient = gamepad1.x;
            beaconLeftOut = gamepad1.left_trigger;
            beaconLeftIn = gamepad1.left_bumper;
            beaconRightOut = gamepad1.right_trigger;
            beaconRightIn = gamepad1.right_bumper;

            //Gamepad 2 controls
            shoot = true;
            screwUp = gamepad2.dpad_up;
            screwDown = gamepad2.dpad_down;
            toggleFlapsIn = gamepad2.x;
            toggleFlapsOut = gamepad2.b;
            float testShoot = -gamepad2.right_stick_y;
            boolean revShoot = gamepad2.y;
            boolean otherShoot = gamepad2.right_bumper;

            if(preciseMode)
            {
                left*=0.2;
                right*=0.2;

            }

            if(!beaconLeftIn && beaconLeftOut==0)
            {
                robot.leftServo.setPosition(128/255);
            }
            else if(beaconLeftIn){
                robot.leftServo.setPosition(0);
            }
            else
            {
                robot.leftServo.setPosition(1);
            }

            if(!beaconRightIn && beaconRightOut==0)
            {
                robot.rightServo.setPosition(128/255);
            }
            else if(beaconRightIn)
            {
                robot.rightServo.setPosition(0);
            }
            else
            {
                robot.rightServo.setPosition(1);
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

            /*if(shoot){
                shooterTarget = shooterTarget + ticksPerRotation;
                robot.shooterMotor.setTargetPosition(shooterTarget);
                robot.shooterMotor.setPower(1);
                Thread.sleep(400);
            }
            if(robot.shooterMotor.getCurrentPosition() >= shooterTarget && robot.shooterMotor.getCurrentPosition() <= shooterTarget+50){
                robot.shooterMotor.setPower(0);
            }*/

            robot.shooterMotor.setPower(1);

            /*if(revShoot)
            {
                robot.shooterMotor.setPower(-1);
            }
            if(!shoot&&!revShoot){
                robot.shooterMotor.setPower(0);
            }*/

            //robot.shooterMotor.setPower(testShoot);

            /*if(otherShoot)
            {
                shooterTarget = shooterTarget + ticksPerRotation;
                robot.shooterMotor.getCurrentPosition();
                robot.shooterMotor.setTargetPosition(shooterTarget);
                robot.shooterMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.shooterMotor.setPower(1);
            }
            else
            {
                robot.shooterMotor.setPower(0);
            }*/

            if(screwUp)
            {
                robot.elevatorMotor.setPower(1);
            }
            else if(screwDown)
            {
                robot.elevatorMotor.setPower(-1);
            }
            else
            {
                robot.elevatorMotor.setPower(0);
            }

            if(toggleFlapsIn)
            {
                flapsIn = !flapsIn;
                Thread.sleep(200);
            }

            if(toggleFlapsOut)
            {
                flapsOut = !flapsOut;
                Thread.sleep(200);
            }
            
            if(flapsIn)
            {
                robot.intakeMotor.setPower(1);
            }
            else if(flapsOut)
            {
                robot.intakeMotor.setPower(-1);
            }
            else
            {
                robot.intakeMotor.setPower(0);
            }
            
            Telemetry.Item addData = telemetry.addData("Say", left);
                    if(toggleDriveOrient)
                    {

                        reverseMode = !reverseMode;
                        Thread.sleep(200);
                    }
                    if(gamepad1.a)  //lower power for more precise movement
                    {

                        preciseMode = !preciseMode;
                        Thread.sleep(200);
                    }

            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
            //telemetry.addData("encoder!", robot.encoderTest.getCurrentPosition());
            telemetry.update();

            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            robot.waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
}
