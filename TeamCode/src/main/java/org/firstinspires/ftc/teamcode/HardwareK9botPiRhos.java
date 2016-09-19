package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.util.Range;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left motor"
 * Motor channel:  Right drive motor:        "right motor"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
@TeleOp(name="Template: Test Crap", group="Linear Opmode")
public abstract class HardwareK9botPiRhos extends OpMode{
    /* Public OpMode members. */
    public DcMotor  leftMotor   = null;
    public DcMotor  rightMotor  = null;
    //public Servo    arm         = null;
    //public Servo    claw        = null;

    public final static double ARM_HOME = 0.2;
    public final static double CLAW_HOME = 0.2;
    public final static double ARM_MIN_RANGE  = 0.20;
    public final static double ARM_MAX_RANGE  = 0.90;
    public final static double CLAW_MIN_RANGE  = 0.20;
    public final static double CLAW_MAX_RANGE  = 0.7;

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareK9botPiRhos() {
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("left motor");
        rightMotor  = hwMap.dcMotor.get("right motor");
        leftMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        /*
        arm = hwMap.servo.get("arm");
        claw = hwMap.servo.get("claw");
        arm.setPosition(ARM_HOME);
        claw.setPosition(CLAW_HOME);*/
    }

    @Override
    public void loop() {

		/*
		 * Gamepad 1 controls the drive motors via the left and right stick
		 *
		 * Gamepad 2 controls the arm via the
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: right_stick_y ranges from -1 to 1, where -1 is full left
        // and 1 is full right
        float throttleL = gamepad1.left_stick_y;
        float throttleR = gamepad1.right_stick_y;


        // clip the right/left values so that the values never exceed +/- 1
        throttleR = Range.clip(throttleR, -1, 1);
        throttleL = Range.clip(throttleL, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.
        throttleL = (float)scaleInput(throttleL);
        throttleR =  (float)scaleInput(throttleR);

        leftMotor.setPower(throttleL);
        rightMotor.setPower(throttleR);

    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */

    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);
        if (index < 0) {
            index = -index;
        } else if (index > 16) {
            index = 16;
        }

        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        return dScale;
    }

    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}
