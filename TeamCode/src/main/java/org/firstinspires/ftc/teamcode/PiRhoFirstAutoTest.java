package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;


/**
 * Created by kevin on 11/8/2016.
 */
//@Autonomous(name = "Work plz", group = "Concept")
public class PiRhoFirstAutoTest extends PiRhoAutoMasterTest {



    public void my_init()
    {
        /*shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
        super.my_init();
        telemetry.addData("Place:", "After Init");
    }
    @Override
    public void runOpMode() throws InterruptedException

    {
        //telemetry.addData("Place:", "Before Init");

        waitForStart();
        telemetry.addData("Place:", "After Wait");
        resetEncoder();
        telemetry.addData("Place:", "After Reset");
        super.shootBall();
        telemetry.addData("Place:", "After shoot");
        super.delay(500);
        telemetry.addData("Place:", "After Delay");
        //super.moveElevator(4);
        //super.shootBall();
        //super.moveBackTo(1);
    }



}


