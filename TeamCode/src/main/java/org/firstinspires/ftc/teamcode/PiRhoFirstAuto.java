package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



/**
 * Created by kevin on 11/8/2016.
 */
@Autonomous(name = "Autonomous", group = "Concept")
public class PiRhoFirstAuto extends PiRhoAutoMaster {
    public void runOpMode() throws InterruptedException

    {

        shootBall();
        shootBall();
    }

}


