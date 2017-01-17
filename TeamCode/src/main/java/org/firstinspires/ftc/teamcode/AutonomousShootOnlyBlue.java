package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by lukemarshall on 1/6/17.
 */
@Autonomous(name = "Autonomous Shoot Only Blue", group = "Concept")
public class AutonomousShootOnlyBlue extends LinearOpMode {
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
        robot.backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        delay(300);
    }

    //Robot functionalities
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
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        robot.backLeftMotor.setPower(power);
        robot.backRightMotor.setPower(power);
        telemetry.addData("Motor Power", robot.backLeftMotor.getPower());
        telemetry.addData("Motor Variable", power);
    }

    public void moveBackTo(int moveInRotations, double leftPower, double rightPower){
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - moveInRotations);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() - moveInRotations);
        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!((robot.backLeftMotor.getCurrentPosition()<= robot.backLeftMotor.getTargetPosition()+TOLERANCE) &&
                (robot.backRightMotor.getCurrentPosition()<= robot.backRightMotor.getTargetPosition()+TOLERANCE))){

            robot.backLeftMotor.setPower(-leftPower);

            robot.backRightMotor.setPower(-rightPower);

        }
        haltDrive();
    }

    public void moveForwardTo(int moveInRotations, double leftPower, double rightPower){
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + moveInRotations);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + moveInRotations);

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!((robot.backLeftMotor.getCurrentPosition()>= robot.backLeftMotor.getTargetPosition()-TOLERANCE) &&
                (robot.backRightMotor.getCurrentPosition()>= robot.backRightMotor.getTargetPosition()-TOLERANCE))){

            robot.backLeftMotor.setPower(leftPower);

            robot.backRightMotor.setPower(rightPower);
        }
        haltDrive();
    }

    //Turning
    public void correctTurn(double power, double headingGoal){
        if(absHeading() < (headingGoal - threshold)){
            calcTurn(headingGoal-absHeading());
            robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + counts);
            robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() - counts);
            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(!((robot.backLeftMotor.getCurrentPosition()<= robot.backLeftMotor.getTargetPosition()-TOLERANCE) &&
                    (robot.backRightMotor.getCurrentPosition()>=robot.backRightMotor.getTargetPosition()+TOLERANCE))){

                robot.backLeftMotor.setPower(power);

                robot.backRightMotor.setPower(-power);
            }
        }
        else if(absHeading() > headingGoal + threshold){
            calcTurn(absHeading()-headingGoal);
            robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - counts);
            robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + counts);

            robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            while(!((robot.backLeftMotor.getCurrentPosition()>= robot.backLeftMotor.getTargetPosition()+TOLERANCE) &&
                    (robot.backRightMotor.getCurrentPosition()<=robot.backRightMotor.getTargetPosition()-TOLERANCE))){

                robot.backLeftMotor.setPower(-power);

                robot.backRightMotor.setPower(power);
            }
        }
        haltDrive();
    }
    public void turnLeftVariableAndStop(double power, int degrees) {
        calcTurn(degrees);
        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() + counts);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() - counts);

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!((robot.backLeftMotor.getCurrentPosition()>= robot.backLeftMotor.getTargetPosition()-TOLERANCE) &&
                (robot.backRightMotor.getCurrentPosition()<=robot.backRightMotor.getTargetPosition()+TOLERANCE))){

            robot.backLeftMotor.setPower(-power);

            robot.backRightMotor.setPower(power);
        }
        haltDrive();
    }

    public void turnRightVariableAndStop(double power, int degrees) {
        calcTurn(degrees);

        robot.backLeftMotor.setTargetPosition(robot.backLeftMotor.getCurrentPosition() - counts);
        robot.backRightMotor.setTargetPosition(robot.backRightMotor.getCurrentPosition() + counts);

        robot.backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while(!((robot.backLeftMotor.getCurrentPosition()<= robot.backLeftMotor.getTargetPosition()+TOLERANCE) &&
                (robot.backRightMotor.getCurrentPosition()>=robot.backRightMotor.getTargetPosition()-TOLERANCE))){

            robot.backLeftMotor.setPower(-power);

            robot.backRightMotor.setPower(power);
        }
        haltDrive();


    }

    //Color Sensors
    public boolean onWhiteLine(){
        if(robot.lineColorSensor.alpha()>=40){
            return true;
        }
        return false;
    }

    public void stopOnLineFoward(double power) {
        telemetry.addData("loop", "not init");
        telemetry.update();


        while (!(robot.lineColorSensor.alpha() >= 40)) {
            telemetry.addData("loop", "init");
            goFoward(power);
            telemetry.addData("alpha", robot.lineColorSensor.alpha());
            telemetry.update();
        }
    }
    public void stopOnLineBackward(double power){
        telemetry.addData("loop","not init");
        telemetry.update();


        while(!(robot.lineColorSensor.alpha()>=40)){
            telemetry.addData("loop","init");
            goFoward(-power);
            telemetry.addData("alpha",robot.lineColorSensor.alpha());
            telemetry.update();
        }

    }
    public boolean rightBeaconIsRed(){
        if(robot.colorSensor.red()>220){
            return true;
        }
        else{
            return false;
        }

    }
    public void extendBeaconPresser(){
        robot.beaconServo.setPower(1.0);//left
    }
    public void retractBeaconPresser(){
        robot.beaconServo.setPower(0.0);//right
    }
    public void pressBeacon(){
        telemetry.addData("red", robot.colorSensor.red());
        telemetry.update();
        if(rightBeaconIsRed()){
            //move and extend beacon presser
            extendBeaconPresser();


        }
        else{
            //move and extend beacon presser
        }
        delay(2000);
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

        delay(10000);
        //move to shooting position
        moveForwardTo(calcDrive(5),0.2, 0.2);
        turnRightVariableAndStop(0.3, 45);
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


        //Stop All Movement
        //resetEncoders();


    }

}
