package org.firstinspires.ftc.teamcode;
import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotWidth;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;


@TeleOp
public class AAAAThesaurusDotJustin extends Movable implements LimelightTags {

    private static DcMotor intakeMotor;
    private static DcMotorEx launcherMotor1, launcherMotor2;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static Servo indR;
    private static Servo indL; //front indicator lights
    private static DoubleSwitchedServo fires;
    private static boolean loading;
    private static boolean shooting;
    private static Limelight3A limelight;
    private static Thread orientRobot;
    private static boolean tracking;
    private static double pastError;
    private static int iterations;
    private static double motorPowerFar, motorPowerClose;
    private static double rpm, tps, targetRPM, P, FClose, FFar, currentTargetRPM;
    private static PIDFCoefficients pidfCoefficients;
    private static Follower follower;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotor.class,"INT");
        lt1 = hardwareMap.get(Servo.class,"LT1");
        lt2 = hardwareMap.get(Servo.class,"LT2");
        fire = hardwareMap.get(Servo.class, "FIRE");
        fires = new DoubleSwitchedServo(fire,.8,.4);
        indR = hardwareMap.get(Servo.class, "INDR");
        indL = hardwareMap.get(Servo.class, "INDL");
        launcherMotor1 = hardwareMap.get(DcMotorEx.class,"LAU1");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class,"LAU2");
        loading = false;
        shooting = false;
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        motorPowerClose = 2500;
        motorPowerFar = 4500;
        targetRPM = motorPowerFar;
        P = 30;
        FClose = 16.8;
        FFar = 15;
        pidfCoefficients = new PIDFCoefficients(P,0,0,FFar);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(new Pose(96, 0 + robotLength()/2, Math.toRadians(-90)));

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        pastError = 0;
        iterations = 0;
        launcherMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        launcherMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);


        enableEncoders();

        waitForStart();



        fires.primaryPos();

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Status", "Running");

            tps = launcherMotor2.getVelocity();

            rpm = tps * 60 / 28;
            telemetry.addData("RPM",rpm);
            telemetry.addData("Target RPM",targetRPM);
            telemetry.addData("Position", follower.getPose().getX() + " " + follower.getPose().getY());
            telemetry.addData("Angle",Math.toDegrees(follower.getPose().getHeading()));

            if(Math.abs(gamepad1.right_stick_x) > 0.1){
                FLW.setPower(gamepad1.right_stick_x);
                FRW.setPower(-gamepad1.right_stick_x);
                BLW.setPower(gamepad1.right_stick_x);
                BRW.setPower(-gamepad1.right_stick_x);
            }else if(gamepad1.dpad_up) {
                moveWheels(0,0.5f);
            }else if(gamepad1.dpad_down){
                moveWheels(0,-0.5f);
            }else if(gamepad1.dpad_left){
                moveWheels(0.5f,0);
            }else if(gamepad1.dpad_right){
                moveWheels(-0.5f,0);
            }else if(gamepad1.right_stick_button){
                if(iterations == 0 ){
                    pastError = 0;
                    LeBotsEyes(pastError,true);
                }else{
                    pastError = LeBotsEyes(pastError,false);
                    LeBotsEyes(pastError,true);

                }
                iterations++;

            }else{
                moveWheels(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
                pastError = 0;
                iterations = 0;
            }
            if (gamepad1.y && delay()) {
                if(intakeMotor.getPower() == 0){
                    intakeMotor.setPower(1);
                    loading = true;
                }else {
                    intakeMotor.setPower(0);
                    loading = false;
                }
                setTime();
            }else if (gamepad1.a && delay()) {
                if(intakeMotor.getPower() == 0){
                    intakeMotor.setPower(-.5);
                    loading = true;
                }else {
                    intakeMotor.setPower(0);
                }
                setTime();
            }else if(gamepad1.x && delay()){
                shooting = !shooting;
                setTime();
            }else if(gamepad1.b && delay()){
                if(targetRPM == motorPowerFar) {//If far
                    //Set to close pos
                    new Thread(() -> {
                        lt1.setPosition(.25);
                        lt2.setPosition(.25);
                        targetRPM = motorPowerClose;
                        pidfCoefficients = new PIDFCoefficients(P,0,0,FClose);
                        launcherMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
                        launcherMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
                        try {
                            Thread.sleep(800);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }).start();
                }else{
                    //Set to far pos
                    new Thread(() -> {
                        lt1.setPosition(.45);
                        lt2.setPosition(.45);
                        targetRPM = motorPowerFar;
                        pidfCoefficients = new PIDFCoefficients(P,0,0,FFar);
                        launcherMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
                        launcherMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidfCoefficients);
                        try {
                            Thread.sleep(700);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                    }).start();
                }
                setTime();
            }else if(gamepad1.right_trigger > 0.5 /*&& delay(1600)*/ && !fires.isSecondaryPos()){
                fires.secondaryPos();
                /*new Thread(() -> {
                    fires.secondaryPos();
                    try {
                        Thread.sleep(1500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    fires.primaryPos();
                }).start();*/

            }else if(gamepad1.right_trigger <= 0.5 && fires.isSecondaryPos()){
                fires.primaryPos();
            }else if(!loading){
                intakeMotor.setPower(0);
            }

            if (shooting) {
                launcherMotor1.setVelocity(targetRPM / 60 * 28);
                launcherMotor2.setVelocity(targetRPM / 60 * 28);
            }else{
                launcherMotor1.setVelocity(0);
                launcherMotor2.setVelocity(0);
            }

            telemetry.addData("FLW encoder",FLW.getCurrentPosition());
            telemetry.addData("FRW encoder",FRW.getCurrentPosition());
            telemetry.addData("BLW encoder",BLW.getCurrentPosition());
            telemetry.addData("BRW encoder",BRW.getCurrentPosition());
            telemetry.update();
        }
    }

    private double LeBotsEyes(double pastError, boolean adjustMotor){
        double desiredX;
        telemetry.addData("D", true);
        if(detectTagSelective(limelight,telemetry) != null){
            LLResultTypes.FiducialResult yes = detectTagSelective(limelight,telemetry);
            desiredX = 0;
            double smoothCoeff = 0.25;
            telemetry.addData("Yes is not null",true);
            double tx = yes.getTargetXDegrees();
            double currentError = desiredX - tx;
            double smoothedError = smoothCoeff*currentError + (1-smoothCoeff)*pastError;
            smoothedError = smoothedError/25;
            telemetry.addData(""+currentError,smoothedError);
            if(adjustMotor) {
                FLW.setPower(-smoothedError);
                FRW.setPower(smoothedError);
                BRW.setPower(smoothedError);
                BLW.setPower(-smoothedError);
                return 0.0;
            }
            return smoothedError;

        }else{
            return 0;
        }

    }


    @Override
    public void tag20() {

    }

    @Override
    public void tag21() {

    }

    @Override
    public void tag22() {

    }

    @Override
    public void tag23() {

    }

    @Override
    public void tag24() {

    }

    @Override
    public void green() {

    }

    @Override
    public void purple() {

    }
}
