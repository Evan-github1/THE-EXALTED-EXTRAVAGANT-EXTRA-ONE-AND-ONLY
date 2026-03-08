package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;

@TeleOp
@Disabled
public class ThesaurusDotTwoPlayer extends Movable implements LimelightTags{
    private static DcMotor intakeMotor, launcherMotor1, launcherMotor2;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static TripleSwitchedServo fires;
    private static Servo fork;
    private static DoubleSwitchedServo forks;
    private static boolean loading;
    private static double motorPower;
    private static boolean shooting;
    private static Limelight3A limelight;
    private static Thread orientRobot;
    private static boolean tracking;
    private static double pastError, currentError;
    private static int iterations;
    private static double motorPowerFar, motorPowerClose;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotor.class,"INT");
        lt1 = hardwareMap.get(Servo.class,"LT1");
        lt2 = hardwareMap.get(Servo.class,"LT2");
        fire = hardwareMap.get(Servo.class, "FIRE");
        fires = new TripleSwitchedServo(fire,.62,.56,.37);
        launcherMotor1 = hardwareMap.get(DcMotor.class,"LAU1");
        launcherMotor2 = hardwareMap.get(DcMotor.class,"LAU2");
        fork = hardwareMap.get(Servo.class,"FORK");
        forks = new DoubleSwitchedServo(fork,.05,.68);
        loading = false;
        motorPower = .54;
        shooting = false;
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        motorPowerClose = .54;
        motorPowerFar = .95;

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pastError = 0;
        currentError = 0;
        iterations = 0;

        enableEncoders();

        waitForStart();

        fires.primaryPos();
        forks.primaryPos();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

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
            }else if(gamepad1.share){
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
                currentError = 0;
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
                if(motorPower == motorPowerFar) {//If far
                    //Set to close pos
                    new Thread(() -> {
                        lt1.setPosition(.25);
                        lt2.setPosition(.25);
                        forks.setPrimaryPos(.05);
                        forks.setSecondaryPos(.68);
                        motorPower = motorPowerClose;
                        try {
                            Thread.sleep(800);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        forks.primaryPos();
                    }).start();
                }else{
                    //Set to far pos
                    new Thread(() -> {
                        lt1.setPosition(.45);
                        lt2.setPosition(.45);
                        forks.setPrimaryPos(.2);
                        forks.setSecondaryPos(.73);
                        motorPower = motorPowerFar;
                        try {
                            Thread.sleep(700);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        forks.primaryPos();
                    }).start();
                }
                setTime();
            }else if(gamepad1.left_trigger > 0.5 && delay(500)){
                if(fire.getPosition() != fires.getSecondaryPos()){
                    new Thread(() -> {
                        fires.primaryPos();
                        forks.secondaryPos();
                        try {
                            Thread.sleep(200);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        fires.secondaryPos();
                        try {
                            Thread.sleep(200);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        forks.primaryPos();
                    }).start();
                }else{
                    new Thread(()->{
                        fires.tertiaryPos();
                        try {
                            Thread.sleep(400);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        fires.primaryPos();
                    }).start();
                }
                setTime();
            }else if(gamepad1.right_trigger > 0.5 && delay(500)){
                if(fire.getPosition() == fires.getSecondaryPos()){
                    new Thread(()->{
                        fires.tertiaryPos();
                        try {
                            Thread.sleep(400);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        fires.primaryPos();
                    }).start();
                }else {
                    loading = true;
                    new Thread(()->{
                        intakeMotor.setPower(1);
                        try {
                            Thread.sleep(150);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        intakeMotor.setPower(0);
                        fires.secondaryPos();
                        loading = false;
                    }).start();
                }
                setTime();
            }else if(!loading){
                intakeMotor.setPower(0);
            }

            if (shooting) {
                launcherMotor1.setPower(motorPower);
                launcherMotor2.setPower(motorPower);
            }else{
                launcherMotor1.setPower(0);
                launcherMotor2.setPower(0);
            }
            telemetry.addData("LT1", lt1.getPosition());
            telemetry.addData("LT2",lt2.getPosition());
            telemetry.addData("Motor power",motorPower);
            telemetry.addData("Motor is running at power",launcherMotor1.getPower());
            telemetry.update();
        }
    }

    private double LeBotsEyes(double pastError, boolean adjustMotor){
        double desiredX;
        telemetry.addData("D",true);
        if(detectTagSelective(limelight,telemetry) != null){
            LLResultTypes.FiducialResult yes = detectTagSelective(limelight,telemetry);
            if(motorPower == motorPowerFar){
                if(yes.getFiducialId() == 20){
                    desiredX = -.25;
                }else{
                    desiredX = -5;
                    telemetry.addData("Desired x changin",true);
                }
            }else{
                if(yes.getFiducialId() == 20){
                    desiredX = -2.5;
                }else{
                    desiredX = -2.5;
                }
            }
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