package org.firstinspires.ftc.teamcode;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.HeadingPID;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;

import static org.firstinspires.ftc.teamcode.AutoConfig.BLUE_PARK;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_PARK;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotWidth;
import org.firstinspires.ftc.teamcode.AutoConfig.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


@TeleOp
public class AAAAThesaurusDotJustin extends Movable implements LimelightTags {

    private static DcMotor intakeMotor, transferMotor;
    private static DcMotorEx launcherMotor1, launcherMotor2;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static ServoImplEx indR;
    private static ServoImplEx indL; //front indicator lights
    private static DoubleSwitchedServo fires;
    private static boolean loading;
    private static boolean shooting;
    private static boolean isAimed;
    private static Limelight3A limelight;
    private static Thread orientRobot;
    private static boolean tracking;
    private static double pastError;
    private static int iterations;
    private static double motorPowerFar, motorPowerClose;
    private static double rpm, rpm2, tps, tps2, targetRPM, P, FClose, FFar, currentTargetRPM;
    private static PIDFCoefficients pidfCoefficients;
    private static PIDFController aimPID;
    private static Follower follower;
    private static Pose holdPose;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotor.class,"INT");
        transferMotor = hardwareMap.get(DcMotor.class, "INT2");
        lt1 = hardwareMap.get(Servo.class,"LT1");
        lt2 = hardwareMap.get(Servo.class,"LT2");
        fire = hardwareMap.get(Servo.class, "FIRE");
        fires = new DoubleSwitchedServo(fire,.8,.4);
        indR = hardwareMap.get(ServoImplEx.class, "INDR");
        indL = hardwareMap.get(ServoImplEx.class, "INDL");
        indR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        indL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        launcherMotor1 = hardwareMap.get(DcMotorEx.class,"LAU1");
        launcherMotor2 = hardwareMap.get(DcMotorEx.class,"LAU2");
        loading = false;
        shooting = false;
        isAimed = false;
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
        limelight.pipelineSwitch(2);
        motorPowerClose = 2500;
        motorPowerFar = 3750; //from 4500
        targetRPM = motorPowerFar;
        P = 50;
        FClose = 16.8;
        FFar = 15.8;
        pidfCoefficients = new PIDFCoefficients(P,0,0,FFar);
        aimPID = new PIDFController(new com.pedropathing.control.PIDFCoefficients(1.2,0,0.05,0.025));
        follower = createFollower(hardwareMap);
        follower.setStartingPose(
            AutoConfig.lastAutoEndPose != null
                ? AutoConfig.lastAutoEndPose
                : AutoConfig.RED_FAR_START
        );
        AutoConfig.lastAutoEndPose = null;

        FLW.setDirection(DcMotor.Direction.FORWARD);
        BLW.setDirection(DcMotor.Direction.FORWARD);
        FRW.setDirection(DcMotor.Direction.REVERSE);
        BRW.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transferMotor.setDirection(DcMotor.Direction.FORWARD);
        transferMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorSimple.Direction.FORWARD);
        pastError = 0;
        iterations = 0;
        launcherMotor1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);
        launcherMotor2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,pidfCoefficients);


        enableEncoders();

        waitForStart();
        follower.startTeleopDrive(true);

        fires.primaryPos();

        while (opModeIsActive()) {
            follower.update();
            telemetry.addData("Status", "Running");

            tps = launcherMotor1.getVelocity();
            tps2 = launcherMotor2.getVelocity();

            rpm = tps * 60 / 28;
            rpm2 = tps2 * 60 / 28;

            telemetry.addData("RPM1",rpm);
            telemetry.addData("RPM2", rpm2);
            telemetry.addData("Target RPM",targetRPM);
            telemetry.addData("Position", follower.getPose().getX() + " " + follower.getPose().getY());
            telemetry.addData("Angle",Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Power",launcherMotor1.getPower());

            if(!follower.isBusy()&&!follower.isTeleopDrive()){
                follower.startTeleopDrive(true);
            }

            if(gamepad1.dpad_up) {
                moveWheels(0,0.25f);
                isAimed = false;
            }else if(gamepad1.dpad_down){
                moveWheels(0,-0.25f);
                isAimed = false;
            }else if(gamepad1.dpad_left){
                moveWheels(0.25f,0);
                isAimed = false;
            }else if(gamepad1.dpad_right){
                moveWheels(-0.25f,0);
                isAimed = false;
            }else {
                isAimed = robotDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger>0.5);
            }


            if(isAimed){
                indL.setPosition(.8);
                indR.setPosition(.8);
            } else if(AutoConfig.isRed){
                indL.setPosition(0.31);
                indR.setPosition(0.31);
            } else{
                indL.setPosition(0.611);
                indR.setPosition(0.611);
            }

            if (gamepad1.yWasPressed()) {
                if(intakeMotor.getPower() == 0){
                    intakeMotor.setPower(1);
                    loading = true;
                }else {
                    intakeMotor.setPower(0);
                    loading = false;
                }
                setTime();
            }
            if (gamepad1.aWasPressed()) {
                if(intakeMotor.getPower() == 0){
                    intakeMotor.setPower(-.5);
                    loading = true;
                }else {
                    intakeMotor.setPower(0);
                }
                setTime();
            }
            if(gamepad2.dpadUpWasPressed()){
                AutoConfig.isRed = !AutoConfig.isRed;
            }
            if(gamepad2.dpadDownWasPressed()){
                relocalize();
            }
            if(gamepad2.guideWasPressed()){
                if(AutoConfig.isRed){
                    follower.setPose(new Pose(8.25,robotLength()/2,-(Math.PI/2)));
                } else{
                    follower.setPose(new Pose(144-8.25,robotLength()/2,-(Math.PI/2)));
                }
            }
            if(gamepad1.guideWasPressed()){
                if(AutoConfig.isRed){
                    if(!follower.isBusy()) {
                        follower.followPath(follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(),RED_PARK))
                                .setLinearHeadingInterpolation(follower.getHeading(),RED_PARK.getHeading())
                                .setConstraints(new PathConstraints(0.995,300))
                                .build());
                    }
                    else{
                        follower.startTeleopDrive(true);
                    }
                } else{
                    if(!follower.isBusy()) {
                        follower.followPath(follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(),BLUE_PARK))
                                .setLinearHeadingInterpolation(follower.getHeading(),BLUE_PARK.getHeading())
                                .setConstraints(new PathConstraints(0.995,300))
                                .build());
                    }
                    else{
                        follower.startTeleopDrive(true);
                    }
                }
            }

            if(gamepad1.xWasPressed()){
                shooting = !shooting;
                setTime();
            }
            if(gamepad1.bWasPressed()){
                if(targetRPM == motorPowerFar) {//If far
                    //Set to close pos
                    new Thread(() -> {
                        lt1.setPosition(.25);//from .25
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
                        lt1.setPosition(.37);
                        lt2.setPosition(.37);
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
            }
            if(gamepad1.right_trigger > 0.5 /*&& delay(1600)*/ && !fires.isSecondaryPos()){
                transferMotor.setPower(1);
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

            }
            if(gamepad1.right_trigger <= 0.5 && fires.isSecondaryPos()){
                transferMotor.setPower(0);
                fires.primaryPos();
            }
            if(!loading){
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
    private boolean relocalize() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            Pose2D ftcPose = new Pose2D(
                DistanceUnit.METER,
                botpose.getPosition().x,
                botpose.getPosition().y,
                AngleUnit.DEGREES,
                botpose.getOrientation().getYaw(AngleUnit.DEGREES)
            );
            Pose pedroPose = PoseConverter.pose2DToPose(ftcPose, InvertedFTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            follower.setPose(pedroPose);
            telemetry.addData("Reloc Raw", "x=%.2f y=%.2f yaw=%.1f",
                botpose.getPosition().x, botpose.getPosition().y,
                botpose.getOrientation().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Reloc Pedro", "x=%.2f y=%.2f h=%.1f",
                pedroPose.getX(), pedroPose.getY(),
                Math.toDegrees(pedroPose.getHeading()));
            return true;
        }
        telemetry.addData("Reloc", "No valid result");
        return false;
    }

    //not used atm
    private double LeBotsEyes(double pastError, boolean adjustMotor){
        double desiredX;
        telemetry.addData("D", true);
        if(detectTagSelective(limelight,telemetry) != null){
            LLResultTypes.FiducialResult yes = detectTagSelective(limelight,telemetry);
            desiredX = 0;
            double smoothCoeff = 0.25;
            double k = 1.0/20;
            telemetry.addData("Yes is not null",true);
            double tx = yes.getTargetXDegrees();
            double currentError = desiredX - tx;
            double smoothedError = smoothCoeff*currentError + (1-smoothCoeff)*pastError;
            smoothedError = smoothedError*k;
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

    //drives robot with given translational and rotational inputs(translational stick, rotational stick) and takes a boolean for if it is auto aiming or not (if auto aiming, it will ignore rotational inputs and instead calculate the rotation needed to face the target and rotate the robot accordingly)
    //uses Pedro drive algorithm
    //uses alliance based on auto setting it
    private boolean robotDrive(double transX, double transY, double turnX, boolean autoAim) {
        double forward = transY;
        double strafe = transX;
        double turn;
        double error = 100000;
        if(autoAim){
            double goalX;
            double goalY = 144;
            if(AutoConfig.isRed){
                goalX=144;
            } else{
                goalX=0;
            }
            //calculate angle from robot to target, add pi to get angle robot needs to face to aim at target (since launcher is on back of robot)
            double targetAngle = Math.atan2(goalY - follower.getPose().getY(), goalX - follower.getPose().getX()) + Math.PI;

            //calculate error between target angle and current angle
            error = targetAngle - follower.getPose().getHeading();

            //convert error to range [-pi, pi] so that robot turns the shortest distance to target
            while(error>Math.PI){error-=2*Math.PI;}
            while(error<-Math.PI){error+=2*Math.PI;}

            //run PID
            aimPID.updateError(error);
            turn = aimPID.run();

        } else{
            turn = -turnX;
        }
        //drive wheels
        follower.setTeleOpDrive(forward,strafe,turn);

        //isAimed check basically
        //checks for is within 0.05 degrees of target angle and not rotating too fast
        return autoAim && Math.abs(error) < Math.toRadians(0.5) && Math.abs(follower.getAngularVelocity()) < Math.toRadians(0.5);
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
