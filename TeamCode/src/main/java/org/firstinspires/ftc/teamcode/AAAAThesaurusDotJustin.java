package org.firstinspires.ftc.teamcode;
import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
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

import static org.firstinspires.ftc.teamcode.AutoConfig.BLUE_CLOSE_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.BLUE_FAR_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.BLUE_PARK;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_CLOSE_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_FAR_SCORE;
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
    private static boolean autoLocalize;
    private boolean useOldAiming;
    private static double pastError;
    private static int iterations;
    private static double motorPowerFar, motorPowerClose;
    private static double DEADZONE = 0.05;
    private static double rpm, rpm2, tps, tps2, targetRPM, P, FClose, FFar, currentTargetRPM;
    private static PIDFCoefficients pidfCoefficients;
    private static PIDFController aimPID;
    private static Follower follower;
    private static Pose holdPose;

    private double raisePosition;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        raisePosition = 0;

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
        useOldAiming = false;
        autoLocalize = true;
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
        limelight.pipelineSwitch(2);
        motorPowerClose = 2500;
        motorPowerFar = 3800; //from 4500
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
        follower.startTeleopDrive(false);

        fires.primaryPos();

        while (opModeIsActive()) {
            /*final double MAX_BORDER = 141.5;

            double farDist, closeDist;
            double distFromGoal;
            if (AutoConfig.isRed) {
                distFromGoal = Math.hypot(follower.getPose().getX() - MAX_BORDER, follower.getPose().getY() - MAX_BORDER);
                farDist = Math.hypot(RED_FAR_SCORE.getPose().getX() - MAX_BORDER, RED_FAR_SCORE.getPose().getY() - MAX_BORDER);;
                closeDist = Math.hypot(RED_CLOSE_SCORE.getPose().getX() - MAX_BORDER, RED_CLOSE_SCORE.getPose().getY() - MAX_BORDER);
            } else {
                distFromGoal = Math.hypot(follower.getPose().getX() - 0, follower.getPose().getY() - MAX_BORDER);
                farDist = Math.hypot(BLUE_FAR_SCORE.getPose().getX() - 0, BLUE_FAR_SCORE.getPose().getY() - MAX_BORDER);
                closeDist = Math.hypot(BLUE_CLOSE_SCORE.getPose().getX() - 0, BLUE_CLOSE_SCORE.getPose().getY() - MAX_BORDER);
            }
            raisePosition = 0.25 +
                    Math.max(0, Math.min(1,
                            (distFromGoal - closeDist) / (farDist - closeDist)
                    )) * (0.37 - 0.25);
            lt1.setPosition(raisePosition);
            lt2.setPosition(raisePosition);*/


            follower.update();
            telemetry.addData("Status", "Running");

            tps = launcherMotor1.getVelocity();
            tps2 = launcherMotor2.getVelocity();

            rpm = tps * 60 / 28;
            rpm2 = tps2 * 60 / 28;

            telemetry.addData("Automatically Localizing:", autoLocalize);
            telemetry.addData("RPM1",rpm);
            telemetry.addData("RPM2", rpm2);
            telemetry.addData("Target RPM",targetRPM);
            telemetry.addData("Position", follower.getPose().getX() + " " + follower.getPose().getY());
            telemetry.addData("Angle",Math.toDegrees(follower.getPose().getHeading()));
            telemetry.addData("Power",launcherMotor1.getPower());
            //telemetry.addData("GP2DPDown", gamepad2.dpad_down);

            if(!follower.isBusy()&&!follower.isTeleopDrive()){
                follower.startTeleopDrive(false);
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
                if(!useOldAiming) {
                    isAimed = robotDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_trigger > 0.5);
                } else if(gamepad1.left_trigger>0.5){
                    try {
                        if (iterations == 0) {
                            pastError = 0;
                            LeBotsEyes(pastError, true);
                        } else {
                            pastError = LeBotsEyes(pastError, false);
                            LeBotsEyes(pastError, true);
                        }
                        iterations++;
                    }catch(Exception ignored){}
                } else{
                    robotDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, false);
                }
            }
            if(!fires.isSecondaryPos()) {
                if (gamepad2.right_trigger > 0.5) {
                    transferMotor.setPower(1);
                } else {
                    transferMotor.setPower(0);
                }
            }
            if(gamepad1.left_trigger > .5 && !useOldAiming){
                final double MAX_BORDER = 140;

                double farDist, closeDist;
                double distFromGoal;

                if (AutoConfig.isRed) {
                    distFromGoal = Math.hypot(follower.getPose().getX() - 140, follower.getPose().getY() - MAX_BORDER) + 5;
                    farDist = Math.hypot(RED_FAR_SCORE.getPose().getX() - 140, RED_FAR_SCORE.getPose().getY() - MAX_BORDER);
                    closeDist = Math.hypot(RED_CLOSE_SCORE.getPose().getX() - 140, RED_CLOSE_SCORE.getPose().getY() - MAX_BORDER);
                } else {
                    distFromGoal = Math.hypot(follower.getPose().getX() - 7, follower.getPose().getY() - MAX_BORDER);
                    farDist = Math.hypot(BLUE_FAR_SCORE.getPose().getX() - 7, BLUE_FAR_SCORE.getPose().getY() - MAX_BORDER);
                    closeDist = Math.hypot(BLUE_CLOSE_SCORE.getPose().getX() - 7, BLUE_CLOSE_SCORE.getPose().getY() - MAX_BORDER);
                }

                // No clamping
                double t = (distFromGoal - closeDist) / (farDist - closeDist);

                // Servo scaling
                raisePosition = 0.25 + t * (0.37 - 0.25);
                lt1.setPosition(raisePosition);
                lt2.setPosition(raisePosition);

                // RPM scaling
                targetRPM = (motorPowerClose + t * (motorPowerFar - motorPowerClose))-40;
            }

            if(useOldAiming){
                indL.setPosition(0.2);
                indR.setPosition(0.2);
            }else if(isAimed){
                indL.setPosition(.8);
                indR.setPosition(.8);
            } else if(AutoConfig.isRed){
                indL.setPosition(0.31);
                indR.setPosition(0.31);
            } else{
                indL.setPosition(0.611);
                indR.setPosition(0.611);
            }
            if(gamepad2.yWasPressed()){
                autoLocalize = !autoLocalize;
            }

            if (gamepad1.yWasPressed()) {
                if(intakeMotor.getPower() == 0){
                    intakeMotor.setPower(1);
                    loading = true;
                }else {
                    intakeMotor.setPower(0);
                    loading = false;
                }

            }
            if (gamepad1.aWasPressed()) {
                if(intakeMotor.getPower() == 0){
                    intakeMotor.setPower(-.5);
                    transferMotor.setPower(-1);
                    loading = true;
                }else {
                    intakeMotor.setPower(0);
                    transferMotor.setPower(0);
                }

            }
            if(gamepad2.dpadUpWasPressed()){
                AutoConfig.isRed = !AutoConfig.isRed;
            }
            if(gamepad2.dpadRightWasPressed()){
                useOldAiming = !useOldAiming;
                int pipeline = (useOldAiming) ? 0 : 2; //sets pipeline to correct one based on current mode(0 = aiming, 2 = localization)
                limelight.pipelineSwitch(pipeline);
            }
            if(delay(2000)&&autoLocalize){
                if(relocalize(false)){
                    //gamepad2.rumbleBlips(1);
                    setTime();
                }
            }
            if(gamepad2.dpadDownWasPressed()){
                boolean isLocalized = relocalize(true);
                if(isLocalized){
                    gamepad2.rumbleBlips(3);
                    gamepad1.rumbleBlips(3);
                }else{
                    gamepad2.rumbleBlips(2);
                }
            }
            if(gamepad2.guideWasPressed()){
                gamepad2.rumbleBlips(3);
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
                                .setConstraints(new PathConstraints(0.995,1000))
                                .build());
                    }
                    else{
                        follower.startTeleopDrive(false);
                    }
                } else{
                    if(!follower.isBusy()) {
                        follower.followPath(follower.pathBuilder()
                                .addPath(new BezierLine(follower.getPose(),BLUE_PARK))
                                .setLinearHeadingInterpolation(follower.getHeading(),BLUE_PARK.getHeading())
                                .setConstraints(new PathConstraints(0.995,1000))
                                .build());
                    }
                    else{
                        follower.startTeleopDrive(false);
                    }
                }
            }

            if(gamepad1.xWasPressed()){
                shooting = !shooting;

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
            }
            if(gamepad1.right_trigger > 0.5 /*&& delay(1600)*/){
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
                launcherMotor1.setVelocity(targetRPM / 60.0 * 28);
                launcherMotor2.setVelocity(targetRPM / 60.0 * 28);
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

    private boolean relocalize(boolean manualOverride) {
        LLResult result = limelight.getLatestResult();
        boolean isStill = (Math.abs(follower.getVelocity().getXComponent())<0.025 && Math.abs(follower.getVelocity().getYComponent())<0.025 && Math.abs(follower.getAngularVelocity()) < 0.025);
        if ((isStill||manualOverride) && result != null && result.isValid() && detectTag(limelight, telemetry) > 0) {
            Pose3D botpose = result.getBotpose();
            Pose2D ftcPose = new Pose2D(
                DistanceUnit.METER,
                botpose.getPosition().x,
                botpose.getPosition().y,
                AngleUnit.DEGREES,
                botpose.getOrientation().getYaw(AngleUnit.DEGREES)
            );
            Pose pedroPose = PoseConverter.pose2DToPose(ftcPose, FTCCoordinates.INSTANCE)
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
        //telemetry.addData("Reloc", "FAILED");
        //telemetry.addData("Reloc result null", result == null);
        if (result != null) {
            //telemetry.addData("Reloc valid", result.isValid());
            //telemetry.addData("Reloc staleness", result.getStaleness());
            //telemetry.addData("Reloc pipeline", result.getPipelineIndex());
        }
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
        if(Math.abs(forward) < DEADZONE){
            forward = 0;
        }
        double strafe = transX;
        if(Math.abs(strafe) < DEADZONE){
            strafe = 0;
        }
        double turn;
        double error = 100000;
        if(autoAim){
            double goalX;
            double goalY = 140;
            if(AutoConfig.isRed){
                goalX=141.5;
            } else{
                goalX=6;//for better aiming?
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
            if(Math.abs(turn) < DEADZONE){
                turn = 0;
            }
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
