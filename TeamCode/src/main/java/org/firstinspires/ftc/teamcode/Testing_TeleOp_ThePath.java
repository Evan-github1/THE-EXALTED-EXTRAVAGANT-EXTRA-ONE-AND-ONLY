package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.AutoConfig.BLUE_CLOSE_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.BLUE_FAR_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.BLUE_PARK;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_CLOSE_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_FAR_SCORE;
import static org.firstinspires.ftc.teamcode.AutoConfig.RED_PARK;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.createFollower;
import static org.firstinspires.ftc.teamcode.PedroPathing.Constants.robotLength;

import com.pedropathing.control.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@TeleOp
public class Testing_TeleOp_ThePath extends Movable implements LimelightTags {
    private static ServoImplEx indR;
    private static ServoImplEx indL; //front indicator lights
    private static DoubleSwitchedServo fires;
    private static boolean loading;
    private static boolean shooting;
    private static boolean isAimed, following;
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
    private static Pose openGate, collect;
    private static Path goToScore,classifierClear,collection;
    private static byte varModifying;

    private double raisePosition;

    private double openGateX, openGateY, openGateHeading, collectX, collectY, collectHeading;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        following = false;
        raisePosition = 0;
        indR = hardwareMap.get(ServoImplEx.class, "INDR");
        indL = hardwareMap.get(ServoImplEx.class, "INDL");
        indR.setPwmRange(new PwmControl.PwmRange(500, 2500));
        indL.setPwmRange(new PwmControl.PwmRange(500, 2500));
        loading = false;
        shooting = false;
        isAimed = false;
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
        limelight.pipelineSwitch(2);
        follower = createFollower(hardwareMap);
        follower.setStartingPose(AutoConfig.RED_FAR_START);
        AutoConfig.lastAutoEndPose = null;
        varModifying = 0;

        FLW.setDirection(DcMotor.Direction.FORWARD);
        BLW.setDirection(DcMotor.Direction.FORWARD);
        FRW.setDirection(DcMotor.Direction.REVERSE);
        BRW.setDirection(DcMotor.Direction.REVERSE);
        pastError = 0;
        iterations = 0;


        enableEncoders();

        waitForStart();
        follower.startTeleopDrive(true);

        fires.primaryPos();

        while (opModeIsActive()) {

            if(!follower.isBusy()&&!follower.isTeleopDrive() && !following){
                follower.startTeleopDrive(true);
            }

            if(gamepad1.optionsWasPressed()){
                boolean isLocalized = relocalize();
                if(isLocalized){
                    gamepad2.rumbleBlips(3);
                }
            }
            if(gamepad1.aWasPressed()){
                varModifying++;
                varModifying%=6;
            }
            if(gamepad1.guideWasPressed()){
                if(!follower.isBusy()) {
                    collect = new Pose(collectX, collectY, collectHeading);
                    openGate = new Pose(openGateX, openGateY, openGateHeading);
                    PathChain path = follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(),collect))
                            .setLinearHeadingInterpolation(follower.getHeading(),collect.getHeading())
                            .addPath(new BezierLine(collect,openGate))
                            .setLinearHeadingInterpolation(collectHeading,openGateHeading,0.7)
                            .build();
                    follower.followPath(path);
                }else{
                    follower.startTeleOpDrive(true);
                }
            }

            if(gamepad1.dpadDownWasPressed()){
                switch(varModifying){
                    case 0:
                        collectX-=.25;
                        break;
                    case 1:
                        collectY-=.25;
                        break;
                    case 2:
                        collectHeading-=Math.toRadians(1);
                        break;
                    case 3:
                        openGateX-=.25;
                        break;
                    case 4:
                        openGateY-=.25;
                        break;
                    case 5:
                        openGateHeading-=Math.toRadians(1);
                        break;
                }
            }

            if(gamepad1.dpadUpWasPressed()){
                switch(varModifying){
                    case 0:
                        collectX+=.25;
                        break;
                    case 1:
                        collectY+=.25;
                        break;
                    case 2:
                        collectHeading+=Math.toRadians(1);
                        break;
                    case 3:
                        openGateX+=.25;
                        break;
                    case 4:
                        openGateY+=.25;
                        break;
                    case 5:
                        openGateHeading+=Math.toRadians(1);
                        break;
                }
            }

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
