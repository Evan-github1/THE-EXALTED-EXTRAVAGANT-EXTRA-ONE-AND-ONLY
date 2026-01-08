//This is the Autonomous for 22335!

package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;

@Autonomous
public class DoomAndDisgust_FAR extends Movable implements LimelightTags {
    private static DcMotor intakeMotor, launcherMotor1, launcherMotor2;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static TripleSwitchedServo fires;
    private static Servo fork;
    private static DoubleSwitchedServo forks;
    private static Limelight3A limelight;
    private static double motorPowerClose,motorPowerFar,pastError;
    private static int iterations;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotor.class,"INT");
        lt1 = hardwareMap.get(Servo.class,"LT1");
        lt2 = hardwareMap.get(Servo.class,"LT2");
        fire = hardwareMap.get(Servo.class, "FIRE");
        fires = new TripleSwitchedServo(fire,.55,.49,.35);
        launcherMotor1 = hardwareMap.get(DcMotor.class,"LAU1");
        launcherMotor2 = hardwareMap.get(DcMotor.class,"LAU2");
        fork = hardwareMap.get(Servo.class,"FORK");
        forks = new DoubleSwitchedServo(fork,0.23,0.75);
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
        limelight.pipelineSwitch(0);
        motorPowerClose = .5185;
        motorPowerFar = .888;

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        enableEncoders();

        waitForStart();
        disablePower();

        FLW.setPower(-.25);
        FRW.setPower(-.25);
        BLW.setPower(-.25);
        BRW.setPower(-.25);
        forks.primaryPos();

        Thread.sleep(300);

        long currentTime = System.currentTimeMillis();

        while(System.currentTimeMillis() - currentTime < 5000) {
            if (iterations == 0) {
                pastError = 0;
                LeBotsEyes(pastError, true);
            } else {
                pastError = LeBotsEyes(pastError, false);
                LeBotsEyes(pastError, true);
            }
            iterations++;
        }
        disablePower();
        lt1.setPosition(.45);
        lt2.setPosition(.45);
        forks.setPrimaryPos(.23);
        forks.setSecondaryPos(.75);
        launcherMotor1.setPower(.925);
        launcherMotor2.setPower(.925);
        Thread.sleep(5000);
        //Shoot first
        fires.tertiaryPos();
        Thread.sleep(400);
        fires.primaryPos();

        //Shoot second
        intakeMotor.setPower(.5);
        Thread.sleep(100);
        intakeMotor.setPower(0);
        forks.secondaryPos();
        Thread.sleep(200);
        fires.secondaryPos();
        Thread.sleep(200);
        forks.primaryPos();
        Thread.sleep(300);
        intakeMotor.setPower(1);
        Thread.sleep(1000);
        fires.tertiaryPos();
        intakeMotor.setPower(0);
        Thread.sleep(300);
        fires.primaryPos();
        Thread.sleep(500);
        intakeMotor.setPower(1);
        Thread.sleep(1000);
        //Shoot third
        forks.secondaryPos();
        Thread.sleep(200);
        fires.secondaryPos();
        Thread.sleep(200);
        forks.primaryPos();
        Thread.sleep(1000);
        fires.tertiaryPos();
        Thread.sleep(300);
        FLW.setPower(-.75);
        BLW.setPower(-.75);
        FRW.setPower(-.75);
        BRW.setPower(-.75);
        Thread.sleep(200);
    }

    private double LeBotsEyes(double pastError, boolean adjustMotor){
        double desiredX;
        telemetry.addData("D",true);
        if(detectTagSelective(limelight,telemetry) != null){
            LLResultTypes.FiducialResult yes = detectTagSelective(limelight,telemetry);
//            if(launcherMotor1.getPower() == motorPowerFar){
//                if(yes.getFiducialId() == 20){
//                    desiredX = -.25;
//                }else{
//                    desiredX = -9.5;
//                    telemetry.addData("Desired x changing",true);
//                }
//            }else{
//                if(yes.getFiducialId() == 20){
//                    desiredX = -2.5;
//                }else{
//                    desiredX = -2.5;
//                }
//            }
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

            //            if(tx < -5){
            //                FLW.setPower(-.1);
            //                FRW.setPower(.1);
            //                BRW.setPower(.1);
            //                BLW.setPower(-.1);
            //            }else if(tx > 5){
            //                FRW.setPower(-.1);
            //                FLW.setPower(1);
            //                BRW.setPower(-.1);
            //                BLW.setPower(.1);
            //            }
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
}

