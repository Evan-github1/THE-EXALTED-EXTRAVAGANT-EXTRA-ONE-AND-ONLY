//This is the Autonomous for 22335!

package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;

@Autonomous
public class DoomAndDisgust_RIGHT extends Movable{
    private static DcMotor intakeMotor, launcherMotor1, launcherMotor2;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static TripleSwitchedServo fires;
    private static Servo fork;
    private static DoubleSwitchedServo forks;

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
        forks = new DoubleSwitchedServo(fork,0.23,0.75);

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
        FLW.setPower(-.25);
        BLW.setPower(-.25);
        Thread.sleep(300);
        disablePower();
        lt1.setPosition(.484);
        lt2.setPosition(.484);
        forks.setPrimaryPos(.23);
        forks.setSecondaryPos(.75);
        launcherMotor1.setPower(1);
        launcherMotor2.setPower(1);
        Thread.sleep(1500);
        forks.primaryPos();
        Thread.sleep(500);
        intakeMotor.setPower(1);
        Thread.sleep(350);
        intakeMotor.setPower(0);
        fires.secondaryPos();
        Thread.sleep(150);
        fires.tertiaryPos();
        Thread.sleep(400);
        fires.primaryPos();
        intakeMotor.setPower(1);
        Thread.sleep(1000);
        intakeMotor.setPower(0);
        fires.secondaryPos();
        Thread.sleep(1000);
        fires.tertiaryPos();
        Thread.sleep(400);
        fires.primaryPos();
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

}
