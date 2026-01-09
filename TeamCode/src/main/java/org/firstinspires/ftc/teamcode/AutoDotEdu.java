package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import java.util.List;

@Autonomous
public class AutoDotEdu extends Movable implements LimelightColor, LimelightTags {
    private static Limelight3A limelight;
    private static Servo swivelServo;
    private static List<LLResultTypes.FiducialResult> results;
    private static DcMotor intakeMotor;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        //Super important initializations
        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        gateways = new DoubleSwitchedServo(gatewayServo, .23, .76);
        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        //Move to outer triangle

        //Turn towards the correct AprilTag

        //Shoot preloaded balls
        //Hooded flywheel motor go brr
        wipersL.secondaryPos();
        wipersL.primaryPos();
        wipersR.secondaryPos();
        wipersR.primaryPos();
        //

    }

    @Override
    public void tag20() {

    }

    @Override
    public void tag21() {}

    @Override
    public void tag22() {}

    @Override
    public void tag23() {}

    @Override
    public void tag24() {

    }

    @Override
    public void nothing() {

    }

    @Override
    public void green() {}

    @Override
    public void purple() {}
}