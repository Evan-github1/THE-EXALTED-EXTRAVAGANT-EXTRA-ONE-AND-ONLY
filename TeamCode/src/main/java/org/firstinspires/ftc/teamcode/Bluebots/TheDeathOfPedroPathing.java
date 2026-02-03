package org.firstinspires.ftc.teamcode.Bluebots;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.ChamberState;
import org.firstinspires.ftc.teamcode.RobotFunctions.Colors;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

public abstract class TheDeathOfPedroPathing extends Movable {

    protected static Limelight3A limelight;
    protected static DcMotor swivelTurretMotor;
    protected static DcMotor intakeMotor;
    protected static DcMotorEx outtakeMotor;
    protected static boolean intakeToggle, outtakeToggle;
    protected static Servo gatewayServo;
    protected static DoubleSwitchedServo gateways;
    protected static Servo wiperL, wiperR;
    protected static DoubleSwitchedServo wipersL, wipersR;
    protected static DoubleSwitchedServo swivelTurret;
    protected static boolean turnLeft;
    protected static boolean tracking;
    protected static Servo hoodServo;
    protected final static double ROBOT_LENGTH = 15.875; // inches
    protected final static double ROBOT_WIDTH = 17.125; // also inches

    protected static Follower follower;
    protected static int pathState;
    protected int motifID;
    protected static Colors leftStoredColor;
    protected static Colors rightStoredColor;
    protected static ChamberState leftState;
    protected static ChamberState rightState;


    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        motifID = -1;

        pathState = 0;
        follower = createFollower(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(0); // april tags
        tracking = true;
        turnLeft = false;

        swivelTurretMotor = hardwareMap.get(DcMotor.class, "swivelTurret");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        intakeToggle = false;
        outtakeToggle = false;
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        gateways = new DoubleSwitchedServo(gatewayServo, .26, .73); // .26: right side (when looking at the back)
        gateways.secondaryPos();

        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);
        wipersL.primaryPos();
        wipersR.primaryPos();

        hoodServo = hardwareMap.get(Servo.class, "hood");
        hoodServo.setPosition(0);

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        FLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        limelight.pipelineSwitch(3); // motif mode

    }

    protected void motifMacroShoot() {
        final Colors[][] MOTIFS = {
                {Colors.GREEN, Colors.PURPLE, Colors.PURPLE}, // 21
                {Colors.PURPLE, Colors.GREEN, Colors.PURPLE}, // 22
                {Colors.PURPLE, Colors.PURPLE, Colors.GREEN} // 23
        };
        new Thread(() -> {
            if (motifID == 21 || motifID == 22 || motifID == 23) {
                Colors[] motif = MOTIFS[motifID - 21];
                for (int i = 0; i < motif.length; i++) {
                    if (leftStoredColor == motif[i]) {
                        liftLeftWiperNT();
                    } else if (rightStoredColor == motif[i]) {
                        liftRightWiperNT();
                    }
                    sleep(300);
                }
            }
        }).start();
    }


    protected void liftRightWiperNT() {
        wipersR.secondaryPos();
        rightState = ChamberState.EMPTY;
        rightStoredColor = Colors.UNKNOWN;
        sleep(1000);
        wipersR.primaryPos();
        rightState = ChamberState.EMPTY;
        rightStoredColor = Colors.UNKNOWN;
    }

    protected void liftLeftWiperNT() {
        wipersL.secondaryPos();
        leftState = ChamberState.EMPTY;
        leftStoredColor = Colors.UNKNOWN;
        sleep(1000);
        wipersL.primaryPos();
        leftState = ChamberState.EMPTY;
        leftStoredColor = Colors.UNKNOWN;
    }

    protected void liftRightWiper() {
        new Thread(() -> {
            wipersR.secondaryPos();
            rightState = ChamberState.EMPTY;
            rightStoredColor = Colors.UNKNOWN;
            sleep(1000);
            wipersR.primaryPos();
            rightState = ChamberState.EMPTY;
            rightStoredColor = Colors.UNKNOWN;
        }).start();
    }

    protected void liftLeftWiper() {
        new Thread(() -> {
            wipersL.secondaryPos();
            leftState = ChamberState.EMPTY;
            leftStoredColor = Colors.UNKNOWN;
            sleep(1000);
            wipersL.primaryPos();
            leftState = ChamberState.EMPTY;
            leftStoredColor = Colors.UNKNOWN;
        }).start();
    }
}
