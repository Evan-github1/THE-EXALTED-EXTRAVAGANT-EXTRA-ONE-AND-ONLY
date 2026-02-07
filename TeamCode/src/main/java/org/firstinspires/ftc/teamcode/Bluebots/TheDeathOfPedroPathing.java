package org.firstinspires.ftc.teamcode.Bluebots;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower;

import com.pedropathing.follower.Follower;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.ChamberState;
import org.firstinspires.ftc.teamcode.RobotFunctions.ColorSensing;
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
    protected final static double ROBOT_LENGTH = 16; // inches
    protected final static double ROBOT_WIDTH = 17.25; // also inches

    protected static Follower follower;
    protected static int pathState;
    protected int motifID;
    protected static Colors leftStoredColor;
    protected static Colors rightStoredColor;
    protected static ChamberState leftState;
    protected static ChamberState rightState;
    protected static volatile boolean shooting = false;
    protected static ColorSensing colorSensing;

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

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(2, 0, 0, 13.2);
        outtakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        leftState = ChamberState.EMPTY;
        rightState = ChamberState.EMPTY;
        leftStoredColor = Colors.UNKNOWN;
        rightStoredColor = Colors.UNKNOWN;
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        colorSensing = new ColorSensing(hardwareMap, 18);

    }

    protected void motifMacroShoot() {
        // sorry if detectColors() is spammed everywhere
        shooting = true;
        detectColors();
        final Colors[][] MOTIFS = {
                {Colors.GREEN, Colors.PURPLE, Colors.PURPLE}, // 21
                {Colors.PURPLE, Colors.GREEN, Colors.PURPLE}, // 22
                {Colors.PURPLE, Colors.PURPLE, Colors.GREEN} // 23
        };
        gatewayServo.setPosition(.495);
        if (motifID == 21 || motifID == 22 || motifID == 23) {
            detectColors();
            Colors[] motif = MOTIFS[motifID - 21];
            for (int i = 0; i < motif.length; i++) {
                detectColors();
                if (leftStoredColor == motif[i]) {
                    liftLeftWiperNT();
                } else if (rightStoredColor == motif[i]) {
                    liftRightWiperNT();
                } else if (leftStoredColor != Colors.UNKNOWN) {
                    liftLeftWiperNT();
                } else if (rightStoredColor != Colors.UNKNOWN) {
                    liftRightWiperNT();
                }
                sleep(250);
            }
        } else {
            // shoot whatever it sees
            for (int i = 1; i <= 3; i++) {
                detectColors();
                if (leftStoredColor != Colors.UNKNOWN) {
                    liftLeftWiperNT();
                } else if (rightStoredColor != Colors.UNKNOWN) {
                    liftRightWiperNT();
                }
            }
        }

        shooting = false;
    }


    protected void liftRightWiperNT() {
        wipersR.secondaryPos();
        rightState = ChamberState.EMPTY;
        rightStoredColor = Colors.UNKNOWN;
        sleep(500);
        wipersR.primaryPos();
        rightState = ChamberState.EMPTY;
        rightStoredColor = Colors.UNKNOWN;
        sleep(500);
        intakeMotor.setPower(1);
    }

    protected void liftLeftWiperNT() {
        wipersL.secondaryPos();
        leftState = ChamberState.EMPTY;
        leftStoredColor = Colors.UNKNOWN;
        sleep(500);
        wipersL.primaryPos();
        leftState = ChamberState.EMPTY;
        leftStoredColor = Colors.UNKNOWN;
        sleep(500);
        intakeMotor.setPower(1);
    }

    private void liftRightWiper() {
        new Thread(() -> {
            gateways.secondaryPos();
            wipersR.secondaryPos();
            rightState = ChamberState.EMPTY;
            rightStoredColor = Colors.UNKNOWN;
            sleep(1000);
            wipersR.primaryPos();
            rightState = ChamberState.EMPTY;
            rightStoredColor = Colors.UNKNOWN;
        }).start();
    }

    private void liftLeftWiper() {
        new Thread(() -> {
            gateways.primaryPos();
            wipersL.secondaryPos();
            leftState = ChamberState.EMPTY;
            leftStoredColor = Colors.UNKNOWN;
            sleep(1000);
            wipersL.primaryPos();
            leftState = ChamberState.EMPTY;
            leftStoredColor = Colors.UNKNOWN;
        }).start();
    }

    protected void detectColors() {
        if (rightState == ChamberState.EMPTY) {
            Colors detected = colorSensing.detectColorRight(telemetry);
            if (detected != Colors.UNKNOWN && wiperR.getPosition() != wipersR.getSecondaryPos()) {
                rightStoredColor = detected;
                rightState = ChamberState.LOADED;
            }
        }

        if (leftState == ChamberState.EMPTY) {
            Colors detected = colorSensing.detectColorLeft(telemetry);
            if (detected != Colors.UNKNOWN && wiperL.getPosition() != wipersL.getSecondaryPos()) {
                leftStoredColor = detected;
                leftState = ChamberState.LOADED;
            }
        }
    }
}
