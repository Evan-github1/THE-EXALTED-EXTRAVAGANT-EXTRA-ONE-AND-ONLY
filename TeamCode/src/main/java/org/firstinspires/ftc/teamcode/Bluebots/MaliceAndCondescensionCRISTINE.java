package org.firstinspires.ftc.teamcode.Bluebots;
import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.ChamberState;
import org.firstinspires.ftc.teamcode.RobotFunctions.ColorSensing;
import org.firstinspires.ftc.teamcode.RobotFunctions.Colors;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
@TeleOp
public class MaliceAndCondescensionCRISTINE extends Movable implements LimelightTags { // robot #22335

    private static Limelight3A limelight;
    private static DcMotorEx swivelTurretMotor;
    private static DcMotor intakeMotor;
    private static DcMotorEx outtakeMotor;
    private static boolean intakeToggle, outtakeToggle;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;
    private static int targetedID;
    private static double targetVelocity;
    private static Servo hoodServo;

    private static ColorSensing colorSensing;
    private static int motifID;

    private static ChamberState leftState;
    private static ChamberState rightState;

    private static Colors leftStoredColor;
    private static Colors rightStoredColor;

    private static volatile boolean sweep;
    private static volatile boolean moveLeft;

    private static final int TURRET_LIMIT_LEFT = -300;
    private static final int TURRET_LIMIT_RIGHT = 300;
    private static int currentPipeline = -1;
    private static final double OUTTAKE_TICKS_PER_REV = 28;
    private static Servo light;

    public double prevSmooth;
    public double alpha = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        prevSmooth = 0;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(3); // motif mode
        targetedID = 20;

        swivelTurretMotor = hardwareMap.get(DcMotorEx.class, "swivelTurret");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        intakeToggle = false;
        outtakeToggle = false;
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetVelocity = 2000;

        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        gateways = new DoubleSwitchedServo(gatewayServo, .26, .73); // .26: left front

        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);

        hoodServo = hardwareMap.get(Servo.class, "hood");

        FLW.setDirection(DcMotor.Direction.FORWARD);
        BLW.setDirection(DcMotor.Direction.FORWARD);
        FRW.setDirection(DcMotor.Direction.REVERSE);
        BRW.setDirection(DcMotor.Direction.REVERSE);
        FLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        colorSensing = new ColorSensing(hardwareMap, 18);

        leftStoredColor = Colors.UNKNOWN;
        rightStoredColor = Colors.UNKNOWN;
        leftState = ChamberState.EMPTY;
        rightState = ChamberState.EMPTY;

        motifID = -1;
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        sweep = false;
        moveLeft = true;

        waitForStart();

        wipersL.primaryPos();
        wipersR.primaryPos();
        hoodServo.setPosition(0);

        swivelTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swivelTurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(2, 0, 0, 13.2);
        outtakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        light = hardwareMap.get(Servo.class, "lights");

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("Camera is targetting", targetedID);
            telemetry.addData("Outtake Encoder", outtakeMotor.getCurrentPosition());
            omnidirectionalMovement(gamepad1.left_stick_x, gamepad1.left_stick_y);
            turn();

            int id = detectTag(limelight, telemetry);

            int desiredPipeline;

            if (motifID == -1) {
                desiredPipeline = 3; // motif mode
            } else {
                desiredPipeline = 0; // targeting mode
            }

            if (desiredPipeline != currentPipeline) {
                limelight.pipelineSwitch(desiredPipeline);
                currentPipeline = desiredPipeline;
            }
            if (motifID == -1 && (id == 21 || id == 22 || id == 23)) {
                motifID = id;
            }

            telemetry.addData("Motif ID", motifID);
            telemetry.addLine("I spy with my little eye... " + id);

            if (gamepad1.leftStickButtonWasPressed()) {
                sweep = !sweep;
            }

            final double POWER = .1;
            telemetry.addData("Sweep", sweep);
            if (sweep) {
                if (moveLeft) {
                    swivelTurretMotor.setPower(-POWER);
                } else {
                    swivelTurretMotor.setPower(POWER);
                }

                if (swivelTurretMotor.getCurrentPosition() <= TURRET_LIMIT_LEFT) {
                    moveLeft = false;
                } else if (swivelTurretMotor.getCurrentPosition() >= TURRET_LIMIT_RIGHT) {
                    moveLeft = true;
                }
            }


            if (!sweep) {
                if (gamepad1.dpad_right && swivelTurretMotor.getCurrentPosition() <= TURRET_LIMIT_RIGHT) {
                    swivelTurretMotor.setPower(POWER);
                } else if (gamepad1.dpad_left && swivelTurretMotor.getCurrentPosition() >= TURRET_LIMIT_LEFT) {
                    swivelTurretMotor.setPower(-POWER);
                } else {
                    swivelTurretMotor.setPower(0);
                }
            }

            telemetry.addData("Turret Rotate Encoder", swivelTurretMotor.getCurrentPosition());


            if (id == targetedID) {
                telemetry.addLine("I see the targeted ID!");
                sweep = false;
                double tx = getTX(limelight);
                // red: .277, green: .444
                if (tx <= -1) {
                    swivelTurretMotor.setPower(-POWER * 1.25);
                } else if (tx >= 1) {
                    swivelTurretMotor.setPower(POWER * 1.25);
                }

                if (tx < 1 && -1 > tx) {
                    light.setPosition(.444);
                } else {
                    light.setPosition(.277);
                }
            } else {
                light.setPosition(.277);
            }

//            if (id == targetedID) {
//                double tx = getTX(limelight);
//                telemetry.addLine("I see the targeted ID at tx = " +tx);
//                sweep = false;
//                double smoothedInput = alpha * tx + (1 - alpha) * prevSmooth;
//                telemetry.addLine("Smoothed input value: "+smoothedInput);
//                swivelTurretMotor.setPower(smoothedInput);
//                prevSmooth = smoothedInput;
//            }

            if (gamepad1.b && delay()) {
                gateways.quickSwitch();
                time = System.currentTimeMillis();
            }

            if (gamepad1.left_trigger >= .5 && delay(1001)) {
                liftRightWiper();
                time = System.currentTimeMillis();
            } else if (gamepad1.right_trigger >= .5 && delay(1001)) {
                liftLeftWiper();
                time = System.currentTimeMillis();
            }

            if (gamepad1.y && delay()) {
                intakeToggle = !intakeToggle;
                time = System.currentTimeMillis();
            } else if (gamepad1.a && delay()) {
                outtakeToggle = !outtakeToggle;
                time = System.currentTimeMillis();
            }

            if (gamepad1.right_stick_y > .3) {
                hoodServo.setPosition(hoodServo.getPosition() - .01);
            } else if (gamepad1.right_stick_y < -.3 && hoodServo.getPosition() <= .8) {
                hoodServo.setPosition(hoodServo.getPosition() + .01);
            }
            telemetry.addData("Hood Position", hoodServo.getPosition());

            if (gamepad1.options && delay()) {
                if (targetedID == 20) {
                    targetedID = 24;
                } else if (targetedID == 24) {
                    targetedID = 20;
                }
                time = System.currentTimeMillis();
            }

            if (gamepad1.dpad_up && targetVelocity < 2000 && delay(100)) {
                targetVelocity += 50;
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && targetVelocity > 1500 && delay(100)) {
                targetVelocity -= 50;
                time = System.currentTimeMillis();
            }

            if (gamepad1.shareWasPressed()) {
                motifMacroShoot();
                time = System.currentTimeMillis();
            }

            telemetry.addData("Gateway Position", gatewayServo.getPosition());

            if (intakeToggle || gamepad1.x) {
                boolean LUp = wiperL.getPosition() == wipersL.getSecondaryPos();
                boolean RUp = wiperR.getPosition() == wipersR.getSecondaryPos();

                if (gamepad1.x) {
                    intakeMotor.setPower(-1);
                } else if (LUp || RUp) {
                    intakeMotor.setPower(0);
                } else {
                    intakeMotor.setPower(1);
                }

            } else if (!intakeToggle && !gamepad1.x) {
                intakeMotor.setPower(0);
            }

            if (outtakeToggle) {
                outtakeMotor.setVelocity(targetVelocity);
            } else {
                outtakeMotor.setVelocity(0);
            }
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Velocity", outtakeMotor.getVelocity());

            // GAMEPAD 2
            if (gamepad2.right_trigger >= .5 && delay(1001)) {
                liftLeftWiper();
                time = System.currentTimeMillis();
            } else if (gamepad2.left_trigger >= .5 && delay(1001)) {
                liftRightWiper();
                time = System.currentTimeMillis();
            }
            if (gamepad2.b && delay()) {
                gateways.quickSwitch();
                time = System.currentTimeMillis();
            }

            if (gamepad2.right_stick_y > 0.3) {
                hoodServo.setPosition(hoodServo.getPosition() - .01);
            } else if (gamepad2.right_stick_y < -0.3 && hoodServo.getPosition() <= .8) {
                hoodServo.setPosition(hoodServo.getPosition() + .01);
            }

            // END OF GAMEPAD 2

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

            telemetry.addData("Left Chamber", leftState);
            telemetry.addData("Left Stored Color", leftStoredColor);

            telemetry.addData("Right Chamber", rightState);
            telemetry.addData("Right Stored Color", rightStoredColor);
            telemetry.update();
        }
    }

    private void motifMacroShoot() {
        final Colors[][] MOTIFS = {
                {Colors.GREEN, Colors.PURPLE, Colors.PURPLE}, // 21
                {Colors.PURPLE, Colors.GREEN, Colors.PURPLE}, // 22
                {Colors.PURPLE, Colors.PURPLE, Colors.GREEN} // 23
        };
        gatewayServo.setPosition(.495);
        new Thread(() -> {
            if (motifID == 21 || motifID == 22 || motifID == 23) {
                Colors[] motif = MOTIFS[motifID - 21];
                for (int i = 0; i < motif.length; i++) {
                    if (leftStoredColor == motif[i]) {
                        liftLeftWiperNT();
                    } else if (rightStoredColor == motif[i]) {
                        liftRightWiperNT();
                    } else if (leftStoredColor != Colors.UNKNOWN) {
                        liftLeftWiperNT();
                    } else if (rightStoredColor != Colors.UNKNOWN) {
                        liftRightWiperNT();
                    }
                    sleep(300);
                }
            } else {
                // shoot whatever it sees
                for (int i = 1; i <= 3; i++) {
                    if (leftStoredColor != Colors.UNKNOWN) {
                        liftLeftWiperNT();
                    } else if (rightStoredColor != Colors.UNKNOWN) {
                        liftRightWiperNT();
                    }
                }
            }
        }).start();
    }

    private void liftRightWiperNT() {
        intakeMotor.setPower(-1);
        wipersR.secondaryPos();
        rightState = ChamberState.EMPTY;
        rightStoredColor = Colors.UNKNOWN;
        sleep(250);
        wipersR.primaryPos();
        rightState = ChamberState.EMPTY;
        rightStoredColor = Colors.UNKNOWN;
        sleep(500);
        intakeMotor.setPower(1);
    }

    private void liftLeftWiperNT() {
        intakeMotor.setPower(-1);
        wipersL.secondaryPos();
        leftState = ChamberState.EMPTY;
        leftStoredColor = Colors.UNKNOWN;
        sleep(250);
        wipersL.primaryPos();
        leftState = ChamberState.EMPTY;
        leftStoredColor = Colors.UNKNOWN;
        sleep(500);
        intakeMotor.setPower(1);
    }

    private void liftRightWiper() {
        new Thread(() -> {
            wipersR.secondaryPos();
            rightState = ChamberState.EMPTY;
            rightStoredColor = Colors.UNKNOWN;
            sleep(250);
            wipersR.primaryPos();
            rightState = ChamberState.EMPTY;
            rightStoredColor = Colors.UNKNOWN;
        }).start();
    }

    private void liftLeftWiper() {
        new Thread(() -> {
            wipersL.secondaryPos();
            leftState = ChamberState.EMPTY;
            leftStoredColor = Colors.UNKNOWN;
            sleep(250);
            wipersL.primaryPos();
            leftState = ChamberState.EMPTY;
            leftStoredColor = Colors.UNKNOWN;
        }).start();
    }

    private double getTargetTicksPerSec(double ticksPerRevolution, double desiredRPM) {
        return (ticksPerRevolution * desiredRPM) / 60;
    }

    @Override
    public void tag20() {}
    @Override
    public void tag21() {}
    @Override
    public void tag22() {}
    @Override
    public void tag23() {}
    @Override
    public void tag24() {}
    @Override
    public void nothing() {}

    @Override
    protected void turn() {
        final double POWER = .75;
        if (gamepad1.right_bumper) { // turn right
            FLW.setPower(-POWER);
            FRW.setPower(POWER);
            BLW.setPower(-POWER);
            BRW.setPower(POWER);
        } else if (gamepad1.left_bumper) { // turn left
            FLW.setPower(POWER);
            FRW.setPower(-POWER);
            BLW.setPower(POWER);
            BRW.setPower(-POWER);
        } else if (gamepad1.rightBumperWasReleased() && gamepad1.leftBumperWasReleased()
                && gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
            disablePower();
        }
    }
}