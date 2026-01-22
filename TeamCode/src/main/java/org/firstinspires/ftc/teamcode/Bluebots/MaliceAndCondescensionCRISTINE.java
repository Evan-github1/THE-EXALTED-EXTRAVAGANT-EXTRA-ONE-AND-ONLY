package org.firstinspires.ftc.teamcode.Bluebots;
import android.graphics.Color;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.ColorSensing;
import org.firstinspires.ftc.teamcode.RobotFunctions.Colors;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTags;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
@TeleOp
public class MaliceAndCondescensionCRISTINE extends Movable implements LimelightTags { // robot #22335

    private static Limelight3A limelight;
    private static Servo swivelTurretServo;
    private static DcMotor intakeMotor;
    private static DcMotorEx outtakeMotor;
    private static boolean intakeToggle, outtakeToggle;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;
    private static DoubleSwitchedServo swivelTurret;
    private static int targetedID;
    private static double targetRPM;
    private static Servo hoodServo;
    private static volatile boolean turretStop;

    private static boolean sweepInit;
    private static boolean sweepActive;
    private static double sweepTarget;

    private static ColorSensing colorSensing;
    private static int motifID = -1;

    private static ChamberState leftState;
    private static ChamberState rightState;

    private static Colors leftStoredColor;
    private static Colors rightStoredColor;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.pipelineSwitch(3); // motif mode
        targetedID = 20;

        swivelTurretServo = hardwareMap.get(Servo.class, "swivelTurret");
        swivelTurret = new DoubleSwitchedServo(swivelTurretServo, .1, .54);

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        intakeToggle = false;
        outtakeToggle = false;
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetRPM = 1450;

        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        gateways = new DoubleSwitchedServo(gatewayServo, .26, .73); // .26: right side (when looking at the back)

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

        turretStop = true;

        sweepInit = false;
        sweepActive = false;
        sweepTarget = 0.07;

        colorSensing = new ColorSensing(hardwareMap, 18);

        leftStoredColor = Colors.UNKNOWN;
        rightStoredColor = Colors.UNKNOWN;
        leftState = ChamberState.EMPTY;
        rightState = ChamberState.EMPTY;

        waitForStart();

        wipersL.primaryPos();
        wipersR.primaryPos();
        swivelTurretServo.setPosition(.1975);
        hoodServo.setPosition(0);

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            telemetry.addData("CURRENT TARGETED ID", targetedID);
            telemetry.addData("Outtake Encoder", outtakeMotor.getCurrentPosition());
            omnidirectionalMovement(gamepad1.left_stick_x, gamepad1.left_stick_y);
            turn();

            int id = detectTag(limelight, telemetry);

            if (motifID == -1) {
                limelight.pipelineSwitch(3);
                if (id == 21 || id == 22 || id == 23)  motifID = id;
            } else {
                limelight.pipelineSwitch(0);
            }
            telemetry.addData("Motif ID", motifID);
            telemetry.addData("Turret Stop", turretStop);

            telemetry.addData("Swivel Turret Position", swivelTurretServo.getPosition());
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

            PrincessEyesv3(); // uses option button

            if (gamepad1.x) {
                intakeMotor.setPower(-1);
            }

            if (gamepad1.dpad_up && delay() && targetRPM <= 2300) {
                if (targetRPM == 1600) {
                    targetRPM = 2100;
                } else {
                    targetRPM += 50;
                }
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && delay() && targetRPM >= 1450) {
                if (targetRPM == 2100) {
                    targetRPM = 1600;
                } else {
                    targetRPM -= 50;
                }
                time = System.currentTimeMillis();
            }

            if (gamepad1.dpad_right && swivelTurretServo.getPosition() > swivelTurret.getPrimaryPos()) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - 0.004);
            } else if (gamepad1.dpad_left) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + 0.004);
            }


            if (gamepad1.leftStickButtonWasPressed() && delay(5000)) {
                motifMacroShoot();
                time = System.currentTimeMillis();
            }

            telemetry.addData("Gateway Position", gatewayServo.getPosition());

            if (intakeToggle) {
                boolean LUp = wiperL.getPosition() == wipersL.getSecondaryPos();
                boolean RUp = wiperR.getPosition() == wipersR.getSecondaryPos();

                if (LUp || RUp) {
                    intakeMotor.setPower(0);
                } else {
                    intakeMotor.setPower(1);
                }
            } else if (!gamepad1.x || !intakeToggle) {
                intakeMotor.setPower(0);
            }

            if (outtakeToggle) {
                outtakeMotor.setVelocity(getTargetTicksPerSec(28, targetRPM));
            } else {
                outtakeMotor.setVelocity(0);
            }
            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("Measured RPM", outtakeMotor.getVelocity() * 60.0 / 28);
            telemetry.addData("Ticks/sec", outtakeMotor.getVelocity());

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

            if (gamepad2.dpad_right && swivelTurretServo.getPosition() > swivelTurret.getPrimaryPos()) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - 0.004);
            } else if (gamepad2.dpad_left && swivelTurretServo.getPosition() < swivelTurret.getSecondaryPos()) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + 0.004);
            }

            // END OF GAMEPAD 2

            if (id == targetedID) {
                telemetry.addData("hi", "yes");
                turretStop = true;
                double tx = getTX(limelight);
                if (tx <= -3 && swivelTurretServo.getPosition() >= .57) {
                    // move turret left
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() - .0001);
                } else if (tx >= 3 && swivelTurretServo.getPosition() <= .07) {
                    // move turret right
                    swivelTurretServo.setPosition(swivelTurretServo.getPosition() + .0001);
                }
            }

            double TX = getTX(limelight);

            if (TX > 0 && id == targetedID) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - .0004);
            } else if (TX < 0 && id == targetedID) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + .0004);
            }

            if (rightState == ChamberState.EMPTY) {
                Colors detected = colorSensing.detectColorRight(telemetry);
                if (detected != Colors.UNKNOWN) {
                    rightStoredColor = detected;
                    rightState = ChamberState.LOADED;
                }
            }

            if (leftState == ChamberState.EMPTY) {
                Colors detected = colorSensing.detectColorLeft(telemetry);
                if (detected != Colors.UNKNOWN) {
                    leftStoredColor = detected;
                    leftState = ChamberState.LOADED;
                }
            }
            telemetry.addData("Left Chamber", leftState);
            telemetry.addData("Left Stored Color", leftStoredColor);

            telemetry.addData("Right Chamber", rightState);
            telemetry.addData("Right Stored Color", rightStoredColor);

            telemetry.addData("TX", TX);
            telemetry.update();
        }
    }

    private void motifMacroShoot() {
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

    enum ChamberState {
        EMPTY,
        LOADED
    }

    private void liftRightWiperNT() {
        wipersR.secondaryPos();
        rightState = ChamberState.EMPTY;
        rightStoredColor = Colors.UNKNOWN;
        sleep(1000);
        wipersR.primaryPos();
        rightState = ChamberState.EMPTY;
        rightStoredColor = Colors.UNKNOWN;
    }

    private void liftLeftWiperNT() {
        wipersL.secondaryPos();
        leftState = ChamberState.EMPTY;
        leftStoredColor = Colors.UNKNOWN;
        sleep(1000);
        wipersL.primaryPos();
        leftState = ChamberState.EMPTY;
        leftStoredColor = Colors.UNKNOWN;
    }

    private void liftRightWiper() {
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

    private void liftLeftWiper() {
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

    private double getTargetTicksPerSec(double ticksPerRev, double targetRPM) {
        return (ticksPerRev * targetRPM) / 60;
    }

    public void PrincessEyesv3() {

        double leftPos  = swivelTurret.getPrimaryPos();
        double rightPos = swivelTurret.getSecondaryPos();
        double step = 0.003;   // sweep speed per cycle

        // -------------------------
        // Initialization (runs once)
        // -------------------------
        if (!sweepInit) {
            sweepInit = true;
            sweepActive = false;
            sweepTarget = leftPos;
        }

        // -------------------------
        // Handle button press toggle
        // -------------------------
        if (gamepad1.share && !sweepActive && delay()) {

            sweepActive = true;
            double current = swivelTurretServo.getPosition();

            // Choose direction based on current position
            if (Math.abs(current - leftPos) < 0.1) {
                sweepTarget = rightPos;
            } else {
                sweepTarget = leftPos;
            }

            time = System.currentTimeMillis();
        }

        // -------------------------
        // Perform sweep motion
        // -------------------------
        if (sweepActive) {
            // Stop if AprilTag found
            int tag = detectTag(limelight, telemetry);
            if (tag == targetedID) {
                sweepActive = false;
                telemetry.addLine("Turret stopped: Tag 20 detected");
                return;
            }

            double current = swivelTurretServo.getPosition();
            double next;

            // Move toward target incrementally
            if (current < sweepTarget) {
                next = Math.min(current + step, sweepTarget);
            } else {
                next = Math.max(current - step, sweepTarget);
            }

            swivelTurretServo.setPosition(next);

            // End sweep when very close
            if (Math.abs(next - sweepTarget) < 0.005) {
                sweepActive = false;
            }
        }

        telemetry.addData("TurretPos", swivelTurretServo.getPosition());
        telemetry.addData("SweepActive", sweepActive);
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
        final double SPEED = 0.0025;
        if (gamepad1.right_bumper) { // turn right
            FLW.setPower(-POWER);
            FRW.setPower(POWER);
            BLW.setPower(-POWER);
            BRW.setPower(POWER);
            if (swivelTurretServo.getPosition() >= .07)
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - SPEED);
        } else if (gamepad1.left_bumper) { // turn left
            FLW.setPower(POWER);
            FRW.setPower(-POWER);
            BLW.setPower(POWER);
            BRW.setPower(-POWER);
            if (swivelTurretServo.getPosition() <= .57)
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + SPEED);
        } else if (gamepad1.rightBumperWasReleased() && gamepad1.leftBumperWasReleased()
                && gamepad1.left_stick_y == 0 && gamepad1.left_stick_x == 0) {
            disablePower();
        }
    }
}