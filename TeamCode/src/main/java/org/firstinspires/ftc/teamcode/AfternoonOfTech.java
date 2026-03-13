package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.ChamberState;
import org.firstinspires.ftc.teamcode.RobotFunctions.ColorSensing;
import org.firstinspires.ftc.teamcode.RobotFunctions.Colors;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@TeleOp
public class AfternoonOfTech extends Movable {

    private static DcMotorEx swivelTurretMotor;
    private static DcMotor intakeMotor;
    private static DcMotorEx outtakeMotor;
    private static boolean intakeToggle, outtakeToggle;
    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static Servo wiperL, wiperR;
    private static DoubleSwitchedServo wipersL, wipersR;
    private static double targetVelocity;
    private static Servo hoodServo;

    private static Servo standL, standR;
    private static DoubleSwitchedServo standsL, standsR;

    private static ColorSensing colorSensing;

    private static ChamberState leftState;
    private static ChamberState rightState;

    private static Colors leftStoredColor;
    private static Colors rightStoredColor;

    private static volatile boolean moveLeft;
    /*
    hood to bumpers (not triggers)
    steering on right stick
     */
    private static final int TURRET_LIMIT_LEFT = -300;
    private static final int TURRET_LIMIT_RIGHT = 300;
    private static int currentPipeline = -1;
    private static Servo light;
    private static final double TPR = 28;

    public double prevSmooth;
    public double alpha = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        prevSmooth = 0;

        swivelTurretMotor = hardwareMap.get(DcMotorEx.class, "swivelTurret");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        intakeToggle = false;
        outtakeToggle = false;
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        targetVelocity = 3000;

        gatewayServo = hardwareMap.get(Servo.class, "gateway");

        wiperR = hardwareMap.get(Servo.class, "wiperR");
        wiperL = hardwareMap.get(Servo.class, "wiperL");

        standR = hardwareMap.get(Servo.class, "standR");
        standL = hardwareMap.get(Servo.class, "standL");

        gateways = new DoubleSwitchedServo(gatewayServo, .26, .73); // .26: left front

        wipersR = new DoubleSwitchedServo(wiperR, 1, .5);
        wipersL = new DoubleSwitchedServo(wiperL, 0, .5);

        standsL = new DoubleSwitchedServo(standR, 0, .75);
        standsR = new DoubleSwitchedServo(standL, 1, .25);

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

        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        standsR.primaryPos();
        standsL.primaryPos();

        wipersL.primaryPos();
        wipersR.primaryPos();
        hoodServo.setPosition(0);

        swivelTurretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        swivelTurretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setVelocityPIDFCoefficients(0, 0, 0, 20);

        light = hardwareMap.get(Servo.class, "lights");

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            omnidirectionalMovement(gamepad1.left_stick_x, gamepad1.left_stick_y);
            turn();

            final double POWER = .1;

            telemetry.addData("Turret Rotate Encoder", swivelTurretMotor.getCurrentPosition());

            if (gamepad1.b && delay()) {
                standsR.quickSwitch();
                standsL.quickSwitch();
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
                hoodServo.setPosition(clampHoodPosition(hoodServo.getPosition() - .01));
            } else if (gamepad1.right_stick_y < -.3 && hoodServo.getPosition() <= .8) {
                hoodServo.setPosition(clampHoodPosition(hoodServo.getPosition() + .01));
            }
            telemetry.addData("Hood Position", hoodServo.getPosition());

            if (gamepad1.dpad_up && targetVelocity < 4200 && delay(50)) {
                targetVelocity += 50;
                time = System.currentTimeMillis();
            } else if (gamepad1.dpad_down && targetVelocity > 2600 &&delay(50)) {
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
                outtakeMotor.setVelocity((targetVelocity * TPR) / 60);
            } else {
                outtakeMotor.setVelocity(0);
            }
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Velocity", (outtakeMotor.getVelocity() * 60) / TPR);

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
                hoodServo.setPosition(clampHoodPosition(hoodServo.getPosition() - .01));
            } else if (gamepad2.right_stick_y < -0.3 && hoodServo.getPosition() <= .8) {
                hoodServo.setPosition(clampHoodPosition(hoodServo.getPosition() + .01));
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

            leftStoredColor = colorSensing.detectColorLeft(telemetry);
            rightStoredColor = colorSensing.detectColorRight(telemetry);

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
            for (int i = 1; i <= 3; i++) {
                if (leftStoredColor != Colors.UNKNOWN) {
                    liftLeftWiperNT();
                } else if (rightStoredColor != Colors.UNKNOWN) {
                    liftRightWiperNT();
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

    private double clampHoodPosition(double requestedPosition) {
        return Math.max(0, Math.min(0.8, requestedPosition));
    }

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