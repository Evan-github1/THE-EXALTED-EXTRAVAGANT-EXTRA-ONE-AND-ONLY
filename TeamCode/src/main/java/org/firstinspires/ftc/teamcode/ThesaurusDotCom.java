package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
<<<<<<< Updated upstream
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
=======
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;
>>>>>>> Stashed changes
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import java.util.List;

@TeleOp
public class ThesaurusDotCom extends Movable {
<<<<<<< Updated upstream

    private static DcMotor intakeMotor;
    private static Servo lt1;
    private static Servo lt2;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotor.class,"INT");
        lt1 = hardwareMap.get(Servo.class,"lt1");
        lt2 = hardwareMap.get(Servo.class,"lt2");
=======
    private static Servo artifactHServo;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        artifactHServo = hardwareMap.get(Servo.class, "artifact handler");
>>>>>>> Stashed changes

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        //TODO: Make sure direction for intakeMotor is correct
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);

        enableEncoders();
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            moveWheels(gamepad1.left_stick_x, gamepad1.left_stick_y);

            if (gamepad1.left_bumper) {
                FLW.setPower(-1);
                FRW.setPower(1);
                BLW.setPower(-1);
                BRW.setPower(1);
            } else if (gamepad1.right_bumper) {
                FLW.setPower(1);
                FRW.setPower(-1);
                BLW.setPower(1);
                BRW.setPower(-1);
            } else {
                disablePower();
            }

            if (gamepad1.b) {
                intakeMotor.setPower(1);
            }
            Thread name = new Thread(() -> {
                intakeMotor.setPower(-1);
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                intakeMotor.setPower(0);
            });
            if (gamepad1.a && delay(1001)) {
                name.start();
                setTime();
            } else if (!(name.isAlive())) {
                intakeMotor.setPower(0);
            }

            telemetry.addData("FLW Encoder", FLW.getCurrentPosition());
            telemetry.addData("FRW Encoder", FRW.getCurrentPosition());
            telemetry.addData("BLW Encoder", BLW.getCurrentPosition());
            telemetry.addData("BRW Encoder", BRW.getCurrentPosition());

            telemetry.update();
        }
    }
}