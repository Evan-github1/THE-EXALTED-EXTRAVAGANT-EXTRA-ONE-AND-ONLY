package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

import java.util.List;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
@TeleOp
public class thesaurusdotcom extends Movable {
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        FLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BLW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FRW.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
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

            telemetry.addData("FLW Encoder", FLW.getCurrentPosition());
            telemetry.addData("FRW Encoder", FRW.getCurrentPosition());
            telemetry.addData("BLW Encoder", BLW.getCurrentPosition());
            telemetry.addData("BRW Encoder", BRW.getCurrentPosition());


            telemetry.update();
        }
    }
}
