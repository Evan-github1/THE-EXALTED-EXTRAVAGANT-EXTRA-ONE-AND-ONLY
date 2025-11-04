package org.firstinspires.ftc.teamcode.TestTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@TeleOp
public class Test1 extends Movable {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);

        enableEncoders();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            if (gamepad1.a) { // bottom
                FLW.setPower(1);
            } else if (gamepad1.b) { //right
                FRW.setPower(1);
            } else if (gamepad1.x) { // left
                BLW.setPower(1);
            } else if (gamepad1.y) { // top
                BRW.setPower(1);
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