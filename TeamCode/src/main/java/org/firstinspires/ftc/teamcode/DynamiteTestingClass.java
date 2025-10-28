package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

import java.util.List;

@TeleOp
public class DynamiteTestingClass extends Movable {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            /*
        BR - 1 (0)
        FR - 3 (1)
        FL = 2 (3)
        BL - 0 (2)
         */
            //What should be spinning back right is spinning front left.
            //What should be spinning back left is spinning front right.
            //What should be spinning
            if (gamepad1.a) {
                FLW.setPower(1);
            } else if (gamepad1.b) {
                FRW.setPower(1);
            } else if (gamepad1.x) {
                BLW.setPower(1);
            } else if (gamepad1.y) {
                BRW.setPower(1);
            } else {
                disablePower();
            }

            telemetry.update();
        }
    }

}