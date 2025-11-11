package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import java.util.List;

@TeleOp
public class ThesaurusDotCom extends Movable {
    private static DcMotor intakeMotor;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo artifactHandler;
    private static TripleSwitchedServo artifactHandlerPos;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        lt1 = hardwareMap.get(Servo.class, "launcherTiltOne");

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);

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

            telemetry.addData("FLW Encoder", FLW.getCurrentPosition());
            telemetry.addData("FRW Encoder", FRW.getCurrentPosition());
            telemetry.addData("BLW Encoder", BLW.getCurrentPosition());
            telemetry.addData("BRW Encoder", BRW.getCurrentPosition());

            telemetry.update();
        }
    }
}
