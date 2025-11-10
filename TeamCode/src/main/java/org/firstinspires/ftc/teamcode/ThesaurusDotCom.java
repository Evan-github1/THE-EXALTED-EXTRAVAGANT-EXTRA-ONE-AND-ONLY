package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImpl;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;
import java.util.List;
@TeleOp
public class ThesaurusDotCom extends Movable {

    private static DcMotor intakeMotor, launcherMotor1, launcherMotor2;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static TripleSwitchedServo fires;


    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotor.class,"INT");
        lt1 = hardwareMap.get(Servo.class,"LT1");
        lt2 = hardwareMap.get(Servo.class,"LT2");
        fire = hardwareMap.get(Servo.class, "FIRE");
        fires = new TripleSwitchedServo(fire,.58,.43,.3);
        launcherMotor1 = hardwareMap.get(DcMotor.class,"LAU1");
        launcherMotor2 = hardwareMap.get(DcMotor.class,"LAU2");

        long moveOutEndTime = 0;

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        //TODO: Make sure direction for intakeMotor is correct
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

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

            if (gamepad1.a && delay(1002)) {
                moveOutEndTime = System.currentTimeMillis() + 1000;
                intakeMotor.setPower(-1);
                setTime();
            } else if (gamepad1.b) {
                intakeMotor.setPower(1);
            }else if (System.currentTimeMillis() <= moveOutEndTime) {
                intakeMotor.setPower(-1);
            }else if(gamepad1.x && delay()) {
                fires.quickSwitch();
            }else if(gamepad1.y){
                if(launcherMotor1.getPower() != 0){
                    launcherMotor1.setPower(0);
                    launcherMotor2.setPower(0);
                }else{
                    launcherMotor1.setPower(1);
                    launcherMotor2.setPower(1);
                }
            }else{
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
