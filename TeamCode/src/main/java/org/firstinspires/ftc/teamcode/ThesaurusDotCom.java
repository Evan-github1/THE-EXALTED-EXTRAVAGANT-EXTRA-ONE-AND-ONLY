package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;
import org.firstinspires.ftc.teamcode.RobotFunctions.TripleSwitchedServo;

@TeleOp
public class ThesaurusDotCom extends Movable {

    private static DcMotor intakeMotor, launcherMotor1, launcherMotor2;
    private static Servo lt1;
    private static Servo lt2;
    private static Servo fire;
    private static TripleSwitchedServo fires;
    private static Servo fork;
    private static DoubleSwitchedServo forks;
    private static boolean loading;

    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        intakeMotor = hardwareMap.get(DcMotor.class,"INT");
        lt1 = hardwareMap.get(Servo.class,"LT1");
        lt2 = hardwareMap.get(Servo.class,"LT2");
        fire = hardwareMap.get(Servo.class, "FIRE");
        fires = new TripleSwitchedServo(fire,.61,.51,.43);
        launcherMotor1 = hardwareMap.get(DcMotor.class,"LAU1");
        launcherMotor2 = hardwareMap.get(DcMotor.class,"LAU2");
        fork = hardwareMap.get(Servo.class,"FORK");
        forks = new DoubleSwitchedServo(fork,0.51,0.82);
        loading = false;

        long moveOutEndTimeOutput = 0;
        long moveInEndTimeInput = 0;

        FLW.setDirection(DcMotor.Direction.REVERSE);
        BLW.setDirection(DcMotor.Direction.REVERSE);
        FRW.setDirection(DcMotor.Direction.FORWARD);
        BRW.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        launcherMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        launcherMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        enableEncoders();

        waitForStart();

        fires.primaryPos();
        forks.primaryPos();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if(Math.abs(gamepad1.right_stick_x) > 0.5){
                FLW.setPower(gamepad1.right_stick_x);
                FRW.setPower(-gamepad1.right_stick_x);
                BLW.setPower(gamepad1.right_stick_x);
                BRW.setPower(-gamepad1.right_stick_x);
            }else{
                moveWheels(gamepad1.left_stick_x, gamepad1.left_stick_y);
            }

            if (gamepad1.a && delay(1002)) {
                moveOutEndTimeOutput = System.currentTimeMillis() + 1000;
                intakeMotor.setPower(-1);
                setTime();
            } else if (gamepad1.b) {
                intakeMotor.setPower(1);
            }else if (System.currentTimeMillis() <= moveOutEndTimeOutput) {
                intakeMotor.setPower(-1);
            }else if(gamepad1.y && delay()){
                setTime();
                if(launcherMotor1.getPower() != 0){
                    launcherMotor1.setPower(0);
                    launcherMotor2.setPower(0);
                }else{
                    launcherMotor1.setPower(1);
                    launcherMotor2.setPower(1);
                }
            }else if(gamepad1.left_bumper){
                lt1.setPosition(lt1.getPosition() - .01);
                lt1.setPosition(Math.max(lt1.getPosition(),.1));
                lt2.setPosition(lt2.getPosition() - .01);
                lt2.setPosition(Math.max(lt2.getPosition(),.1));
            }else if(gamepad1.right_bumper){
                lt1.setPosition(lt1.getPosition() + .01);
                lt1.setPosition(Math.min(lt1.getPosition(),.9));
                lt2.setPosition(lt2.getPosition() + .01);
                lt2.setPosition(Math.min(lt2.getPosition(),.9));
            }else if(gamepad1.left_trigger > 0.5 && delay(1000)){
                setTime();
                if(fire.getPosition() != .51){
                    new Thread(() -> {
                        fires.primaryPos();
                        forks.secondaryPos();
                        try {
                            Thread.sleep(200);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        fires.secondaryPos();
                        try {
                            Thread.sleep(200);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        forks.primaryPos();
                    }).start();
                }
            }else if(gamepad1.right_trigger > 0.5 && delay(1000)){
                setTime();
                if(fire.getPosition() == .51){
                    new Thread(()->{
                        fires.tertiaryPos();
                        try {
                            Thread.sleep(400);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        fires.primaryPos();
                    }).start();
                }else {
                    loading = true;
                    new Thread(()->{
                        intakeMotor.setPower(1);
                        try {
                            Thread.sleep(400);
                        } catch (InterruptedException e) {
                            throw new RuntimeException(e);
                        }
                        intakeMotor.setPower(0);
                        fires.secondaryPos();
                        loading = false;
                    }).start();
                }
            }else if(!loading){
                intakeMotor.setPower(0);
            }

            telemetry.addData("FLW Encoder", FLW.getCurrentPosition());
            telemetry.addData("FRW Encoder", FRW.getCurrentPosition());
            telemetry.addData("BLW Encoder", BLW.getCurrentPosition());
            telemetry.addData("BRW Encoder", BRW.getCurrentPosition());
            telemetry.addData("Left trigger", gamepad1.left_trigger);

            telemetry.update();
        }
    }
}
