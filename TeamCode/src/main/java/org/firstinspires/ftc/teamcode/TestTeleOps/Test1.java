package org.firstinspires.ftc.teamcode.TestTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test1 extends LinearOpMode {

    private static Servo swivelTurretServo;

    @Override
    public void runOpMode() throws InterruptedException {

        swivelTurretServo = hardwareMap.get(Servo.class, "swivelTurret");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");

            if(gamepad1.right_bumper) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() + .01);
            } else if (gamepad1.left_bumper) {
                swivelTurretServo.setPosition(swivelTurretServo.getPosition() - .01);
            }

            telemetry.addData("Swivel Turret Servo", swivelTurretServo.getPosition());
            telemetry.update();
        }
    }

}