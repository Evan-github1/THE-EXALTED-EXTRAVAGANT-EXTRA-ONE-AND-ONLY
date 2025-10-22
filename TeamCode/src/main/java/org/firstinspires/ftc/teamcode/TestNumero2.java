package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotFunctions.DoubleSwitchedServo;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

import java.time.Duration;
import java.time.Instant;
import java.util.List;

@TeleOp
public class TestNumero2 extends LinearOpMode {

    private static Servo gatewayServo;
    private static DoubleSwitchedServo gateways;
    private static long time;
    @Override
    public void runOpMode() throws InterruptedException {
        gatewayServo = hardwareMap.get(Servo.class, "gateway");
        gateways = new DoubleSwitchedServo(gatewayServo, .23, .76);
        time = System.currentTimeMillis();
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad2.b && delay()) {
                gateways.quickSwitch();
                time = System.currentTimeMillis();
            }

            telemetry.update();
        }
    }

    public boolean delay() {
        return System.currentTimeMillis() >= time + 250;
    }


}