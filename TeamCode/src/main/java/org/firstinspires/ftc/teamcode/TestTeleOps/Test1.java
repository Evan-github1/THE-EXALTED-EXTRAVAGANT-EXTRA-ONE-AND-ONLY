package org.firstinspires.ftc.teamcode.TestTeleOps;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class Test1 extends LinearOpMode {

    private DcMotorEx outtake;

    // Motor constants
    private final double TICKS_PER_REV = 28;  // GoBilda 5023 encoder
    private final double GEAR_RATIO = 1/2.0; // flywheel = motor * 2

    @Override
    public void runOpMode() {
        // Initialize motor
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        double targetRPM = 1500; // default RPM
        while (opModeIsActive()) {

            // Example: adjust RPM with gamepad buttons
            if(gamepad1.dpad_up) targetRPM += 50;   // increase RPM
            if(gamepad1.dpad_down) targetRPM -= 50; // decrease RPM
            if(targetRPM < 0) targetRPM = 0;

            setMotorRPM(outtake, targetRPM);

            // Telemetry
            telemetry.addData("Target Output RPM", targetRPM);
            telemetry.addData("Motor RPM", outtake.getVelocity() * 60 / TICKS_PER_REV);
            telemetry.update();

            sleep(50); // small delay to avoid spamming
        }
    }

    private void setMotorRPM(DcMotorEx motor, double flywheelRPM) {
        double motorRPM = flywheelRPM / GEAR_RATIO; // convert flywheel RPM to motor RPM
        double ticksPerSecond = motorRPM * TICKS_PER_REV / 60.0;
        motor.setVelocity(ticksPerSecond);
    }

}
