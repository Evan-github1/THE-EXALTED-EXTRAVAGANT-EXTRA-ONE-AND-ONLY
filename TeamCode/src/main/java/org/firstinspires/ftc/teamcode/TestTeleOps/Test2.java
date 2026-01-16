package org.firstinspires.ftc.teamcode.TestTeleOps;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@TeleOp
public class Test2 extends Movable {
    private static GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        waitForStart();

        while (opModeIsActive()) {
            Pose2D pose = pinpoint.getPosition();

            moveWheels(gamepad1.left_stick_x, gamepad1.left_stick_y);

            telemetry.addData("X", pose.getX(DistanceUnit.INCH));
            telemetry.addData("Y", pose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (deg)", pose.getHeading(AngleUnit.DEGREES));

            telemetry.update();

            telemetry.update();
        }
    }

    public boolean delay() {
        return System.currentTimeMillis() >= time + 250;
    }

    public boolean delay(long duration) {
        return System.currentTimeMillis() >= time + duration;
    }


    @Override
    public void green() {

    }

    @Override
    public void purple() {

    }

    @Override
    public void tag20() {

    }

    @Override
    public void tag21() {

    }

    @Override
    public void tag22() {

    }

    @Override
    public void tag23() {

    }

    @Override
    public void tag24() {

    }
}