package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTag;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

import java.net.InetAddress;

@TeleOp
public class AnimosityAndMortification extends LinearOpMode implements LimelightTag, LimelightColor {

    private static Limelight3A limelight;
    private static int timer = 0;
    private static int pipeline = 0; //0 = AprilTag, 1 = Green, 2 = Purple

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
//`            tagDetection(limelight, telemetry);
            colorDetectionGreen(limelight, telemetry);
            telemetry.update();
        }
    }

    public void updatePhoneConsole() {
        telemetry.update();
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
    public void green() {

    }

    @Override
    public void purple() {

    }
}