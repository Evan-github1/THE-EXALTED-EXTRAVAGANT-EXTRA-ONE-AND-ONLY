package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightTag;
import org.firstinspires.ftc.teamcode.RobotFunctions.LimelightColor;
import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@TeleOp
public class MaliceAndCondescension extends Movable implements LimelightTag, LimelightColor {

    private static Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        waitForStart();

//        new Thread(() -> {
//            while (opModeIsActive()) {
//                try {
//                    tagDetection(limelight,telemetry);
//                    Thread.sleep(1000);
//                    colorDetectionGreen(limelight,telemetry);
//                    Thread.sleep(1000);
//                    colorDetectionPurple(limelight,telemetry);
//                    Thread.sleep(1000);
//                } catch (InterruptedException e) {
//                    throw new RuntimeException(e);
//                }
//            }
//        }).start();

        while (opModeIsActive()) {
            telemetry.addData("Status", "Running");
            tagDetection(limelight, telemetry);
            updatePhoneConsole();
        }
    }

    public void updatePhoneConsole() {
        telemetry.update();
    }

    @Override
    public void tag21() {
        FLW.setPower(1);
        BLW.setPower(-1);
        FRW.setPower(1);
        BRW.setPower(-1);
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