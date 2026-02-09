package org.firstinspires.ftc.teamcode.Bluebots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class JavaBotdrillardCloseShootRed extends JavaBotdrillard {

    private final ElapsedTime matchTimer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();
        matchTimer.reset();

        outtakeMotor.setVelocity(1500);
        hoodServo.setPosition(.5);

        move(Direction.FORWARD, 2500, .3);

        sleep(5000);

        liftRightWiper();
        sleep(500);
        liftLeftWiper();
        sleep(500);
        liftRightWiper();

        sleep(500);
        move(Direction.BACKWARD, 1000, .45);

        while (opModeIsActive() && matchTimer.seconds() < 25.0) {
            telemetry.addData("Waiting, current time", matchTimer.seconds());
            telemetry.update();
        }

        move(Direction.STRAFE_LEFT, 2000, .6);
    }
}
