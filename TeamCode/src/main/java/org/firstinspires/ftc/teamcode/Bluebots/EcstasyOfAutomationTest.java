package org.firstinspires.ftc.teamcode.Bluebots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
public class EcstasyOfAutomationTest extends EcstasyOfAutomation {
    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        waitForStart();

        new Thread(() -> {
            try {
                Thread.sleep(25000);
                // move out of starting pos
                move(Direction.BACKWARD, 800, .6); // placeholder
                // turn left
                move(Direction.LEFT, 1000);
            } catch (InterruptedException e) {} // swallow
        }).start();

        new Thread(() -> {
            while (!isStopRequested()) {

            }
        }).start();

        new Thread(() -> {
            // detect for tag
            outtakeMotor.setVelocity(getTargetTicksPerSec(28, 2200));
            intakeMotor.setPower(1);
            hoodServo.setPosition(.79);

            sleep(4000);
            //PrincessEyes(20);

            liftRightWiper();
            sleep(1000);
            liftLeftWiper();
            sleep(1000);
            liftRightWiper();
            outtakeMotor.setVelocity(getTargetTicksPerSec(28, 0));
            sleep(1000);


            // collect balls
            //move(Direction.FORWARD, 750);
            // detect for tag (again)
            //PrincessEyes();
            // shoot da balls
        }).start();
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

    @Override
    public void nothing() {

    }
}