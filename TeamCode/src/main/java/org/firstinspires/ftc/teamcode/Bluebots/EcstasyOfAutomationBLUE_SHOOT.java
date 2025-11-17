package org.firstinspires.ftc.teamcode.Bluebots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class EcstasyOfAutomationBLUE_SHOOT extends EcstasyOfAutomation {
    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();

        outtakeMotor.setVelocity(getTargetTicksPerSec(28, 2200));
        intakeMotor.setPower(1);
        hoodServo.setPosition(.82);

        sleep(5000);

        liftRightWiper();
        sleep(1000);
        liftLeftWiper();
        sleep(1000);
        liftRightWiper();
        outtakeMotor.setVelocity(getTargetTicksPerSec(28, 0));

        sleep(3000);
        // move out of starting pos
        move(Direction.BACKWARD, 1200, .3);
        // turn left
        move(Direction.RIGHT, 2350);
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