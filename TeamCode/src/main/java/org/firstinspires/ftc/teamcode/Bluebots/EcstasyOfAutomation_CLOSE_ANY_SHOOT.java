package org.firstinspires.ftc.teamcode.Bluebots;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class EcstasyOfAutomation_CLOSE_ANY_SHOOT extends EcstasyOfAutomation {

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        waitForStart();

        outtakeMotor.setVelocity(getTargetTicksPerSec(28, 1600));
        intakeMotor.setPower(1);
        hoodServo.setPosition(0);
        int id = -1;

        move(Direction.FORWARD, 4500, .2);

        sleep(2000);

        liftRightWiper();
        sleep(1500);
        liftLeftWiper();
        sleep(500);
        liftRightWiper();

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