package org.firstinspires.ftc.teamcode.Bluebots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@Autonomous
public class JavaBotdrillardCloseShootAnyColor extends JavaBotdrillard {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        outtakeMotor.setVelocity(1500);
        hoodServo.setPosition(.5);

        move(Direction.FORWARD, 2500, .3);

        sleep(5000);

        liftRightWiper();
        sleep(500);
        liftLeftWiper();
        sleep(500);
        liftRightWiper();
    }
}
