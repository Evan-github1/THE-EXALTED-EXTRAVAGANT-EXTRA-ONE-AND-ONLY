package org.firstinspires.ftc.teamcode.Bluebots;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotFunctions.Movable;

@Autonomous
public class JavBotdrillardMoveForward2Sec extends JavaBotdrillard {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();

        waitForStart();

        move(Direction.FORWARD, 2000, .5);

    }
}
