package org.firstinspires.ftc.teamcode.Bluebots;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class EcstasyOfAutomation_FAR_RED_SHOOT extends EcstasyOfAutomation {

    @Override
    public void runOpMode() throws InterruptedException{
        super.runOpMode();
        waitForStart();

        outtakeMotor.setVelocity(getTargetTicksPerSec(28, 2200));
        intakeMotor.setPower(1);
        hoodServo.setPosition(.82);
        int id = -1;

        /*do {
            List<LLResultTypes.FiducialResult> results = getResultList(limelight);
            List<Integer> ids = new ArrayList<>();
            if (results != null) {
                for (int i = 0; i < results.size(); i++) {
                    int a = results.get(i).getFiducialId();
                    ids.add(a);
                    telemetry.addData("ID", a);
                }
            }
            if (ids.contains(21)) {
                id = 21;
            } else if (ids.contains(22)) {
                id = 22;
            } else if (ids.contains(23)) {
                id = 23;
            }
            telemetry.update();
        } while(id != 21 && id != 22 && id != 23);

        sleep(500);*/
        swivelTurretServo.setPosition(.11);
        //PrincessEyesv4(24, true);

        sleep(5000);

        // there should be two purple balls in the right and one green ball in the left.

        if (id == 21) {
            //GPP
            liftLeftWiper();
            sleep(500);
            liftRightWiper();
            sleep(500);
            liftRightWiper();
        } else if (id == 22){
            //PGP
            liftRightWiper();
            sleep(500);
            liftLeftWiper();
            sleep(500);
            liftRightWiper();
        } else if (id == 23) {
            //PPG
            liftRightWiper();
            sleep(500);
            liftRightWiper();
            sleep(500);
            liftLeftWiper();
        } else {
            liftRightWiper();
            sleep(500);
            liftLeftWiper();
            sleep(500);
            liftRightWiper();
        }

        move(Direction.STRAFE_LEFT, 2800, .3);

        hoodServo.setPosition(.7);

        // move forward and take the balls
        // take in a ball and switch
        move(Direction.FORWARD, 1600, .2);
        gateways.quickSwitch();
        move(Direction.FORWARD, 1700, .2);

        move(Direction.BACKWARD, 3300, .2);

        move(Direction.STRAFE_RIGHT, 3000, .3);

        //PrincessEyesv4(24, true);

        liftLeftWiper();
        sleep(500);
        liftRightWiper();
        sleep(500);
        liftLeftWiper();

        move(Direction.STRAFE_LEFT, 500, 1);

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