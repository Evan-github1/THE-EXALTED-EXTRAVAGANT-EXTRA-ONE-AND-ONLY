package org.firstinspires.ftc.teamcode.RobotFunctions;

import com.qualcomm.robotcore.util.ElapsedTime;

public class HeadingPID {
    private double kP;
    private double kD;
    private double kStatic;

    private double previousError = 0;
    private ElapsedTime timer;

    /**
     * @param kP      Proportional gain (The main muscle)
     * @param kD      Derivative gain (The brakes)
     * @param kStatic Minimum power required to move the robot (Anti-creep)
     */
    public HeadingPID(double kP, double kD, double kStatic) {
        this.kP = kP;
        this.kD = kD;
        this.kStatic = kStatic;
        this.timer = new ElapsedTime();
    }

    /**
     * Calculates the motor power needed to turn to the target angle.
     * * @param targetAngle  The angle you want to face (in radians)
     * @param currentAngle Your current odometry heading (in radians)
     * @return Motor turn power (between -1.0 and 1.0)
     */
    public double calculate(double targetAngle, double currentAngle) {
        // 1. Calculate the raw error
        double error = targetAngle - currentAngle;

        // 2. Angle Wrap: Force the error to be the shortest path (between -PI and PI)
        while (error > Math.PI) error -= 2 * Math.PI;
        while (error < -Math.PI) error += 2 * Math.PI;

        // 3. Calculate time elapsed since last loop for the Derivative
        double dt = timer.seconds();
        timer.reset();

        // 4. Proportional Power
        double pPower = error * kP;

        // 5. Derivative Power (Rate of change of error)
        double dPower = 0;
        if (dt > 0) {
            dPower = ((error - previousError) / dt) * kD;
        }
        previousError = error;

        // 6. Static Friction Feedforward (Anti-Creep)
        double fPower = 0;
        // Only apply kStatic if we are more than ~1 degree (0.017 radians) off target
        if (Math.abs(error) > 0.017) {
            fPower = Math.copySign(kStatic, error);
        }

        // Combine all terms
        double totalPower = pPower + dPower + fPower;

        // Clamp the output so we don't send insane values to the motors
        return Math.max(-1.0, Math.min(1.0, totalPower));
    }

    // Optional: Allow tuning on the fly via dashboard or gamepad
    public void setCoefficients(double kP, double kD, double kStatic) {
        this.kP = kP;
        this.kD = kD;
        this.kStatic = kStatic;
    }
}
