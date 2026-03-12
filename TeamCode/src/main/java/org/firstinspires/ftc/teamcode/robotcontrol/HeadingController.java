package org.firstinspires.ftc.teamcode.robotcontrol;

import com.qualcomm.robotcore.util.Range;
import com.rbrabson.control.pid.PID;

import org.firstinspires.ftc.teamcode.utils.MathEx;

/**
 * Maintains heading whenever the driver is not actively commanding turn. Translation control is
 * handled separately by TranslationController.
 */
public class HeadingController {
    // Tunable PID constants for heading control; adjust based on the robot's response characteristics.
    private static final double kP = 1.6;
    private static final double kI = 0.0;
    private static final double kD = 0.08;

    // Driver and hold behavior; adjust as needed based on driver preference and robot response.
    private static final double INPUT_DEADBAND_TURN = 0.06;
    private static final double HEADING_ERROR_TOLERANCE_RAD = Math.toRadians(1.0);
    private static final double MAX_HEADING_CORRECTION = 0.45;

    private final PID headingPid = new PID(kP, kI, kD);

    private double targetHeading;
    private boolean locked;

    /**
     * Updates the heading controller based on driver input and current heading. If the driver is
     * commanding a turn, the controller will unlock and pass through the driver input. If the
     * driver is not commanding a turn, the controller will lock onto the current heading and
     * apply corrections to maintain that heading.
     *
     * @param turnInput      Driver turn input.
     * @param currentHeading Current robot heading in radians.
     * @return Turn command (driver turn or heading hold correction).
     */
    public double update(double turnInput, double currentHeading) {
        boolean driverTurning = Math.abs(turnInput) > INPUT_DEADBAND_TURN;

        // Driver turn overrides heading hold.
        if (driverTurning) {
            locked = false;
            targetHeading = currentHeading;
            headingPid.reset();
            return turnInput;
        }

        // No turn command: lock once, then hold.
        if (!locked) {
            targetHeading = currentHeading;
            locked = true;
            headingPid.reset();
        }

        double error = MathEx.angleWrap(targetHeading - currentHeading);

        // Avoid micro-corrections near zero error.
        if (Math.abs(error) <= HEADING_ERROR_TOLERANCE_RAD) {
            return 0;
        }

        double correction = headingPid.calculate(0.0, -error);
        return Range.clip(correction, -MAX_HEADING_CORRECTION, MAX_HEADING_CORRECTION);
    }

    /**
     * Resets the heading controller, unlocking it and clearing PID state.
     */
    public void reset() {
        locked = false;
        headingPid.reset();
    }
}
