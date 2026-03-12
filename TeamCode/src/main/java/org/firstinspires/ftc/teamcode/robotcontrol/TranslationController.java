package org.firstinspires.ftc.teamcode.robotcontrol;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.Range;
import com.rbrabson.control.pid.PID;

import org.firstinspires.ftc.teamcode.utils.Vector2;

/**
 * A translation controller that maintains the robot's translation position when the driver is
 * not actively translating. Heading control is handled separately by HeadingController.
 */
public class TranslationController {
    // Translation gains and limits.
    private static final double kP_TRANSLATION = 2.0;
    private static final double kI_TRANSLATION = 0.0;
    private static final double kD_TRANSLATION = 0.08;
    private static final double INPUT_DEADBAND_TRANSLATION = 0.05;
    private static final double MAX_TRANSLATION_CORRECTION = 0.6;

    private final PID xPid = new PID(kP_TRANSLATION, kI_TRANSLATION, kD_TRANSLATION);
    private final PID yPid = new PID(kP_TRANSLATION, kI_TRANSLATION, kD_TRANSLATION);

    private double targetX;
    private double targetY;
    private boolean translationLocked;

    /**
     * Updates the translation controller based on the current driver inputs and robot pose.
     *
     * @param inputX      The driver's x input for translation.
     * @param inputY      The driver's y input for translation.
     * @param currentPose The robot's current pose.
     * @return A Vector2 containing the translation corrections to apply to the robot.
     */
    @NonNull
    public Vector2 update(double inputX, double inputY, @NonNull Pose currentPose) {
        return update(inputX, inputY, currentPose.getX(), currentPose.getY(), currentPose.getHeading());
    }

    /**
     * Updates the translation controller based on the current driver inputs and robot pose.
     *
     * @param inputX            The driver's x input for translation.
     * @param inputY            The driver's y input for translation.
     * @param currentX          The robot's current x position in the global frame.
     * @param currentY          The robot's current y position in the global frame.
     * @param currentHeadingRad The robot's current heading in radians.
     * @return A Vector2 containing the translation corrections to apply to the robot.
     */
    @NonNull
    public Vector2 update(double inputX, double inputY, double currentX, double currentY, double currentHeadingRad) {
        double outX;
        double outY;

        boolean translating = Math.hypot(inputX, inputY) > INPUT_DEADBAND_TRANSLATION;

        if (translating) {
            // If the driver is actively translating, unlock translation and reset PID controllers.
            translationLocked = false;
            targetX = currentX;
            targetY = currentY;

            xPid.reset();
            yPid.reset();

            outX = inputX;
            outY = inputY;
        } else if (!translationLocked) {
            // If the driver is not translating and translation is not locked, lock translation to the current position.
            targetX = currentX;
            targetY = currentY;
            translationLocked = true;

            xPid.reset();
            yPid.reset();

            outX = 0.0;
            outY = 0.0;
        } else {
            // If the driver is not translating and translation is locked, calculate the correction to maintain the target position.
            double outGlobalX = xPid.calculate(targetX, currentX);
            double outGlobalY = yPid.calculate(targetY, currentY);

            double cos = Math.cos(currentHeadingRad);
            double sin = Math.sin(currentHeadingRad);

            double xRobot = (outGlobalX * cos) + (outGlobalY * sin);
            double yRobot = (-outGlobalX * sin) + (outGlobalY * cos);

            outX = Range.clip(xRobot, -MAX_TRANSLATION_CORRECTION, MAX_TRANSLATION_CORRECTION);
            outY = Range.clip(yRobot, -MAX_TRANSLATION_CORRECTION, MAX_TRANSLATION_CORRECTION);
        }

        return new Vector2(outX, outY);
    }

    /**
     * Resets the translation controller, unlocking translation and resetting PID controllers.
     */
    public void reset() {
        translationLocked = false;
        xPid.reset();
        yPid.reset();
    }
}
