package org.firstinspires.ftc.teamcode.robotcontrol;

import androidx.annotation.NonNull;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.utils.Vector2;

import java.util.Objects;

/**
 * MecanumDriveController combines a HeadingController and a TranslationController to provide a unified
 * interface for controlling both aspects of robot movement. It takes driver inputs for translation
 * and turning, along with the current robot pose, and produces a combined output for
 * driving the robot.
 */
public class MecanumDriveController {
    private final HeadingController headingController;
    private final TranslationController translationController;

    /**
     * Default constructor using default heading and translation controllers.
     */
    public MecanumDriveController() {
        this(new HeadingController(), new TranslationController());
    }

    /**
     * Constructor with custom heading and translation controllers.
     *
     * @param headingController     Heading controller to use.
     * @param translationController Translation controller to use.
     */
    public MecanumDriveController(@NonNull HeadingController headingController, @NonNull TranslationController translationController) {
        this.headingController = Objects.requireNonNull(headingController, "headingController");
        this.translationController = Objects.requireNonNull(translationController, "translationController");
    }

    /**
     * Update using a Pose.
     *
     * @param inputX      Driver translation X input.
     * @param inputY      Driver translation Y input.
     * @param turnInput   Driver turn input.
     * @param currentPose Current robot pose.
     * @return Combined drive output.
     */
    @NonNull
    public DriveOutput update(double inputX, double inputY, double turnInput, @NonNull Pose currentPose) {
        return update(inputX, inputY, turnInput, currentPose.getX(), currentPose.getY(), currentPose.getHeading());
    }

    /**
     * Update using raw position and heading.
     *
     * @param inputX            Driver translation X input.
     * @param inputY            Driver translation Y input.
     * @param turnInput         Driver turn input.
     * @param currentX          Current global X.
     * @param currentY          Current global Y.
     * @param currentHeadingRad Current heading in radians.
     * @return Combined drive output.
     */
    @NonNull
    public DriveOutput update(double inputX, double inputY, double turnInput,
                              double currentX, double currentY, double currentHeadingRad) {
        Vector2 translation = translationController.update(inputX, inputY, currentX, currentY, currentHeadingRad);
        double turn = headingController.update(turnInput, currentHeadingRad);
        return new DriveOutput(translation, turn);
    }

    /**
     * Reset the controller to its initial state.
     */
    public void reset() {
        translationController.reset();
        headingController.reset();
    }

    /*
     * Container for combined drive output.
     */
    public static final class DriveOutput {
        private final double x;
        private final double y;
        private final double turn;

        /**
         * Constructor for DriveOutput with translation and turn components.
         *
         * @param translation Translation vector output from the translation controller.
         * @param turn        Turn output from the heading controller.
         */
        public DriveOutput(@NonNull Vector2 translation, double turn) {
            this(translation.x, translation.y, turn);
        }

        /**
         * Constructor for DriveOutput with raw components.
         *
         * @param x    X component of the translation output.
         * @param y    Y component of the translation output.
         * @param turn Turn output from the heading controller.
         */
        public DriveOutput(double x, double y, double turn) {
            this.x = x;
            this.y = y;
            this.turn = turn;
        }

        /**
         * Get the X component of the translation output.
         *
         * @return X component of the translation.
         */
        public double getX() {
            return x;
        }

        /**
         * Get the Y component of the translation output.
         *
         * @return Y component of the translation.
         */
        public double getY() {
            return y;
        }

        /**
         * Get the turn output.
         *
         * @return Turn output.
         */
        public double getTurn() {
            return turn;
        }
    }
}
