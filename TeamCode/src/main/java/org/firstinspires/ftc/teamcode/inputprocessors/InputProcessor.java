package org.firstinspires.ftc.teamcode.inputprocessors;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
 * The Input interface defines a common structure for all input processors used in the robot. Each
 * processor must implement the execute method, which takes in the current state of gamepad1 and
 * gamepad2 and performs the necessary actions to control the robot's mechanisms.
 */
public interface InputProcessor {

    /**
     * Executes the input processor's logic based on the current state of gamepad1 and gamepad2.
     * This method is called repeatedly during the teleop period to update the robot's behavior
     * based on driver input.
     *
     * @param gamepad1 The current state of gamepad1.
     * @param gamepad2 The current state of gamepad2.
     */
    void process(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2);
}
