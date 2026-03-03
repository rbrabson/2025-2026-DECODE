package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceConfig;

/**
 * TeleOp for the BLUE alliance.
 *
 * This is a thin wrapper around {@link MainTeleOp} that sets the alliance to BLUE.
 * The turret will track toward the blue basket at field coordinate (0, 144).
 *
 * All the actual TeleOp logic lives in MainTeleOp -- see that class for controls
 * and documentation.
 *
 * TODO: [Student] Why do we need separate TeleOp classes for each alliance
 *       instead of just one? Hint: the FTC Driver Station needs distinct
 *       OpMode names to appear in the selection list.
 */
@TeleOp(name = "TeleOp Blue", group = "TeleOp")
public class TeleOpBlue extends MainTeleOp {

    public TeleOpBlue() {
        super(AllianceConfig.BLUE);
    }
}
