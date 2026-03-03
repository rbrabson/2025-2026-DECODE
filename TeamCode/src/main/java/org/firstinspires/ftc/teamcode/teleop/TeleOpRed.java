package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AllianceConfig;

/**
 * TeleOp for the RED alliance.
 *
 * This is a thin wrapper around {@link MainTeleOp} that sets the alliance to RED.
 * The turret will track toward the red basket at field coordinate (144, 144).
 *
 * All the actual TeleOp logic lives in MainTeleOp -- see that class for controls
 * and documentation.
 */
@TeleOp(name = "TeleOp Red", group = "TeleOp")
public class TeleOpRed extends MainTeleOp {

    public TeleOpRed() {
        super(AllianceConfig.RED);
    }
}
