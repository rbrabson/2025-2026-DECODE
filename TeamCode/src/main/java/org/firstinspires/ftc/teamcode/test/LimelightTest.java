package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * Tests the Limelight 3A camera for AprilTag detection.
 *
 * This test helps verify that:
 * - The Limelight is connected and responding
 * - AprilTags can be detected on pipeline 0
 * - The motif can be identified from tag IDs (21=GPP, 22=PGP, 23=PPG)
 * - Pipeline switching works correctly
 * - The tx/ty targeting values are correct
 *
 * CONTROLS:
 *   DPAD UP    - Switch to pipeline 0 (AprilTag fiducial detection)
 *   DPAD DOWN  - Switch to pipeline 1 (tracking mode)
 *   A          - Switch to pipeline 2
 *   B          - Switch to pipeline 3
 *
 * TODO: [Student] Hold the robot in front of each AprilTag and verify:
 *       - Tag 21 shows motif "GPP"
 *       - Tag 22 shows motif "PGP"
 *       - Tag 23 shows motif "PPG"
 *       Does tx=0 mean the tag is centered in the camera view?
 */
public class LimelightTest implements BaseTest {

    private Limelight3A limelight;
    private int currentPipeline = 0;

    @Override
    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop(Telemetry telemetry, Gamepad gamepad, HardwareMap hardwareMap) {
        // Pipeline switching
        if (gamepad.dpad_up) { currentPipeline = 0; limelight.pipelineSwitch(0); }
        if (gamepad.dpad_down) { currentPipeline = 1; limelight.pipelineSwitch(1); }
        if (gamepad.a) { currentPipeline = 2; limelight.pipelineSwitch(2); }
        if (gamepad.b) { currentPipeline = 3; limelight.pipelineSwitch(3); }

        // Get latest result
        LLResult result = limelight.getLatestResult();

        telemetry.addLine("=== Limelight Test ===");
        telemetry.addData("Pipeline", currentPipeline);
        telemetry.addLine("DPad Up=P0  DPad Down=P1  A=P2  B=P3");
        telemetry.addLine("");

        if (result != null && result.isValid()) {
            telemetry.addData("Status", "VALID RESULT");
            telemetry.addData("tx (horizontal error)", String.format("%.2f deg", result.getTx()));
            telemetry.addData("ty (vertical error)", String.format("%.2f deg", result.getTy()));
            telemetry.addData("ta (target area)", String.format("%.2f%%", result.getTa()));

            // Show fiducial results (AprilTags)
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                telemetry.addLine("");
                telemetry.addLine("--- AprilTags Detected ---");
                for (LLResultTypes.FiducialResult fid : fiducials) {
                    int tagId = fid.getFiducialId();
                    String motif;
                    if (tagId == 21) motif = "GPP";
                    else if (tagId == 22) motif = "PGP";
                    else if (tagId == 23) motif = "PPG";
                    else motif = "Unknown";

                    telemetry.addData("Tag ID " + tagId, "Motif: " + motif);
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }
        } else {
            telemetry.addData("Status", "NO VALID RESULT");
            telemetry.addLine("Check Limelight connection and pipeline");
        }
    }

    @Override
    public void stop() {
        // Limelight continues running (safe to leave on)
    }
}
