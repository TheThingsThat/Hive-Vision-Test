package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.BallColor;
import org.firstinspires.ftc.teamcode.vision.BallDetection;
import org.firstinspires.ftc.teamcode.vision.HiveLimelight;
import org.firstinspires.ftc.teamcode.vision.LimelightConstants;

/**
 * Bench test for the Hive-Vision ball detector on the Limelight 3A. Needs no drivetrain or odometry.
 * <p>
 * Shows every detection above the confidence threshold, the best per color, the confirmation streak,
 * and the estimated floor position. Put a ball at a measured distance and compare.
 * <ul>
 *   <li>D-pad left / right: previous / next pipeline</li>
 *   <li>Y: save a snapshot on the Limelight (view it in the web UI)</li>
 *   <li>A / B: lower / raise the confidence threshold by 0.05</li>
 * </ul>
 */
@TeleOp(name = "Limelight: Ball Detector Test", group = "Limelight")
public class LimelightBallTest extends OpMode {
    private HiveLimelight limelight;

    @Override
    public void init() {
        limelight = new HiveLimelight(hardwareMap);
        limelight.start(LimelightConstants.PIPELINE_BALL_DETECTOR);
        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Limelight polling. Detections show during init too.");
    }

    @Override
    public void loop() {
        if (gamepad1.dpadLeftWasPressed()) limelight.setPipeline(Math.max(0, limelight.pipeline() - 1));
        if (gamepad1.dpadRightWasPressed()) limelight.setPipeline(Math.min(9, limelight.pipeline() + 1));
        if (gamepad1.yWasPressed()) limelight.snapshot("ball_test");
        if (gamepad1.aWasPressed()) LimelightConstants.MIN_CONFIDENCE = Math.max(0.05, LimelightConstants.MIN_CONFIDENCE - 0.05);
        if (gamepad1.bWasPressed()) LimelightConstants.MIN_CONFIDENCE = Math.min(0.95, LimelightConstants.MIN_CONFIDENCE + 0.05);

        limelight.update();

        limelight.addTelemetry(telemetry);
        telemetry.addData("Min confidence", "%.2f (A/B)", LimelightConstants.MIN_CONFIDENCE);
        telemetry.addData("Detections", limelight.detections().size());
        for (BallDetection d : limelight.detections()) {
            telemetry.addData("  " + d.color, "conf=%.2f tx=%.1f ty=%.1f area=%.2f%% dist=%.1fin bearing=%.0fdeg",
                    d.confidence, d.txDeg, d.tyDeg, d.areaPercent,
                    d.hasRangeEstimate() ? d.distanceIn() : Double.NaN,
                    d.hasRangeEstimate() ? Math.toDegrees(d.bearingRad()) : Double.NaN);
        }
        for (BallColor c : BallColor.values()) {
            BallDetection best = limelight.confirmedBestOf(c);
            if (best != null) telemetry.addData("Confirmed " + c, best.toString());
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }
}
