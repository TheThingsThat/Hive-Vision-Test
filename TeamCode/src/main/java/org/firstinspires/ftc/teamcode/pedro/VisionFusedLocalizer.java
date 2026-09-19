package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.localization.FusionLocalizer;
import com.pedropathing.localization.Localizer;
import com.pedropathing.localization.MotionState;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.vision.HiveLimelight;
import org.firstinspires.ftc.teamcode.vision.LimelightConstants;
import org.firstinspires.ftc.teamcode.vision.VisionPoseEstimate;

import java.util.Map;

/**
 * A Pedro {@link Localizer} that wraps dead-reckoning odometry in Pedro's {@link FusionLocalizer}
 * (a latency-compensated Kalman filter) and feeds it Limelight AprilTag pose estimates every loop.
 * <p>
 * Drop it into {@code new Follower(localizer, drivetrain, algorithm)} and the follower's pose will be
 * odometry between tags and pulled back onto the tags whenever they are visible. The odometry-only pose
 * stays available through {@link #odometryPose()} so the two can be compared in telemetry.
 */
public class VisionFusedLocalizer implements Localizer {
    private final Localizer odometry;
    private final FusionLocalizer fusion;
    private final HiveLimelight limelight;

    private boolean visionEnabled = true;
    private int accepted;
    private int rejected;
    private VisionPoseEstimate lastEstimate;
    private VisionPoseEstimate lastApplied;

    public VisionFusedLocalizer(Localizer odometry, HiveLimelight limelight) {
        this.odometry = odometry;
        this.limelight = limelight;
        this.fusion = new FusionLocalizer(
                odometry,
                new Pose(LimelightConstants.INITIAL_VARIANCE_XY, LimelightConstants.INITIAL_VARIANCE_XY,
                        LimelightConstants.INITIAL_VARIANCE_HEADING),
                new Pose(LimelightConstants.PROCESS_VARIANCE_XY, LimelightConstants.PROCESS_VARIANCE_XY,
                        LimelightConstants.PROCESS_VARIANCE_HEADING),
                new Pose(LimelightConstants.VISION_VARIANCE_XY, LimelightConstants.VISION_VARIANCE_XY,
                        LimelightConstants.VISION_VARIANCE_HEADING),
                LimelightConstants.FUSION_HISTORY_SIZE);
    }

    // ------------------------------------------------------------------ Localizer

    @Override
    public void update() {
        // Everything the fusion filter records this loop is stamped at or after this instant, so a vision
        // measurement must never be newer than it or FusionLocalizer will drop it.
        long beforeUpdate = System.nanoTime();
        fusion.update();

        limelight.updateRobotHeading(fusion.pose().heading());
        limelight.update();

        VisionPoseEstimate estimate = limelight.takeNewPoseEstimate();
        if (estimate == null) return;
        lastEstimate = estimate;
        if (!visionEnabled) return;

        if (!accept(estimate)) {
            rejected++;
            return;
        }

        long timestamp = Math.min(estimate.timestampNanos, beforeUpdate);
        fusion.addMeasurement(estimate.measurementPose(), timestamp, estimate.variance);
        lastApplied = estimate;
        accepted++;
    }

    private boolean accept(VisionPoseEstimate estimate) {
        double maxJump = LimelightConstants.MAX_SINGLE_TAG_JUMP_IN;
        if (maxJump > 0 && estimate.tagCount < 2 && estimate.pose.distance(fusion.pose()) > maxJump) {
            return false;
        }
        return true;
    }

    @Override
    public MotionState state() {
        return fusion.state();
    }

    @Override
    public void setPose(Pose pose) {
        fusion.setPose(pose);
    }

    @Override
    public void reset() {
        fusion.reset();
    }

    @Override
    public Map<String, Object> debug() {
        Map<String, Object> map = Localizer.super.debug();
        map.put("odometryPose", odometry.pose());
        map.put("visionEnabled", visionEnabled);
        map.put("visionAccepted", accepted);
        map.put("visionRejected", rejected);
        map.put("lastVision", lastEstimate);
        return map;
    }

    // ------------------------------------------------------------------ extras

    /** Raw dead-reckoning pose, unaffected by vision. */
    public Pose odometryPose() {
        return odometry.pose();
    }

    public HiveLimelight limelight() {
        return limelight;
    }

    /** Enables or disables applying vision measurements (heading is still pushed for MegaTag2). */
    public void setVisionEnabled(boolean enabled) {
        visionEnabled = enabled;
    }

    public boolean visionEnabled() {
        return visionEnabled;
    }

    /**
     * Hard-sets the pose to the most recent AprilTag estimate, bypassing the filter and the jump gate.
     * Useful at the start of TeleOp or whenever odometry is known to be wrong.
     *
     * @return true if an estimate was available and applied
     */
    public boolean snapToVision() {
        if (lastEstimate == null) return false;
        Pose target = lastEstimate.megaTag2
                ? new Pose(lastEstimate.pose.x(), lastEstimate.pose.y(), fusion.pose().heading())
                : lastEstimate.pose;
        fusion.setPose(target);
        return true;
    }

    public VisionPoseEstimate lastEstimate() {
        return lastEstimate;
    }

    public VisionPoseEstimate lastApplied() {
        return lastApplied;
    }

    public int accepted() {
        return accepted;
    }

    public int rejected() {
        return rejected;
    }
}
