package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;

import java.util.Locale;

/** One AprilTag-derived robot pose from the Limelight, ready to be fed to Pedro's {@code FusionLocalizer}. */
public final class VisionPoseEstimate {
    /** Robot pose in Pedro coordinates (inches, radians). */
    public final Pose pose;

    /** The same pose in the FTC field frame, inches. For telemetry only. */
    public final Pose ftcPoseInches;

    /** {@code System.nanoTime()}-based estimate of the capture time (latency compensated). */
    public final long timestampNanos;

    /** Measurement variance for x (in^2), y (in^2), heading (rad^2). */
    public final Pose variance;

    public final int tagCount;
    public final double avgTagDistanceM;

    /** True when this came from MegaTag2, whose heading is just the IMU heading we supplied. */
    public final boolean megaTag2;

    public VisionPoseEstimate(Pose pose, Pose ftcPoseInches, long timestampNanos, Pose variance,
                              int tagCount, double avgTagDistanceM, boolean megaTag2) {
        this.pose = pose;
        this.ftcPoseInches = ftcPoseInches;
        this.timestampNanos = timestampNanos;
        this.variance = variance;
        this.tagCount = tagCount;
        this.avgTagDistanceM = avgTagDistanceM;
        this.megaTag2 = megaTag2;
    }

    /**
     * The pose to hand to the Kalman filter. MegaTag2 does not measure heading independently, so its
     * heading is reported as NaN, which tells {@code FusionLocalizer} to leave heading untouched.
     */
    public Pose measurementPose() {
        return megaTag2 ? new Pose(pose.x(), pose.y(), Double.NaN) : pose;
    }

    public double ageMs() {
        return (System.nanoTime() - timestampNanos) / 1e6;
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "%s (%.1f, %.1f, %.0f deg) tags=%d avg=%.2fm age=%.0fms",
                megaTag2 ? "MT2" : "MT1", pose.x(), pose.y(), Math.toDegrees(pose.heading()),
                tagCount, avgTagDistanceM, ageMs());
    }
}
