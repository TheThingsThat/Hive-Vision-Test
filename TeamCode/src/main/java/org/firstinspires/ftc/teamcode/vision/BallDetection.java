package org.firstinspires.ftc.teamcode.vision;

import com.qualcomm.hardware.limelightvision.LLResultTypes;

import java.util.Locale;

/** One ball reported by the Limelight neural detector, with a floor-position estimate in the robot frame. */
public final class BallDetection {
    public final BallColor color;
    public final String className;
    public final double confidence;

    /** Horizontal angle from the crosshair to the ball, degrees, positive = right. */
    public final double txDeg;

    /** Vertical angle from the crosshair to the ball, degrees, positive = up. */
    public final double tyDeg;

    /** Bounding-box area as a percentage of the image (0 to 100). */
    public final double areaPercent;

    public final double pixelX;
    public final double pixelY;

    /** Estimated ball-center position in the robot frame, inches. NaN when no range estimate is possible. */
    public final double forwardIn;
    public final double lateralIn;

    /** {@code System.nanoTime()} estimate of when the frame was captured. */
    public final long timestampNanos;

    public BallDetection(BallColor color, String className, double confidence, double txDeg, double tyDeg,
                         double areaPercent, double pixelX, double pixelY, double forwardIn, double lateralIn,
                         long timestampNanos) {
        this.color = color;
        this.className = className;
        this.confidence = confidence;
        this.txDeg = txDeg;
        this.tyDeg = tyDeg;
        this.areaPercent = areaPercent;
        this.pixelX = pixelX;
        this.pixelY = pixelY;
        this.forwardIn = forwardIn;
        this.lateralIn = lateralIn;
        this.timestampNanos = timestampNanos;
    }

    public static BallDetection from(LLResultTypes.DetectorResult r, long timestampNanos) {
        BallColor color = BallColor.fromLabel(r.getClassName());
        if (color == null) color = BallColor.fromClassId(r.getClassId());
        double[] floor = CameraGeometry.floorPoint(r.getTargetXDegrees(), r.getTargetYDegrees());
        return new BallDetection(color, r.getClassName(), r.getConfidence(),
                r.getTargetXDegrees(), r.getTargetYDegrees(), r.getTargetArea(),
                r.getTargetXPixels(), r.getTargetYPixels(), floor[0], floor[1], timestampNanos);
    }

    public boolean hasRangeEstimate() {
        return !Double.isNaN(forwardIn) && !Double.isNaN(lateralIn);
    }

    /** Straight-line floor distance from the robot's tracking center to the ball, inches. */
    public double distanceIn() {
        return Math.hypot(forwardIn, lateralIn);
    }

    /** Bearing to the ball in the robot frame, radians, positive = left (counter-clockwise). */
    public double bearingRad() {
        return Math.atan2(lateralIn, forwardIn);
    }

    /** Age of this detection in milliseconds. */
    public double ageMs() {
        return (System.nanoTime() - timestampNanos) / 1e6;
    }

    @Override
    public String toString() {
        if (hasRangeEstimate()) {
            return String.format(Locale.US, "%s conf=%.2f tx=%.1f ty=%.1f fwd=%.1fin lat=%.1fin",
                    color, confidence, txDeg, tyDeg, forwardIn, lateralIn);
        }
        return String.format(Locale.US, "%s conf=%.2f tx=%.1f ty=%.1f (no range)", color, confidence, txDeg, tyDeg);
    }
}
