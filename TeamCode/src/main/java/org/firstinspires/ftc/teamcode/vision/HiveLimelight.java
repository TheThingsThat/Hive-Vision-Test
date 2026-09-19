package org.firstinspires.ftc.teamcode.vision;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Robot-side wrapper around the Limelight 3A for the Hive-Vision stack.
 * <p>
 * Two jobs:
 * <ol>
 *   <li><b>Ball detection</b> from the Hive-Vision SSD-MobileNetV2 neural-detector pipeline. Mirrors the
 *       {@code bestOf(BallColor)} API of Hive-Vision's Control Hub pipelines and adds the multi-frame
 *       confirmation that Hive-Vision recommends before acting on a detection.</li>
 *   <li><b>AprilTag localization</b> (MegaTag1 / MegaTag2 botpose) converted into Pedro coordinates as a
 *       {@link VisionPoseEstimate}, which {@code VisionFusedLocalizer} feeds into Pedro's Kalman filter.</li>
 * </ol>
 * Only one Limelight pipeline is active at a time, so switch with {@link #setPipeline(int)} depending on
 * what the OpMode needs.
 * <p>
 * Call {@link #update()} once per loop (the fused localizer does this for you), then read the accessors.
 */
public class HiveLimelight {
    private final Limelight3A limelight;

    private LLResult latest;
    private long latestCaptureNanos;
    private double lastLimelightTimestamp = Double.NaN;
    private long lastControlHubStamp = Long.MIN_VALUE;
    private int pipeline = -1;
    private boolean running;

    private final List<BallDetection> detections = new ArrayList<>();
    private final Map<BallColor, BallDetection> bestByColor = new EnumMap<>(BallColor.class);
    private final Map<BallColor, Integer> streak = new EnumMap<>(BallColor.class);

    private VisionPoseEstimate poseEstimate;
    private boolean poseEstimateConsumed = true;
    private long lastHeadingPushNanos = 0;
    private int botposeRejections;

    public HiveLimelight(HardwareMap hardwareMap) {
        this(hardwareMap, LimelightConstants.HARDWARE_NAME);
    }

    public HiveLimelight(HardwareMap hardwareMap, String hardwareName) {
        limelight = hardwareMap.get(Limelight3A.class, hardwareName);
        limelight.setPollRateHz(LimelightConstants.POLL_RATE_HZ);
        for (BallColor c : BallColor.values()) streak.put(c, 0);
    }

    // ------------------------------------------------------------------ lifecycle

    /** Starts polling on the ball-detector pipeline. Nothing is returned by {@link #update()} until this is called. */
    public void start() {
        start(LimelightConstants.PIPELINE_BALL_DETECTOR);
    }

    public void start(int pipelineIndex) {
        setPipeline(pipelineIndex);
        limelight.start();
        running = true;
    }

    public void pause() {
        limelight.pause();
        running = false;
    }

    public void stop() {
        limelight.stop();
        running = false;
    }

    public boolean isRunning() {
        return running && limelight.isRunning();
    }

    public boolean isConnected() {
        return limelight.isConnected();
    }

    /** Switches pipelines (no-op if already active) and clears cached detections. */
    public void setPipeline(int pipelineIndex) {
        if (pipelineIndex == pipeline) return;
        limelight.pipelineSwitch(pipelineIndex);
        pipeline = pipelineIndex;
        clearFrame();
        poseEstimate = null;
        poseEstimateConsumed = true;
    }

    public int pipeline() {
        return pipeline;
    }

    public boolean onBallPipeline() {
        return pipeline == LimelightConstants.PIPELINE_BALL_DETECTOR;
    }

    public boolean onAprilTagPipeline() {
        return pipeline == LimelightConstants.PIPELINE_APRILTAG;
    }

    public LLStatus status() {
        return limelight.getStatus();
    }

    /** Raw SDK result from the last {@link #update()}, or null. */
    public LLResult latestResult() {
        return latest;
    }

    public Limelight3A device() {
        return limelight;
    }

    public boolean snapshot(String name) {
        return limelight.captureSnapshot(name);
    }

    // ------------------------------------------------------------------ MegaTag2 heading

    /**
     * Pushes the robot's current heading to the Limelight so MegaTag2 can solve with a known yaw.
     * Call every loop with the Pedro heading (radians). The fused localizer does this automatically.
     */
    public void updateRobotHeading(double pedroHeadingRad) {
        limelight.updateRobotOrientation(FieldCoordinates.pedroHeadingToFtcYawDegrees(pedroHeadingRad));
        lastHeadingPushNanos = System.nanoTime();
    }

    private boolean headingIsFresh() {
        return lastHeadingPushNanos != 0
                && (System.nanoTime() - lastHeadingPushNanos) < LimelightConstants.MAX_HEADING_AGE_MS * 1_000_000L;
    }

    // ------------------------------------------------------------------ per-loop update

    /**
     * Pulls the latest result and refreshes detections / pose estimate.
     *
     * @return true if a new Limelight frame was processed this call.
     */
    public boolean update() {
        LLResult result = limelight.getLatestResult();
        if (result == null) {
            latest = null;
            clearFrame();
            return false;
        }

        // If the Limelight has gone quiet (unplugged, rebooting), do not keep serving old detections.
        long staleness = result.getStaleness();
        if (staleness > LimelightConstants.MAX_RESULT_AGE_MS) {
            latest = result;
            clearFrame();
            return false;
        }

        // Detect a genuinely new frame. The Limelight's own timestamp (seconds) changes per frame; the
        // Control Hub stamp changes per poll, so it is only a fallback.
        double llTs = result.getTimestamp();
        long hubStamp = result.getControlHubTimeStamp();
        boolean newFrame = llTs != 0 ? llTs != lastLimelightTimestamp : hubStamp != lastControlHubStamp;
        if (!newFrame) {
            return false;
        }
        lastLimelightTimestamp = llTs;
        lastControlHubStamp = hubStamp;
        latest = result;

        // The SDK's Control Hub stamp is wall-clock (currentTimeMillis) based, and Pedro's fusion history is
        // keyed on System.nanoTime(). So build the capture time relatively: now, minus how long ago the hub
        // received it, minus the Limelight's own capture + targeting latency.
        double latencyMs = staleness + result.getCaptureLatency() + result.getTargetingLatency();
        latestCaptureNanos = System.nanoTime() - (long) (latencyMs * 1e6);

        parseBalls(result, latestCaptureNanos);
        parseBotpose(result, latestCaptureNanos);
        return true;
    }

    private void clearFrame() {
        detections.clear();
        bestByColor.clear();
        for (BallColor c : BallColor.values()) streak.put(c, 0);
    }

    // ------------------------------------------------------------------ balls

    private void parseBalls(LLResult result, long captureNanos) {
        detections.clear();
        bestByColor.clear();

        List<LLResultTypes.DetectorResult> raw = result.getDetectorResults();
        if (raw != null) {
            for (LLResultTypes.DetectorResult r : raw) {
                if (r.getConfidence() < LimelightConstants.MIN_CONFIDENCE) continue;
                BallDetection d = BallDetection.from(r, captureNanos);
                if (d.color == null) continue;
                detections.add(d);
                BallDetection best = bestByColor.get(d.color);
                if (best == null || isBetter(d, best)) bestByColor.put(d.color, d);
            }
        }

        for (BallColor c : BallColor.values()) {
            streak.put(c, bestByColor.containsKey(c) ? streak.get(c) + 1 : 0);
        }
    }

    /** Ranks detections: higher confidence wins, ties broken by larger (closer) area. */
    private static boolean isBetter(BallDetection a, BallDetection b) {
        if (Math.abs(a.confidence - b.confidence) > 0.05) return a.confidence > b.confidence;
        return a.areaPercent > b.areaPercent;
    }

    /** Every detection above the confidence threshold in the most recent frame. */
    public List<BallDetection> detections() {
        return Collections.unmodifiableList(detections);
    }

    /** Best detection of the color in the most recent frame, or null. Same contract as Hive-Vision's pipelines. */
    public BallDetection bestOf(BallColor color) {
        return bestByColor.get(color);
    }

    /** Closest detection of the color (by estimated range) in the most recent frame, or null. */
    public BallDetection closestOf(BallColor color) {
        BallDetection closest = null;
        for (BallDetection d : detections) {
            if (d.color != color || !d.hasRangeEstimate()) continue;
            if (closest == null || d.distanceIn() < closest.distanceIn()) closest = d;
        }
        return closest;
    }

    /** Number of consecutive frames in which the color has been detected. */
    public int streak(BallColor color) {
        return streak.get(color);
    }

    /** True once the color has been seen for {@code CONFIRM_FRAMES} consecutive frames. */
    public boolean isConfirmed(BallColor color) {
        return streak(color) >= LimelightConstants.CONFIRM_FRAMES;
    }

    /** {@link #bestOf(BallColor)} gated by {@link #isConfirmed(BallColor)}. */
    public BallDetection confirmedBestOf(BallColor color) {
        return isConfirmed(color) ? bestOf(color) : null;
    }

    // ------------------------------------------------------------------ AprilTags

    private void parseBotpose(LLResult result, long captureNanos) {
        int tagCount = result.getBotposeTagCount();
        if (tagCount < LimelightConstants.MIN_TAG_COUNT || tagCount <= 0) return;

        double avgDist = result.getBotposeAvgDist();
        if (avgDist > LimelightConstants.MAX_TAG_AVG_DISTANCE_M) {
            botposeRejections++;
            return;
        }

        // Prefer MegaTag2 when we have pushed a fresh heading; fall back to MegaTag1 if MT2 is missing or
        // all zeros (older firmware, or no solution yet).
        boolean useMt2 = LimelightConstants.USE_MEGATAG2 && headingIsFresh();
        Pose ftcIn = null;
        Pose3D p3 = null;
        if (useMt2) {
            p3 = result.getBotpose_MT2();
            ftcIn = usablePose(p3);
            if (ftcIn == null) useMt2 = false;
        }
        if (!useMt2) {
            p3 = result.getBotpose();
            ftcIn = usablePose(p3);
        }
        if (ftcIn == null) return;

        Pose pedro = FieldCoordinates.pose3DToPedro(p3);
        if (pedro == null || !FieldCoordinates.isOnField(pedro, LimelightConstants.FIELD_MARGIN_IN)) {
            botposeRejections++;
            return;
        }

        double[] stddev = useMt2 ? result.getStddevMt2() : result.getStddevMt1();
        Pose variance = varianceFrom(stddev, tagCount, avgDist);

        poseEstimate = new VisionPoseEstimate(pedro, ftcIn, captureNanos, variance, tagCount, avgDist, useMt2);
        poseEstimateConsumed = false;
    }

    /** The botpose as an FTC-frame pose in inches, or null if it is missing, NaN, or the all-zero "no solution". */
    private static Pose usablePose(Pose3D p3) {
        Pose ftcIn = FieldCoordinates.pose3DToFtcInches(p3);
        if (ftcIn == null) return null;
        if (Double.isNaN(ftcIn.x()) || Double.isNaN(ftcIn.y()) || Double.isNaN(ftcIn.heading())) return null;
        if (ftcIn.x() == 0 && ftcIn.y() == 0 && ftcIn.heading() == 0) return null;
        return ftcIn;
    }

    /**
     * Builds a measurement variance from the Limelight's reported standard deviations
     * ([x, y, z, roll, pitch, yaw] in meters / degrees) or, when absent, from the defaults scaled by
     * tag distance and count.
     */
    private static Pose varianceFrom(double[] stddev, int tagCount, double avgDistM) {
        final double inPerM = 39.3701;
        if (stddev != null && stddev.length >= 6 && stddev[0] > 0 && stddev[1] > 0) {
            double vx = Math.pow(stddev[0] * inPerM, 2);
            double vy = Math.pow(stddev[1] * inPerM, 2);
            double vh = stddev[5] > 0 ? Math.pow(Math.toRadians(stddev[5]), 2) : LimelightConstants.VISION_VARIANCE_HEADING;
            return new Pose(vx, vy, vh);
        }
        double scale = Math.pow(Math.max(1.0, avgDistM), 2) / Math.max(1, tagCount);
        return new Pose(LimelightConstants.VISION_VARIANCE_XY * scale,
                LimelightConstants.VISION_VARIANCE_XY * scale,
                LimelightConstants.VISION_VARIANCE_HEADING * scale);
    }

    /** Most recent accepted AprilTag pose estimate (may already have been consumed), or null. */
    public VisionPoseEstimate poseEstimate() {
        return poseEstimate;
    }

    /** Returns the latest pose estimate once, then null until a newer one arrives. Used by the fused localizer. */
    public VisionPoseEstimate takeNewPoseEstimate() {
        if (poseEstimateConsumed) return null;
        poseEstimateConsumed = true;
        return poseEstimate;
    }

    public int botposeRejections() {
        return botposeRejections;
    }

    // ------------------------------------------------------------------ telemetry

    public void addTelemetry(Telemetry telemetry) {
        LLStatus s = limelight.getStatus();
        if (s != null) {
            telemetry.addData("LL status", "%s fps=%.0f temp=%.0fC cpu=%.0f%% pipe=%d(%s)",
                    s.getName(), s.getFps(), s.getTemp(), s.getCpu(), s.getPipelineIndex(), s.getPipelineType());
        }
        telemetry.addData("LL link", "connected=%b running=%b pipeline=%d", isConnected(), isRunning(), pipeline);
        if (latest == null) {
            telemetry.addData("LL result", "none");
            return;
        }
        telemetry.addData("LL result", "valid=%b stale=%dms latency=%.0fms",
                latest.isValid(), latest.getStaleness(), latest.getCaptureLatency() + latest.getTargetingLatency());
        for (BallColor c : BallColor.values()) {
            BallDetection d = bestOf(c);
            telemetry.addData(c.name(), d == null ? "-" : String.format(Locale.US, "%s streak=%d%s",
                    d, streak(c), isConfirmed(c) ? " CONFIRMED" : ""));
        }
        telemetry.addData("Botpose", poseEstimate == null ? "-" : poseEstimate.toString());
    }
}
