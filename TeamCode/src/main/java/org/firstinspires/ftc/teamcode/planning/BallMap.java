package org.firstinspires.ftc.teamcode.planning;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.vision.BallColor;
import org.firstinspires.ftc.teamcode.vision.BallDetection;
import org.firstinspires.ftc.teamcode.vision.HiveLimelight;

import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumSet;
import java.util.Iterator;
import java.util.List;
import java.util.Set;

/**
 * Accumulates Limelight ball detections into a field-frame map so the planner can choose among every ball
 * the robot has seen, not just the ones currently in view.
 * <p>
 * Each detection is projected from the robot frame into the field using the robot pose at (roughly) the
 * time of capture, then merged with any known ball of the same color within {@code MERGE_DISTANCE_IN}.
 */
public class BallMap {
    private final List<FieldBall> balls = new ArrayList<>();
    private int nextId = 1;
    private long lastFrameNanos = Long.MIN_VALUE;
    private int framesIngested;

    /**
     * Ingest the Limelight's latest frame. Safe to call every loop; frames already seen are skipped.
     *
     * @param robotPose the robot's pose (Pedro coordinates) when the frame was captured
     * @return number of detections merged into the map this call
     */
    public int ingest(HiveLimelight limelight, Pose robotPose) {
        long stamp = limelight.latestCaptureNanos();
        if (stamp == lastFrameNanos || stamp == 0) return 0;
        lastFrameNanos = stamp;
        framesIngested++;

        int merged = 0;
        for (BallDetection d : limelight.detections()) {
            if (d.color == null || !d.hasRangeEstimate()) continue;
            if (d.distanceIn() > PlannerConstants.MAX_DETECTION_RANGE_IN) continue;
            Pose field = robotPose.compose(new Pose(d.forwardIn, d.lateralIn, 0));
            observe(d.color, field.x(), field.y(), d.confidence, d.timestampNanos);
            merged++;
        }
        prune();
        return merged;
    }

    /** Add or merge one field-frame observation. */
    public FieldBall observe(BallColor color, double x, double y, double confidence, long seenNanos) {
        FieldBall nearest = null;
        double nearestDist = PlannerConstants.MERGE_DISTANCE_IN;
        Vector2D p = Vector2D.cartesian(x, y);
        for (FieldBall b : balls) {
            if (b.color != color) continue;
            double dist = b.distanceTo(p);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearest = b;
            }
        }
        if (nearest != null) {
            nearest.observe(x, y, confidence, seenNanos);
            return nearest;
        }
        FieldBall b = new FieldBall(nextId++, color, x, y, confidence, seenNanos);
        balls.add(b);
        return b;
    }

    /** Drop balls that have not been seen for {@code BALL_MAX_AGE_S}. */
    public void prune() {
        long now = System.nanoTime();
        Iterator<FieldBall> it = balls.iterator();
        while (it.hasNext()) {
            FieldBall b = it.next();
            if ((now - b.lastSeenNanos()) / 1e9 > PlannerConstants.BALL_MAX_AGE_S) it.remove();
        }
    }

    /** Balls with enough sightings, filtered to the allowed colors. */
    public List<FieldBall> plannable(Set<BallColor> allowed) {
        List<FieldBall> out = new ArrayList<>();
        for (FieldBall b : balls) {
            if (b.observations() >= PlannerConstants.MIN_OBSERVATIONS && allowed.contains(b.color)) out.add(b);
        }
        return out;
    }

    public List<FieldBall> plannable() {
        return plannable(EnumSet.allOf(BallColor.class));
    }

    public List<FieldBall> all() {
        return Collections.unmodifiableList(balls);
    }

    public int size() {
        return balls.size();
    }

    public int framesIngested() {
        return framesIngested;
    }

    public boolean remove(FieldBall ball) {
        return balls.remove(ball);
    }

    public boolean remove(int id) {
        for (Iterator<FieldBall> it = balls.iterator(); it.hasNext(); ) {
            if (it.next().id == id) {
                it.remove();
                return true;
            }
        }
        return false;
    }

    /** Remove every ball within {@code radius} of a point (used to mark a pickup as done). */
    public int removeNear(Vector2D point, double radius) {
        int removed = 0;
        for (Iterator<FieldBall> it = balls.iterator(); it.hasNext(); ) {
            if (it.next().distanceTo(point) <= radius) {
                it.remove();
                removed++;
            }
        }
        return removed;
    }

    public void clear() {
        balls.clear();
    }

    public String summary() {
        int[] counts = new int[BallColor.values().length];
        for (FieldBall b : balls) counts[b.color.ordinal()]++;
        StringBuilder sb = new StringBuilder();
        for (BallColor c : BallColor.values()) sb.append(c.name().charAt(0)).append('=').append(counts[c.ordinal()]).append(' ');
        return sb.toString().trim();
    }
}
