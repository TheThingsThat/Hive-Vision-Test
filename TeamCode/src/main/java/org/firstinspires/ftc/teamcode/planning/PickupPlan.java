package org.firstinspires.ftc.teamcode.planning;

import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

/** The result of {@link PickupPlanner#plan}: which balls, in what order, how to approach each, and the Pedro path. */
public final class PickupPlan {
    /** Balls in pickup order. */
    public final List<FieldBall> order;

    /** Robot center pose at the moment each ball reaches the capture point. */
    public final List<Pose> pickupPoses;

    /** Direction of travel at each pickup, radians. */
    public final double[] approachHeadings;

    /** Direction the robot first moves in from its starting pose, radians. */
    public final double startTangent;

    /** Pedro path that drives the whole plan. */
    public final Path path;

    /** Estimated time to drive the path with the planner's motion model, seconds. */
    public final double estimatedSeconds;

    public final long planningMillis;
    public final long sequencesEvaluated;

    PickupPlan(List<FieldBall> order, List<Pose> pickupPoses, double[] approachHeadings, double startTangent,
               Path path, double estimatedSeconds, long planningMillis, long sequencesEvaluated) {
        this.order = Collections.unmodifiableList(order);
        this.pickupPoses = Collections.unmodifiableList(pickupPoses);
        this.approachHeadings = approachHeadings;
        this.startTangent = startTangent;
        this.path = path;
        this.estimatedSeconds = estimatedSeconds;
        this.planningMillis = planningMillis;
        this.sequencesEvaluated = sequencesEvaluated;
    }

    public int size() {
        return order.size();
    }

    public String summary() {
        StringBuilder sb = new StringBuilder();
        sb.append(String.format(Locale.US, "%d balls, est %.2fs, planned in %dms (%d sequences)",
                order.size(), estimatedSeconds, planningMillis, sequencesEvaluated));
        for (int i = 0; i < order.size(); i++) {
            FieldBall b = order.get(i);
            sb.append(String.format(Locale.US, "\n %d. %s #%d at (%.0f,%.0f) approach %.0f deg",
                    i + 1, b.color, b.id, b.x(), b.y(), Math.toDegrees(approachHeadings[i])));
        }
        return sb.toString();
    }
}
