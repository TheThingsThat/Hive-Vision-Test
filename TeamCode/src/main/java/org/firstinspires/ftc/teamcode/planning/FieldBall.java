package org.firstinspires.ftc.teamcode.planning;

import com.pedropathing.math.Vector2D;

import org.firstinspires.ftc.teamcode.vision.BallColor;

import java.util.Locale;

/** A ball tracked in Pedro field coordinates (inches), accumulated over several Limelight frames. */
public final class FieldBall {
    public final int id;
    public final BallColor color;
    private double x;
    private double y;
    private int observations;
    private double confidence;
    private long firstSeenNanos;
    private long lastSeenNanos;

    public FieldBall(int id, BallColor color, double x, double y, double confidence, long seenNanos) {
        this.id = id;
        this.color = color;
        this.x = x;
        this.y = y;
        this.confidence = confidence;
        this.observations = 1;
        this.firstSeenNanos = seenNanos;
        this.lastSeenNanos = seenNanos;
    }

    /** Construct directly at a field position (for tests or manually entered balls). */
    public static FieldBall at(int id, BallColor color, double x, double y) {
        FieldBall b = new FieldBall(id, color, x, y, 1.0, System.nanoTime());
        b.observations = Integer.MAX_VALUE / 2;
        return b;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public Vector2D position() {
        return Vector2D.cartesian(x, y);
    }

    public int observations() {
        return observations;
    }

    public double confidence() {
        return confidence;
    }

    public long lastSeenNanos() {
        return lastSeenNanos;
    }

    public long firstSeenNanos() {
        return firstSeenNanos;
    }

    public double ageSeconds() {
        return (System.nanoTime() - lastSeenNanos) / 1e9;
    }

    /** Blend a new sighting into the estimate. Early sightings move it a lot, later ones a little. */
    void observe(double nx, double ny, double conf, long seenNanos) {
        double alpha = Math.max(0.25, 1.0 / (observations + 1));
        x += alpha * (nx - x);
        y += alpha * (ny - y);
        confidence = Math.max(confidence, conf);
        observations++;
        lastSeenNanos = seenNanos;
    }

    public double distanceTo(Vector2D p) {
        return Math.hypot(x - p.x(), y - p.y());
    }

    @Override
    public String toString() {
        return String.format(Locale.US, "#%d %s (%.1f, %.1f) n=%d conf=%.2f", id, color, x, y, observations, confidence);
    }
}
