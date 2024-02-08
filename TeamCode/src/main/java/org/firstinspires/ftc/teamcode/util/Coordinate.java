package org.firstinspires.ftc.teamcode.util;

/**
 * A single coordinate on the playing field.
 * Note that all coordinates are relative to the robot's starting position.
 * Robots may also have their own axis that these coordinates lie on.
 */
public class Coordinate {

    /**
     * The coordinate's X value.
     */
    private final double X;

    /**
     * The coordinate's Y value.
     */
    private final double Y;

    /**
     * The coordinate's theta value (in radians).
     */
    private final double THETA;

    /**
     * Creates a coordinate value with the corresponding X, Y, and Theta values.
     * @param x the double X value to assign the coordinate
     * @param y the double Y value to assign the coordinate
     * @param theta the double theta value to assign the coordinate
     */
    public Coordinate(double x, double y, double theta) {
        X = x;
        Y = y;
        THETA = theta;
    }

    /**
     * Retrieve this coordinate's X value.
     * @return this coordinate's double X value.
     */
    public double getX() {
        return X;
    }
    /**
     * Retrieve this coordinate's Y value.
     * @return this coordinate's double Y value.
     */
    public double getY() {
        return Y;
    }
    /**
     * Retrieve this coordinate's theta value.
     * @return this coordinate's double theta value.
     */
    public double getTheta() {
        return THETA;
    }
}
