package org.firstinspires.ftc.teamcode.util;

import java.util.HashMap;
import java.util.NoSuchElementException;

/**
 * Stores information on coordinates used in Ops.
 * Coordinates shift when setting the initial starting position.
 */
public class CoordinateSet {

    /**
     * The robot's starting position on the field.
     */
    public enum StartingPoint {
        FRONT_BLUE,
        FRONT_RED,
        BACK_BLUE,
        BACK_RED
    }

    /**
     * The names of important locations on the field.
     */
    public enum KeyPoints {
        ORIGIN,
        SPIKE_CHECKPOINT,
        SPIKE_LEFT,
        SPIKE_MIDDLE,
        SPIKE_RIGHT,
        PIXEL_STACK_CHECKPOINT,
        PIXEL_STACK,
        MIDDLE_BACK,
        MIDDLE_FRONT,
        PIXEL_BOARD_CHECKPOINT,
        PIXEL_BOARD,
        APRIL_TAG_LEFT,
        APRIL_TAG_MIDDLE,
        APRIL_TAG_RIGHT,
        PARKING_LEFT_CHECKPOINT,
        PARKING_LEFT,
        PARKING_RIGHT_CHECKPOINT,
        PARKING_RIGHT,
    }

    /**
     * A mapping of KeyPoints to a Coordinate object.
     */
    HashMap<KeyPoints, Coordinate> coordinates;

    /**
     * Assigns coordinates to key points which depend on the starting point of the robot.
     *
     * @param startingPoint the starting point of the robot
     */
    public CoordinateSet(StartingPoint startingPoint) {
        coordinates.put(KeyPoints.ORIGIN, new Coordinate(0, 0, 0));
        switch (startingPoint) {
            case FRONT_BLUE:
                coordinates.put(KeyPoints.SPIKE_CHECKPOINT, new Coordinate(71.5, 0, 0));
                coordinates.put(KeyPoints.SPIKE_LEFT, new Coordinate(107.76, 19.99, 0));
                coordinates.put(KeyPoints.SPIKE_MIDDLE, new Coordinate(120.36, -12.48, 0));
                coordinates.put(KeyPoints.SPIKE_RIGHT, new Coordinate(69.48, -22, 2.11));

//                coordinates.put(KeyPoints.PIXEL_STACK_CHECKPOINT, new Coordinate());
//                coordinates.put(KeyPoints.PIXEL_STACK, new Coordinate());
//
//                coordinates.put(KeyPoints.MIDDLE_BACK, new Coordinate());
//                coordinates.put(KeyPoints.MIDDLE_FRONT, new Coordinate());

                coordinates.put(KeyPoints.PIXEL_BOARD_CHECKPOINT, new Coordinate(133, 31, Math.PI / 2));
                coordinates.put(KeyPoints.PIXEL_BOARD, new Coordinate(80.5, 49, Math.PI / 2));

                coordinates.put(KeyPoints.APRIL_TAG_LEFT, new Coordinate(66, 85,  Math.PI / 2));
                coordinates.put(KeyPoints.APRIL_TAG_MIDDLE, new Coordinate(76, 85,  Math.PI / 2));
                coordinates.put(KeyPoints.APRIL_TAG_RIGHT, new Coordinate(90, 85, Math.PI / 2));

                coordinates.put(KeyPoints.PARKING_LEFT_CHECKPOINT, new Coordinate(9, 80, Math.PI / 2));
                coordinates.put(KeyPoints.PARKING_LEFT, new Coordinate(9, 111, Math.PI / 2));

                coordinates.put(KeyPoints.PARKING_RIGHT_CHECKPOINT, new Coordinate(133, 80, Math.PI / 2));
                coordinates.put(KeyPoints.PARKING_RIGHT, new Coordinate(133, 111, Math.PI / 2));

                break;
            case FRONT_RED:
//                coordinates.put(KeyPoints.SPIKE_CHECKPOINT, new Coordinate());
                coordinates.put(KeyPoints.SPIKE_LEFT, new Coordinate(73.48, 19.57, -Math.PI /2));
                coordinates.put(KeyPoints.SPIKE_MIDDLE, new Coordinate(120.26, 2.02, 0));
                coordinates.put(KeyPoints.SPIKE_RIGHT, new Coordinate(108.9, -26.5, 0));

//                coordinates.put(KeyPoints.PIXEL_STACK_CHECKPOINT, new Coordinate());
//                coordinates.put(KeyPoints.PIXEL_STACK, new Coordinate());
//
//                coordinates.put(KeyPoints.MIDDLE_BACK, new Coordinate());
//                coordinates.put(KeyPoints.MIDDLE_FRONT, new Coordinate());

                coordinates.put(KeyPoints.PIXEL_BOARD_CHECKPOINT, new Coordinate(125.55, -42.61, -Math.PI / 2));
                coordinates.put(KeyPoints.PIXEL_BOARD, new Coordinate(64, -59, -Math.PI / 2));

                coordinates.put(KeyPoints.APRIL_TAG_LEFT, new Coordinate(75, -85, -Math.PI / 2));
                coordinates.put(KeyPoints.APRIL_TAG_MIDDLE, new Coordinate(68, -85, -Math.PI / 2));
                coordinates.put(KeyPoints.APRIL_TAG_RIGHT, new Coordinate(52, -85, -Math.PI / 2));

                coordinates.put(KeyPoints.PARKING_LEFT_CHECKPOINT, new Coordinate(9, -80, Math.PI / 2));
                coordinates.put(KeyPoints.PARKING_LEFT, new Coordinate(9, -111, Math.PI / 2));

                coordinates.put(KeyPoints.PARKING_RIGHT_CHECKPOINT, new Coordinate(133, -80, Math.PI / 2));
                coordinates.put(KeyPoints.PARKING_RIGHT, new Coordinate(133, -111, Math.PI / 2));

                break;
            case BACK_BLUE:
//                coordinates.put(KeyPoints.SPIKE_CHECKPOINT, new Coordinate());
//                coordinates.put(KeyPoints.SPIKE_LEFT, new Coordinate());
//                coordinates.put(KeyPoints.SPIKE_MIDDLE, new Coordinate());
//                coordinates.put(KeyPoints.SPIKE_RIGHT, new Coordinate());
//
//                coordinates.put(KeyPoints.PIXEL_STACK_CHECKPOINT, new Coordinate());
//                coordinates.put(KeyPoints.PIXEL_STACK, new Coordinate());
//
//                coordinates.put(KeyPoints.MIDDLE_BACK, new Coordinate());
//                coordinates.put(KeyPoints.MIDDLE_FRONT, new Coordinate());
//
//                coordinates.put(KeyPoints.PIXEL_BOARD_CHECKPOINT, new Coordinate());
//                coordinates.put(KeyPoints.PIXEL_BOARD, new Coordinate());
//
//                coordinates.put(KeyPoints.APRIL_TAG_LEFT, new Coordinate());
//                coordinates.put(KeyPoints.APRIL_TAG_MIDDLE, new Coordinate());
//                coordinates.put(KeyPoints.APRIL_TAG_RIGHT, new Coordinate());
//
//                coordinates.put(KeyPoints.PARKING_LEFT_CHECKPOINT, new Coordinate());
//                coordinates.put(KeyPoints.PARKING_LEFT, new Coordinate());
//
//                coordinates.put(KeyPoints.PARKING_RIGHT_CHECKPOINT, new Coordinate());
//                coordinates.put(KeyPoints.PARKING_RIGHT, new Coordinate());

                break;
            default: // BACK_RED
                coordinates.put(KeyPoints.SPIKE_CHECKPOINT, new Coordinate(71.5, -0, 0));
                coordinates.put(KeyPoints.SPIKE_LEFT, new Coordinate(107.76, 19.99, 0));
                coordinates.put(KeyPoints.SPIKE_MIDDLE, new Coordinate(120.36, -12.48, 0));
                coordinates.put(KeyPoints.SPIKE_RIGHT, new Coordinate(69.48, -22, 2.11));

                coordinates.put(KeyPoints.PIXEL_STACK_CHECKPOINT, new Coordinate(129, 43.5, -Math.PI/2));
                coordinates.put(KeyPoints.PIXEL_STACK, new Coordinate(129, 60.8, -Math.PI/2));

                coordinates.put(KeyPoints.MIDDLE_BACK, new Coordinate(130, 8, -Math.PI / 2));
                coordinates.put(KeyPoints.MIDDLE_FRONT, new Coordinate(130, -130, -Math.PI / 2));

                coordinates.put(KeyPoints.PIXEL_BOARD_CHECKPOINT, new Coordinate(127, -181.2, -Math.PI/2));
                coordinates.put(KeyPoints.PIXEL_BOARD, new Coordinate(60, -194.56, -Math.PI/2));

                coordinates.put(KeyPoints.APRIL_TAG_LEFT, new Coordinate(75, -220,  - Math.PI / 2));
                coordinates.put(KeyPoints.APRIL_TAG_MIDDLE, new Coordinate(68, -220, - Math.PI / 2));
                coordinates.put(KeyPoints.APRIL_TAG_RIGHT, new Coordinate(52, -220, - Math.PI / 2));

                coordinates.put(KeyPoints.PARKING_LEFT_CHECKPOINT, new Coordinate(9, -210, Math.PI / 2));
                coordinates.put(KeyPoints.PARKING_LEFT, new Coordinate(9, -250, Math.PI / 2));

                coordinates.put(KeyPoints.PARKING_RIGHT_CHECKPOINT, new Coordinate(133, -210, Math.PI / 2));
                coordinates.put(KeyPoints.PARKING_RIGHT, new Coordinate(133, -250, Math.PI / 2));
        }
    }

    /**
     * Retrieve a single Coordinate object when supplied a key point.
     *
     * @param keyPoint the name of the coordinate to get
     * @return a Coordinate object of the requested key point
     */
    public Coordinate getCoordinate(KeyPoints keyPoint) {
        if (!coordinates.containsKey(keyPoint))
            throw new IllegalArgumentException(
                    "Keypoint '" +keyPoint +"' does not exist " +
                            "(Check that it's been made in the CoordinateSet)!"
            );
        return coordinates.get(keyPoint);
    }
}
