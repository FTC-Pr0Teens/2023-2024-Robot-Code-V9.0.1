package org.firstinspires.ftc.teamcode.util;

public class IntervalControl {
    private Interval[] intervals;

    public IntervalControl(Interval... intervals) {
        this.intervals = intervals;
    }

    public double getOutput(double input) {
        for (Interval interval : intervals) {
            if (input >= interval.getStartPos() && input <= interval.getEndPos()) {
                return interval.getOutput();
            }
        }
        //throw an exception if no such interval found
        throw new IllegalArgumentException("Input " + input + " is not in any interval");
    }


}

