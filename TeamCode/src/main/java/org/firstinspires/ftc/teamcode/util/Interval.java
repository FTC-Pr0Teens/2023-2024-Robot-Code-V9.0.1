package org.firstinspires.ftc.teamcode.util;

public class Interval {
    private double startPos;
    private double endPos;
    private double output;

    public Interval(double start, double end, double output) {
        this.startPos = start;
        this.endPos = end;
        this.output = output;
    }

    public double getStartPos() {
        return startPos;
    }

    public double getEndPos() {
        return endPos;
    }

    public double getOutput() {
        return output;
    }
}
