package org.firstinspires.ftc.teamcode.util;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.threadopmode.subsystems.DistanceSensorSubsystem;


public class Initialize extends Specifications{
    public double x;
    public double y;
    public double theta;
    private DistanceSensorSubsystem distance_sensor_backLeft;
    private DistanceSensorSubsystem distance_sensor_backRight;
    private DistanceSensorSubsystem distance_sensor_right;
    private DistanceSensorSubsystem distance_sensor_left;

    public Initialize(DistanceSensorSubsystem distance_sensor_backLeft, DistanceSensorSubsystem distance_sensor_backRight, DistanceSensorSubsystem distance_sensor_right, DistanceSensorSubsystem distance_sensor_left) {
        this.distance_sensor_backLeft = distance_sensor_backLeft;
        this.distance_sensor_backRight = distance_sensor_backRight;
        this.distance_sensor_right = distance_sensor_right;
        this.distance_sensor_left = distance_sensor_left;
    }

    public enum Position{
        LEFT,
        RIGHT
    }

    public void setPosition(@NonNull Position position) {

        double bl = distance_sensor_backLeft.getDistance();
        double br = distance_sensor_backRight.getDistance();
        double r = distance_sensor_right.getDistance();
        double l = distance_sensor_left.getDistance();

        double thetaTemp;
        double arctan = Math.atan((br - bl) / distance_sensor_back_space);
        switch (position){
            case LEFT:
                thetaTemp = arctan;
                theta = thetaTemp;
                x = (((br + bl) / 2) + distance_sensor_back_margin) * Math.cos(thetaTemp);
                y = (l + (distance_sensor_left_space * Math.tan(thetaTemp)) + distance_sensor_left_margin) * Math.cos(thetaTemp);
                break;
            case RIGHT:
                thetaTemp = arctan;
                theta = thetaTemp;
                y = (365.76 - ((r + (distance_sensor_right_space * Math.tan(thetaTemp)) + distance_sensor_right_margin) * Math.cos(thetaTemp)));
                x = (((bl + br) / 2) + distance_sensor_back_margin) * Math.cos(thetaTemp);
                break;
        }
    }
}
