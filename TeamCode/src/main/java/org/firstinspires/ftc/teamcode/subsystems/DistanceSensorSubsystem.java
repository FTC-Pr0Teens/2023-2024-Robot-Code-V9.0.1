package org.firstinspires.ftc.teamcode.threadopmode.subsystems;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.Specifications;

public class DistanceSensorSubsystem extends Specifications {

    private DistanceSensor distanceSensor;
    private Rev2mDistanceSensor rev2mDistanceSensor;

    public enum SensorPosition{
        frontLeft,
        frontRight,
        backLeft,
        backRight,
        leftFront,
        leftBack,
        rightFront,
        rightBack,
        output,
        intake,
        left,
        right
    }

    public DistanceSensorSubsystem(HardwareMap hardwareMap, SensorPosition sensorPosition) {
        if (sensorPosition == SensorPosition.frontLeft){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_FT_LF);
        } else if (sensorPosition == SensorPosition.frontRight){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_FT_RT);
        } else if (sensorPosition == SensorPosition.backLeft){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_BK_LF);
        } else if (sensorPosition == SensorPosition.backRight){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_BK_RT);
        } else if (sensorPosition == SensorPosition.leftFront){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_LF_FT);
        } else if (sensorPosition == SensorPosition.leftBack){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_LF_BK);
        } else if (sensorPosition == SensorPosition.rightFront){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_RT_FT);
        } else if (sensorPosition == SensorPosition.rightBack){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_RT_BK);
        } else if (sensorPosition == SensorPosition.output){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_OUTPUT);
        } else if (sensorPosition == SensorPosition.intake){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_INTAKE);
        } else if (sensorPosition == SensorPosition.left){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_LF);
        } else if (sensorPosition == SensorPosition.right){
            distanceSensor = hardwareMap.get(DistanceSensor.class, DS_RT);
        }
        rev2mDistanceSensor = (Rev2mDistanceSensor) distanceSensor;
    }

    public double getDistance() {
        return distanceSensor.getDistance(DistanceUnit.CM);
    }
    public boolean getTimeOut() {
        return rev2mDistanceSensor.didTimeoutOccur();
    }

}