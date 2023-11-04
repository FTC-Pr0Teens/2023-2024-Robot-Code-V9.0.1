package org.firstinspires.ftc.teamcode.threadopmode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PotentiometerPositionServoSubsystem extends NormalServoSubsystem{

    AnalogInput potentiometer;
    ElapsedTime timer;
    double error = 0;
    double lastError;
    private double kp = 0.05;
    private double ki = 0;
    private double kd = 0;
    private double derivative = 0;
    private double integral = 0;
    private double adjustment = 0;
    private double PIDPosition;

    private boolean pr = false;
    private ElapsedTime prTimer;

    public PotentiometerPositionServoSubsystem(HardwareMap hardwareMap, ServoType servoType) {
        super(hardwareMap, servoType);
        potentiometer = hardwareMap.get(AnalogInput.class, "potentiometer");
        timer = new ElapsedTime();
        prTimer = new ElapsedTime();
    }

    @Override
    public double getAngle() {
        return potentiometer.getVoltage() / 3.3 * 360;
    }

    public double getTargetAngle() {
        return ((position - 0.05)/0.85) * (220 - 141) + 141;
    }

    public double getLinearDisplacement(){
        return 6 * 18.4 * Math.cos(Math.toRadians(getAngle()));
    }

    public double getTargetLinearDisplacement(){
        return 6 * 18.4 * Math.cos(Math.toRadians(getTargetAngle()));
    }

    @Override
    public void pidToPosition(boolean run, double position) {
        if (run){
            timer.reset();
            lastError = error;
            error = getTargetLinearDisplacement() - getLinearDisplacement();
            derivative = (error - lastError) / timer.time();
            integral += error * timer.time();
            adjustment = kp * error + ki * integral + kd * derivative;
            PIDPosition += adjustment;
            toPosition(true, (Math.acos(PIDPosition/6/18.4) - 141) / (220 - 141) * 0.85 + 0.05);
        }
    }

    public double getActualPosition(){
        return (getAngle() - 141) / (220 - 141) * 0.85 + 0.05;
    }

    public boolean positionReached() {
        if (!pr && Math.abs(getActualPosition() - position) < 0.08){
            prTimer.reset();
        }
        pr = Math.abs(getActualPosition() - position) < 0.08;
        return prTimer.time() > 0.40 && pr;
    }
}
