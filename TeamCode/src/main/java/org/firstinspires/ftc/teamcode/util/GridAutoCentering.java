package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

public class GridAutoCentering {
    private MecanumSubsystem mecanumSubsystem;
    private GyroOdometry gyroOdometry;
    private double Kp = 0.75;
    private double Kd = 0;
    private double Ki = 0;
    private double integralSum = 0;
    private ElapsedTime timer;
    private double lastError = 0;
    public double baseAngle = 0; //default (reference) angle
    public double error;
    public double derivative;

    private double targetAngle = 0; //Offset angle from reference angle

    public GridAutoCentering(MecanumSubsystem mecanumSubsystem, GyroOdometry gyroOdometry) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.gyroOdometry = gyroOdometry;
        timer = new ElapsedTime();
    }

    public void reset(){
        baseAngle = gyroOdometry.getAngle();
        targetAngle = baseAngle;
    }

    public void setConstants(double kp, double ki, double kd){
        Kp = kp;
        Ki = ki;
        Kd = kd;
    }

    public void offsetTargetAngle(double angle){
        targetAngle = baseAngle + angle;
    }

    public void process(boolean run){
        if(run) {
            timer.reset();
            error = targetAngle - gyroOdometry.getAngle();
            if (error >= Math.PI) {
                error -= Math.PI * 2;
            } else if (error <= -Math.PI) {
                error += Math.PI * 2;
            }
            integralSum += error * timer.time();
            derivative = (error - lastError) / timer.time();
            lastError = error;
            if (Math.abs(error) > 0.005) {
                mecanumSubsystem.partialMoveAdjustment1(true, 0, 0, (error * Kp) + (derivative * Kd)/* + (integralSum * Ki)*/);
            }
        } else {
            mecanumSubsystem.partialMoveAdjustment1(true, 0, 0, 0);
        }
    }
    public void secondaryProcess(boolean run){
        timer.reset();
        error = targetAngle - gyroOdometry.getAngle();
        if (error >= Math.PI){
            error -= Math.PI*2;
        } else if (error <= -Math.PI){
            error += Math.PI*2;
        }
        integralSum += error * timer.time();
        derivative = (error - lastError) / timer.time();
        lastError = error;
        if (Math.abs(error) > 0.005 && run){
            mecanumSubsystem.fieldOrientedMove(0, 0, (error * Kp) + (derivative * Kd)/* + (integralSum * Ki)*/, 0);
        }
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void setTargetAngle(double targetAngle) {
        this.targetAngle = targetAngle;
    }
}
