package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.subsystems.IMUSubsystem;

public class AntiTip {
    private MecanumSubsystem mecanumSubsystem;
    private IMUSubsystem imuSubsystem;
    private PIDCore pidCore;
    private double Kp = 2.1;
    private double Ki = 0;
    private double Kd = 0;
    public double positionvalue = 0;
    public boolean running = false;

    public AntiTip(MecanumSubsystem mecanumSubsystem, IMUSubsystem imuSubsystem) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.imuSubsystem = imuSubsystem;
        pidCore = new PIDCore(Kp, Kd, Ki);
    }

    public void process(){
        if (Math.abs(imuSubsystem.angleX()) > 0.07 || Math.abs(imuSubsystem.angleY()) > 0.1){
            running = true;
            positionvalue = pidCore.outputPositional(0, -imuSubsystem.angleX());
            mecanumSubsystem.partialMoveAdjustment2(true, pidCore.outputPositional(0, -imuSubsystem.angleX()), pidCore.outputPositional(0, imuSubsystem.angleY()), 0);
        } else {
            running = false;
            mecanumSubsystem.partialMoveAdjustment2(true, 0, 0, 0);
        }
    }
}
