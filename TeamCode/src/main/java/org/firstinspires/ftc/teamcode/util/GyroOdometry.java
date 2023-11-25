package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.subsystems.OdometrySubsystem;

import java.util.concurrent.TimeUnit;

public class GyroOdometry extends Specifications{
    private OdometrySubsystem odometrySubsystem;
    private IMUSubsystem imuSubsystem;
    private Initialize initialize;
    public double totalsEncoder = 0;
    public double totalfEncoder = 0;
    public double tempXIntegrate = 0;
    public double tempYIntegrate = 0;
    public double x = 0;
    public double y = 0;
    public double theta = 0;
    public double dx;
    public double dy;
    public double vxGlobal = 0;
    public double vyGlobal = 0;
    public double vxLocal = 0;
    public double vyLocal = 0;
    public double vTheta = 0;
    public ElapsedTime time;
    public double loopTime;
    private double sEncoderf;
    private double bEncoderf;
    private double lEncoderf;
    private double sEncoderi = 0;
    private double bEncoderi = 0;
    private double lEncoderi = 0;
    private double twoPi = Math.PI*2;
    public double tempX;
    public double tempY;
    public double dTheta;
    private double dc =odometryCir/odometryTick;
    public double dxc =SideOdometryToCentre*Math.cos(sideOdometryAngleFromCentre);
    public double dyc =lengthFromOdometrySideToFront*Math.cos(frontOdometryAngleFromCentre);
    private double thetaTemp;
    public double testx/* =  dx*Math.cos(theta)*/;
    public double testx2/* = dy*Math.sin(theta)*/;
    public double testy/* = dy*Math.cos(theta)*/;
    public double testy2/* = dx*Math.sin(theta)*/;
//    public double positionTime;

    public GyroOdometry(OdometrySubsystem odometrySubsystem, IMUSubsystem imuSubsystem) {
        this.odometrySubsystem = odometrySubsystem;
        this.imuSubsystem = imuSubsystem;
        time = new ElapsedTime();
    }

    public void initialize(){
        x = initialize.x;
        y = initialize.y;
        theta = initialize.theta;
    }

    public void process(){
        time.reset();
        theta = imuSubsystem.Theta;
        dTheta = imuSubsystem.dTheta;
        vTheta = imuSubsystem.vTheta;
        totalfEncoder += bEncoderf;
        totalsEncoder += sEncoderf;
        sEncoderf = odometrySubsystem.rightEncoder(); //s = side, b = back
        bEncoderf = odometrySubsystem.backEncoder(); //dc = cm per tick, dyc = offset Y encoder, dxc = offset X encoder
        dx = (sEncoderf-sEncoderi)*dc - dxc*dTheta;
        dy = (bEncoderf-bEncoderi)*dc + dyc*dTheta;
        tempX = (dx*Math.cos(theta))+(dy*Math.sin(theta));
        tempY = (dy*Math.cos(theta))-(dx*Math.sin(theta));
        testx = (bEncoderf-bEncoderi)*dc;
        testx2 = (sEncoderf-sEncoderi)*dc;
        testy = dxc*dTheta;
        testy2 = dyc*dTheta;
        x = x+tempX;
        y = y+tempY;

        tempXIntegrate += tempX*time.time();
        tempYIntegrate += tempY*time.time();
        vxGlobal = tempX/time.time();
        vyGlobal = tempY/time.time();
        vxLocal = dx/time.time();
        vyLocal = dy/time.time();
        sEncoderi = sEncoderf;
        bEncoderi = bEncoderf;
        loopTime = time.time();
    }

    public void odometryProcess(){
        odometrySubsystem.process();
        x = odometrySubsystem.x;
        y = odometrySubsystem.y;
        theta = odometrySubsystem.theta;
    }

    public void combinedProcess(){
        time.reset();
        lEncoderf = odometrySubsystem.leftEncoder();
        sEncoderf = odometrySubsystem.rightEncoder();
        bEncoderf = odometrySubsystem.backEncoder();
        dx = odometrySubsystem.dxc*((lEncoderf-lEncoderi)+(sEncoderf-sEncoderi));
        dTheta = odometrySubsystem.dThetac*((sEncoderf-sEncoderi)-(lEncoderf-lEncoderi)); //unit circle direction
//        dTheta = imuSubsystem.angleZ() - theta - imuSubsystem.cTheta;
//        if (dTheta >= Math.PI){
//            dTheta -= twoPi;
//        } else if (dTheta <= -Math.PI){
//            dTheta += twoPi;
//        }
        theta = imuSubsystem.angleZ() - imuSubsystem.cTheta;
        dy = (odometrySubsystem.dyc*(bEncoderf-bEncoderi))+(lengthFromOdometrySideToFront*dTheta);
        tempX = (dx*Math.cos(theta))+(dy*Math.sin(theta));
        tempY = (dy*Math.cos(theta))-(dx*Math.sin(theta));
        x = x+tempX;
        y = y+tempY;
        vxGlobal = tempX/time.time(TimeUnit.SECONDS);
        vyGlobal = tempY/time.time(TimeUnit.SECONDS);
        vxLocal = dx/time.time(TimeUnit.SECONDS);
        vyLocal = dy/time.time(TimeUnit.SECONDS);
        vTheta = dTheta/time.time(TimeUnit.SECONDS);
        sEncoderi = sEncoderf;
        lEncoderi = lEncoderf;
        bEncoderi = bEncoderf;
    }

    public void angleProcess(){
        theta = imuSubsystem.Theta;
        dTheta = imuSubsystem.dTheta;
        vTheta = imuSubsystem.vTheta;
    }

    public double getAngle(){
        return imuSubsystem.angleZ() - imuSubsystem.cTheta;
    }

    public double getAngle2() {
        return imuSubsystem.getTheta();
    }
}
