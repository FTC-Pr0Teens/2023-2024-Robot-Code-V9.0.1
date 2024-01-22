package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.Specifications;

import java.util.concurrent.TimeUnit;

public class OdometrySubsystem extends Specifications {
    private DcMotor leftEncoder;
    private DcMotor rightEncoder;
    private DcMotor backEncoder;

    //d stands for Δ
    //c stands for constant
    //l: left
    //r: right
    //b: back (front)
    //i: initial
    //f: final

    public double x = 0;
    public double y = 0;
    public double theta = 0;
    public double vxGlobal = 0;
    public double vyGlobal = 0;
    public double vxLocal = 0;
    public double vyLocal = 0;
    public double vTheta = 0;
    public double headingChange = 0;
    public double dx = 0;
    public double dy = 0;
    public double dTheta = 0;
    private int lEncoderi = 0;
    private int rEncoderi = 0;
    private int bEncoderi = 0;
    private int lEncoderf = 0;
    private int rEncoderf = 0;
    private int bEncoderf = 0;
    private ElapsedTime time;

    //Constants in the calculation of dx, dy, and dTheta
    public final double dxc = odometryCir/(odometryTick*2);
    public final double dThetac = odometryCir/(odometryTick*2*SideOdometryToCentre*Math.cos(sideOdometryAngleFromCentre));
    public final double dyc = odometryCir/odometryTick;
    private final double twoPi = 2*Math.PI;
    private double tempX;
    private double tempY;

    //async process for position and angle measurement
    public void process(){  
        time.reset();
        lEncoderf = leftEncoder();
        rEncoderf = rightEncoder();
        bEncoderf = backEncoder();
        dx = dxc*((lEncoderf-lEncoderi)+(rEncoderf-rEncoderi));
        dTheta = dThetac*((rEncoderf-rEncoderi)-(lEncoderf-lEncoderi)); //unit circle direction
        dy = (dyc*(bEncoderf-bEncoderi))+(lengthFromOdometrySideToFront*dTheta);
        //ThetaTemp = 0.96388888888*Theta+(dTheta/2);
//        x = x+(dx*Math.cos(Theta))-(dy*Math.sin(Theta));
//        if (dTheta > 0) {
//            x = x + (dx*Math.cos(Theta)) + (dy*Math.sin(Theta));
//            y = y + (dy*Math.cos(Theta)) - (dx*Math.sin(Theta));
//        } else {
//            x = x - (dx*Math.cos(Theta)) + (dy*Math.sin(Theta));
//            y = y - (dy*Math.cos(Theta)) - (dx*Math.sin(Theta));
//        }
        //Drive Gears
        /*
        x += dx * Math.cos(theta) - dy * Math.sin(theta);
        y += dx * Math.sin(theta) + dy * Math.cos(theta);
        */

        //TODO:other code

        x += dx * Math.cos(theta) + dy * Math.cos(Math.PI/2 + theta);
        y += dx * Math.sin(theta) + dy * Math.sin(Math.PI/2 + theta);


        x += dx * Math.cos(theta) + dy * Math.sin(theta);
        y += -dx * Math.sin(theta) + dy * Math.cos(theta);


        x = x+tempX;
        y = y+tempY;
        theta += dTheta/*1.03746397695*/;
        if (theta > twoPi){
            theta -= twoPi;
        } else if (theta < -twoPi){
            theta += twoPi;
        }
        rEncoderi = rEncoderf;
        lEncoderi = lEncoderf;
        bEncoderi = bEncoderf;
    }

    public OdometrySubsystem(HardwareMap hardwareMap) {

        if (navSystem != NavSystem.IMU){
            if (navSystem == NavSystem.ODOMETRY){
                leftEncoder = hardwareMap.get(DcMotor.class, LF_ENCODER);
                leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
                leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            rightEncoder = hardwareMap.get(DcMotor.class, RT_ENCODER);
            backEncoder = hardwareMap.get(DcMotor.class, BK_ENCODER);
            rightEncoder.setDirection(DcMotorSimple.Direction.FORWARD);
            backEncoder.setDirection(DcMotorSimple.Direction.REVERSE);
            rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        time = new ElapsedTime();
    }

    public void reset(){
        if (navSystem != NavSystem.IMU){
            if (navSystem == NavSystem.ODOMETRY) {
                leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public double convertBearing(double theta){

        if (theta < 0){
            theta %= -2*Math.PI;
            theta += 2*Math.PI;
        } else {
            theta %= 2*Math.PI;
        }
        return Math.toDegrees(theta);
    }

    public int backEncoder(){ return -backEncoder.getCurrentPosition(); }

    public int leftEncoder(){
        return -leftEncoder.getCurrentPosition();
    }

    public int rightEncoder(){
        return -rightEncoder.getCurrentPosition();
    }
}
