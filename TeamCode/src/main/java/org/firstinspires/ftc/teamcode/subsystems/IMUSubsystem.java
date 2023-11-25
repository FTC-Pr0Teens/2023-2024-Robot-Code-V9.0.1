package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Specifications;

//IMU: Inertial Measuring Unit
public class IMUSubsystem extends Specifications {
    private BNO055IMU imu;
    private Orientation orientation;
    private Orientation orientationDegree;
    private Acceleration acceleration;
    BNO055IMU.Parameters parameters;

    //d stands for Δ
    //c stands for constant

    public double x = 0;
    public double y = 0;
    public double Theta = 0;
    public double dTheta = 0;
    public double vxGlobal = 0;
    public double vyGlobal = 0;
    public double vxLocal = 0;
    public double vyLocal = 0;
    public double vTheta = 0;
    public double cTheta = 0;
    public double cx = 0;
    public double cy = 0;
    public ElapsedTime timer;
    public double loopTime;
    private final double twoPi = Math.PI*2;

    public IMUSubsystem(@NonNull HardwareMap hardwareMap) {
        timer = new ElapsedTime();
        parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imu = hardwareMap.get(BNO055IMU.class, IMU);
        imu.initialize(parameters);
        orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        orientationDegree = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        acceleration = imu.getLinearAcceleration();
    }

    public void resetAngle(){
        cTheta = orientation.thirdAngle;
    }

    public void resetAngleXY(){
        cx = orientation.firstAngle;
        cy = orientation.secondAngle;
    }

    public double angleX(){
        orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        return orientation.firstAngle - cx;
    }

    public double angleY(){
        orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        return orientation.secondAngle - cy;
    }

    public double angleZ(){
        orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        return orientation.thirdAngle;
    }

    public double angleZDeg(){
        orientationDegree = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        return orientationDegree.thirdAngle;
    }

    public double accelX(){
        return acceleration.xAccel;
    }

    public double accelY(){
        return acceleration.yAccel;
    }

    public double accelZ(){
        return acceleration.zAccel;
    }

    //async process for position and angle measurement
    public void imuProcess() {
        timer.reset();
        orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        orientationDegree = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        acceleration = imu.getLinearAcceleration();
        dTheta = (orientation.thirdAngle - cTheta) - Theta;
        vTheta = dTheta / timer.time(); //differentiate
        Theta = orientation.thirdAngle - cTheta;
        //normalize angle to: 0 < Theta < 2Π
        if (Theta >= twoPi){
            Theta -= twoPi;
        } else if (Theta < 0){
            Theta += twoPi;
        }
        vxLocal = vxLocal + accelX()*timer.time();//integrate
        vyLocal = vyLocal + accelY()*timer.time();//integrate
        vxGlobal = (vxLocal*Math.cos(Theta))+(vyLocal*Math.sin(Theta));//rotate
        vyGlobal = (vyLocal*Math.cos(Theta))-(vxLocal*Math.sin(Theta));//rotate
        x += vxGlobal * timer.time();//integrate
        y += vyGlobal * timer.time();//integrate
    }

    //async process for angle measurement
    public void gyroProcess(){
        timer.reset();
        orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        dTheta = orientation.thirdAngle - Theta - cTheta;
        Theta = orientation.thirdAngle - cTheta;
        vTheta = dTheta / timer.time();
        if (dTheta >= Math.PI){
            dTheta -= twoPi;
        } else if (dTheta <= -Math.PI){
            dTheta += twoPi;
        }
        loopTime = timer.time();
    }

    public double getTheta(){
        orientation = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        double Theta = orientation.thirdAngle - cTheta;
        //normalize angle to: 0 < Theta < 2Π
        if (Theta >= twoPi){
            Theta -= twoPi;
        } else if (Theta < 0){
            Theta += twoPi;
        }
        return Theta;
    }
}