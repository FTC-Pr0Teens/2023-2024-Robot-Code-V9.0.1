package org.firstinspires.ftc.teamcode.command;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.threadopmode.subsystems.DistanceSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.PIDCore;
import org.firstinspires.ftc.teamcode.util.PurePursuit;
import org.firstinspires.ftc.teamcode.util.VectorCartesian;

@Config
public class MecanumCommand {
    private MecanumSubsystem mecanumSubsystem;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private DistanceSensorSubsystem distance_sensor_backLeft, distance_sensor_backRight, distance_sensor_right, distance_sensor_left;
    private boolean run;
    private OpMode opMode;
    private ElapsedTime elapsedTime;
    private PIDCore globalPositionController;
    public PIDCore globalXController;
    public PIDCore globalYController;
    public PIDCore globalThetaController;
    public PIDCore globalVXController;
    public PIDCore globalVYController;
    public PIDCore globalVThetaController;
    private static double kp = 0.066;
    private static double ki = 0;
    private static double kd = 0.08;
    private static double kpx = 0.066;
    private static double kdx = 0.08;
    private static double kix = 0;
    private static double kpy = 0.08;
    private static double kdy = 0.0005;
    private static double kiy = 0;
    private static double kptheta = 2;
    private static double kdtheta = 0.2;
    private static double kitheta = 0.0;
    private static double kpvx = 0.25;
    private static double kdvx = 0.2;
    private static double kivx = 0;
    private static double kpvy = 0.25;
    private static double kdvy = 0.2;
    private static double kivy = 0;
    private static double kpvtheta = 5;
    private static double kdvtheta = 0.2;
    private static double kivtheta = 0;
    private double e = 0;
    private double heading = 0;
    private double ex = 0;
    private double ey = 0;
    private double etheta = 0;
    private double xFinal;
    private double yFinal;
    private double thetaFinal;
    private double velocity;
//    public double currentTime;
    public double positionTime;
    private PurePursuit purePursuit;
    private boolean runPurePursuit = false;
    public VectorCartesian currentTargetPoint;

    public void setthetaConstants(double kptheta, double kdtheta, double kitheta){
        MecanumCommand.kptheta = kptheta;
        MecanumCommand.kdtheta = kdtheta;
        MecanumCommand.kitheta = kitheta;
        globalThetaController.setConstant(kptheta,kdtheta,kitheta);
    }
    public void setConstants(double kpx, double kdx, double kix, double kpy, double kdy, double kiy, double kptheta, double kdtheta, double kitheta){
        MecanumCommand.kpx = kpx;
        MecanumCommand.kdx = kdx;
        MecanumCommand.kix = kix;
        MecanumCommand.kpy = kpy;
        MecanumCommand.kdy = kdy;
        MecanumCommand.kiy = kiy;
        MecanumCommand.kptheta = kptheta;
        MecanumCommand.kdtheta = kdtheta;
        MecanumCommand.kitheta = kitheta;
        globalXController.setConstant(kpx, kdx, kix);
        globalYController.setConstant(kpy, kdy, kiy);
        globalThetaController.setConstant(kptheta, kdtheta, kitheta);
    }

    public MecanumCommand(MecanumSubsystem mecanumSubsystem, OdometrySubsystem odometrySubsystem, DistanceSensorSubsystem distance_sensor_backLeft, DistanceSensorSubsystem distance_sensor_backRight, DistanceSensorSubsystem distance_sensor_right, DistanceSensorSubsystem distance_sensor_left, GyroOdometry gyroOdometry, OpMode opMode, PurePursuit purePursuit) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.distance_sensor_backLeft = distance_sensor_backLeft;
        this.distance_sensor_backRight = distance_sensor_backRight;
        this.distance_sensor_right = distance_sensor_right;
        this.distance_sensor_left = distance_sensor_left;
        this.gyroOdometry = gyroOdometry;
        this.opMode = opMode;
        globalPositionController = new PIDCore(kp, kd, ki);
        globalXController = new PIDCore(kpx, kdx, kix);
        globalYController = new PIDCore(kpy, kdy, kiy);
        globalThetaController = new PIDCore(kptheta, kdtheta, kitheta);
        globalVXController = new PIDCore(kpvx, kdvx, kivx);
        globalVYController = new PIDCore(kpvy, kdvy, kivy);
        globalVThetaController = new PIDCore(kpvtheta, kdvtheta, kivtheta);
        elapsedTime = new ElapsedTime();
        this.purePursuit = purePursuit;
        xFinal = gyroOdometry.x;
        yFinal = gyroOdometry.y;
        thetaFinal = gyroOdometry.theta;
        velocity = 0;
    }

    public void pidProcess(){
        ex = globalXController.outputPositional(xFinal, gyroOdometry.x);
        ey = globalYController.outputPositional(yFinal, gyroOdometry.y);
        etheta = globalThetaController.outputPositional(thetaFinal, gyroOdometry.theta);
        if (Math.abs(ex) > velocity || Math.abs(ey) > velocity){
            double max = Math.max(Math.abs(ex), Math.abs(ey));
            ex = ex / max * velocity;
            ey = ey / max * velocity;
            etheta = etheta / max * velocity;
        }
        moveGlobalPartial(true, ex, ey, etheta);
    }

    public void pidProcessPurePursuit(){
        e = globalPositionController.outputPositional(Math.sqrt(Math.pow((purePursuit.getPoints().get(purePursuit.getCurrentTargetPoint()).getX() - gyroOdometry.x), 2) + Math.pow((purePursuit.getPoints().get(purePursuit.getCurrentTargetPoint()).getY() - gyroOdometry.y), 2)));
        heading = Math.atan2(xFinal - gyroOdometry.x, yFinal - gyroOdometry.y);
        ex = e * Math.sin(heading);
        ey = e * Math.cos(heading);
        etheta = globalThetaController.outputPositional(thetaFinal, gyroOdometry.theta);
        if (Math.abs(ex) > velocity || Math.abs(ey) > velocity){
            double max = Math.max(Math.abs(ex), Math.abs(ey));
            ex = ex / max * velocity;
            ey = ey / max * velocity;
            etheta = etheta / max * velocity;
        }
        moveGlobalPartial(true, ex, ey, etheta);
    }

    public void move(boolean run, double lv, double lh, double rv, double rh) {
        mecanumSubsystem.move(run, lv, lh, rv, rh);
    }

    public void moveGlobal(boolean run, double vertical, double horizontal, double rotational){
        if (run){
            double localVertical = vertical*Math.cos(gyroOdometry.theta) - horizontal*Math.cos(Math.PI/2-gyroOdometry.theta);
            double localHorizontal = vertical*Math.sin(gyroOdometry.theta) + horizontal*Math.sin(Math.PI/2-gyroOdometry.theta);
            mecanumSubsystem.move(true, localVertical, localHorizontal, rotational);
        }
    }

    public void moveGlobalVelocity(boolean run, double vertical, double horizontal, double rotational){
        if (run){
            double localVertical = vertical*Math.cos(gyroOdometry.theta) - horizontal*Math.cos(Math.PI/2-gyroOdometry.theta);
            double localHorizontal = vertical*Math.sin(gyroOdometry.theta) + horizontal*Math.sin(Math.PI/2-gyroOdometry.theta);
            mecanumSubsystem.moveVelocity(true, localVertical, localHorizontal, rotational);
        }
    }

    public void moveGlobalPartial(boolean run, double vertical, double horizontal, double rotational){
        if (run){
            double angle = Math.PI/2 - gyroOdometry.theta;
            double localVertical = vertical*Math.cos(gyroOdometry.theta) - horizontal*Math.cos(angle);
            double localHorizontal = vertical*Math.sin(gyroOdometry.theta) + horizontal*Math.sin(angle);
            mecanumSubsystem.partialMove(true, localVertical, localHorizontal, rotational);
        }
    }
//
//    public void movePartial(boolean run, double vertical, double horizontal, double rotational){
//        if (run){
//            vertical *= mecanumSubsystem.MAX_ANGULAR_VEL;
//            horizontal *= mecanumSubsystem.MAX_ANGULAR_VEL;
//            rotational *= mecanumSubsystem.MAX_ANGULAR_VEL;
//            mecanumSubsystem.partialMove(true, vertical, horizontal, rotational);
//        }
//    }
//
//    public void localToCoordMixed(boolean run, double xf, double yf, double power){
//        double xi = odometrySubsystem.rightEncoder();
//        double yi = odometrySubsystem.backEncoder();
//        double x = 0;
//        double y = 0;
//        while (run && (Math.abs(xf - x) > 0.1 || Math.abs(yf - y) > 0.1)){
//            x = (odometrySubsystem.rightEncoder() - xi) / odometrySubsystem.odometryTick * odometrySubsystem.odometryCir;
//            y = (odometrySubsystem.backEncoder() - yi) / odometrySubsystem.odometryTick * odometrySubsystem.odometryCir;
//            mecanumSubsystem.partialMove(true, Math.copySign(power, xf - x), Math.copySign(power, (yf - y)), 0);
//        }
//        mecanumSubsystem.partialMove(true, 0, 0, 0);
//    }
//
//    public void turnToAngle(double pow, double angle) {
//
////        double xTemp = mecanumSubsystem.x;
////        double yTemp = mecanumSubsystem.y;
//
//        if (angle < 0) {
//            angle += 360;
//        }
//
//        double angleTemp = angle - odometrySubsystem.convertBearing(odometrySubsystem.theta);
//        if (angleTemp < 0) {
//            angleTemp += 360;
//        }
//
//        if (angleTemp < 180) {
//            while (Math.abs(odometrySubsystem.convertBearing(odometrySubsystem.theta) - angle) > 3 && opMode.opModeIsActive()) {
//                mecanumSubsystem.move(true, 0 , 0, pow);
//            }
//        } else {
//            while (Math.abs(odometrySubsystem.convertBearing(odometrySubsystem.theta) - angle) > 3 && opMode.opModeIsActive()) {
//                mecanumSubsystem.move(true, 0, 0, -pow);
//            }
//        }
////        while ( Math.abs(mecanumSubsystem.Theta - angle) > 3 ) {
////            mecanumSubsystem.move(true, 0, 0, directionalPow);
////        }
//        mecanumSubsystem.stop(true);
////        mecanumSubsystem.x = xTemp;
////        mecanumSubsystem.y = yTemp;
//
//    }
//
//    public void turnToAngleMixed(double pow, double angle) {
//
////        double xTemp = mecanumSubsystem.x;
////        double yTemp = mecanumSubsystem.y;
//
//        if (angle < 0) {
//            angle += 360;
//        }
//
//        double angleTemp = angle - odometrySubsystem.convertBearing(gyroOdometry.theta);
//        if (angleTemp < 0) {
//            angleTemp += 360;
//        }
//
//        if (angleTemp < 180) {
//            while (Math.abs(odometrySubsystem.convertBearing(gyroOdometry.theta) - angle) > 3 && opMode.opModeIsActive()) {
//                mecanumSubsystem.move(true, 0 , 0, pow);
//            }
//        } else {
//            while (Math.abs(odometrySubsystem.convertBearing(gyroOdometry.theta) - angle) > 3 && opMode.opModeIsActive()) {
//                mecanumSubsystem.move(true, 0, 0, -pow);
//            }
//        }
////        while ( Math.abs(mecanumSubsystem.Theta - angle) > 3 ) {
////            mecanumSubsystem.move(true, 0, 0, directionalPow);
////        }
//        mecanumSubsystem.stop(true);
////        mecanumSubsystem.x = xTemp;
////        mecanumSubsystem.y = yTemp;
//
//    }
//
//    public void turnToZeroMixed(double pow) {
//
////        double xTemp = odometrySubsystem.x;
////        double yTemp = odometrySubsystem.y;
//
//        if(odometrySubsystem.convertBearing(gyroOdometry.theta) < 0) {
//            while ((Math.abs(odometrySubsystem.convertBearing(gyroOdometry.theta)) > 5) && opMode.opModeIsActive()) {
//                mecanumSubsystem.move(true, 0, 0, -pow);
//            }
//        } else {
//            while ((Math.abs(odometrySubsystem.convertBearing(gyroOdometry.theta)) > 5) && opMode.opModeIsActive()) {
//                mecanumSubsystem.move(true, 0, 0, pow);
//            }
//        }
//        mecanumSubsystem.stop(true);
////        mecanumSubsystem.x = xTemp;
////        mecanumSubsystem.y = yTemp;
//    }
//
//    public void turnToZero(double pow) {
//
////        double xTemp = mecanumSubsystem.x;
////        double yTemp = mecanumSubsystem.y;
//
//        if(odometrySubsystem.convertBearing(odometrySubsystem.theta) < 0) {
//            while ((Math.abs(odometrySubsystem.convertBearing(odometrySubsystem.theta)) > 5) && opMode.opModeIsActive()) {
//                mecanumSubsystem.move(true, 0, 0, -pow);
//            }
//        } else {
//            while ((Math.abs(odometrySubsystem.convertBearing(odometrySubsystem.theta)) > 5) && opMode.opModeIsActive()) {
//                mecanumSubsystem.move(true, 0, 0, pow);
//            }
//        }
//        mecanumSubsystem.stop(true);
////        mecanumSubsystem.x = xTemp;
////        mecanumSubsystem.y = yTemp;
//    }

//    public void HugWallRed(boolean run, double pow) {
//
//        if (run){
//            elapsedTime.reset();
//            if (distance_sensor_right.getDistance() > distance_sensor_rightBack.getDistance()){
//                while (distance_sensor_right.getDistance() > distance_sensor_rightBack.getDistance() + 0.5 && opMode.opModeIsActive()){
//                    mecanumSubsystem.move(true, 0, 0, -pow*0.5);
//                    if (elapsedTime.time(TimeUnit.SECONDS) > 3){
//                        mecanumSubsystem.move(true, 0.5, 0, 0);
//                        opMode.sleep(500);
//                        mecanumSubsystem.move(true, 0, 0, 0);
//                        elapsedTime.reset();
//                    }
//                }
//            } else {
//                while (distance_sensor_right.getDistance() < distance_sensor_rightBack.getDistance() + 0.5 && opMode.opModeIsActive()){
//                    mecanumSubsystem.move(true, 0, 0, pow*0.5);
//                    if (elapsedTime.time(TimeUnit.SECONDS) > 3){
//                        mecanumSubsystem.move(true, 0.5, 0, 0);
//                        opMode.sleep(500);
//                        mecanumSubsystem.move(true, 0, 0, 0);
//                        elapsedTime.reset();
//                    }
//                }
//            }
//            mecanumSubsystem.Stop(true);
//            elapsedTime.reset();
//            if (distance_sensor_rightBack.getDistance() > 5.2){
//                do {
//                    mecanumSubsystem.move(true, 0, -pow, 0);
//                    if (elapsedTime.time(TimeUnit.SECONDS) > 3){
//                        mecanumSubsystem.move(true, 0.5, 0, 0);
//                        opMode.sleep(500);
//                        mecanumSubsystem.move(true, 0, 0, 0);
//                        elapsedTime.reset();
//                    }
//                } while (distance_sensor_right.getDistance() > 5.2 && opMode.opModeIsActive());
//            }
//            mecanumSubsystem.Stop(true);
//        }
//    }
//
//    public void HugWallBlue(boolean run, double pow) {
//
//        elapsedTime.reset();
//        if (run){
//            if (distance_sensor_left.getDistance() > distance_sensor_leftBack.getDistance()){
//                while (distance_sensor_left.getDistance() > distance_sensor_leftBack.getDistance() + 0.5 && opMode.opModeIsActive()){
//                    mecanumSubsystem.move(true, 0, 0, pow*0.5);
//                    if (elapsedTime.time(TimeUnit.SECONDS) > 3){
//                        mecanumSubsystem.move(true, 0.5, 0, 0);
//                        opMode.sleep(500);
//                        mecanumSubsystem.move(true, 0, 0, 0);
//                        elapsedTime.reset();
//                    }
//                }
//            } else {
//                while (distance_sensor_left.getDistance() < distance_sensor_leftBack.getDistance() + 0.5 && opMode.opModeIsActive()){
//                    mecanumSubsystem.move(true, 0, 0, -pow*0.5);
//                    if (elapsedTime.time(TimeUnit.SECONDS) > 3){
//                        mecanumSubsystem.move(true, 0.5, 0, 0);
//                        opMode.sleep(500);
//                        mecanumSubsystem.move(true, 0, 0, 0);
//                        elapsedTime.reset();
//                    }
//                }
//            }
//            mecanumSubsystem.Stop(true);
//            elapsedTime.reset();
//            if (distance_sensor_leftBack.getDistance() > 7){
//                do {
//                    mecanumSubsystem.move(true, 0, pow, 0);
//                    if (elapsedTime.time(TimeUnit.SECONDS) > 3){
//                        mecanumSubsystem.move(true, 0.5, 0, 0);
//                        opMode.sleep(500);
//                        mecanumSubsystem.move(true, 0, 0, 0);
//                        elapsedTime.reset();
//                    }
//                } while (distance_sensor_left.getDistance() > 7 && opMode.opModeIsActive());
//            }
//            mecanumSubsystem.Stop(true);
//        }
//    }

//    public void ToCoordVel(boolean run, ArrayList<VectorCartesian> vtGraph, double xf, double yf){
//        if (run){
//            followVelocity(MecanumSubsystem.vtVector);
//        }
//
//    }
//
//    private void followVelocity(boolean run, VectorCartesian vtVector){
//        mecanumSubsystem.getLeftBack().setVelocity(pidController.velocityOfRobotToWheel(PIDController.Wheel.leftBa  ck, magnitude, vtVector.theta));
//        mecanumSubsystem.getRightBack().setVelocity();
//        mecanumSubsystem.getLeftForward().setVelocity();
//        mecanumSubsystem.getRightForward().setVelocity();
//    }

//    public void toCoordOdometry(boolean run, double pow, double xf, double yf){
//        if (run){
//            double dx = xf - odometrySubsystem.x;
//            double dy = yf - odometrySubsystem.y;
//            double theta = odometrySubsystem.theta;
//            while (theta > 2*Math.PI && opMode.opModeIsActive()){
//                theta = theta - (2*Math.PI);
//            }
//            while (theta < 0 && opMode.opModeIsActive()){
//                theta = theta + (2*Math.PI);
//            }
//            double angle = Math.atan2(dy, dx)- (2*Math.PI) + theta;
//            while (angle < 0 && opMode.opModeIsActive()){
//                angle = angle + (2*Math.PI);
//            }
//            double temp = Math.tan(angle);
////            Log.d("tempLogs", "Current ratio: " + Double.toString(temp));
//            Log.d("Auto2Logs", "Angle from endpoint: " + angle);
//
//            if (angle < Math.PI){
//                if (angle < Math.PI/2){
//                    if (angle <= Math.PI/4){
//                        //Works
////                        mecanumSubsystem.move(true, pow, pow*temp, 0);
//                        mecanumSubsystem.move(true, pow, pow*temp, 0);
//                    } else {
//                        //Works
//                        temp = 1/temp;
////                        mecanumSubsystem.move(true, pow*temp, pow, 0);
//                        mecanumSubsystem.move(true, pow*temp, pow, 0);
//                    }
//                } else {
//
//                    if (angle <= 3*Math.PI/4){
//                        //Works
//                        temp = 1/temp;
////                        mecanumSubsystem.move(true, pow*temp, pow, 0);
//                        mecanumSubsystem.move(true, pow*temp, pow, 0);
//                    } else {
//                        //Works
////                        mecanumSubsystem.move(true, -pow, -pow*temp, 0);
//                        mecanumSubsystem.move(true, -pow, -pow*temp, 0);
//                    }
//                }
//            } else {
//                if (angle < 3*Math.PI/2){
//                    //Works
//                    if (angle <= 5*Math.PI/4){
////                        mecanumSubsystem.move(true, -pow, -pow*temp, 0);
//                        mecanumSubsystem.move(true, -pow, -pow*temp, 0);
//                    } else {
//                        temp = 1/temp;
////                        mecanumSubsystem.move(true, -pow*temp, -pow, 0);
//                        mecanumSubsystem.move(true, -pow*temp, -pow, 0);
//                    }
//                } else {
//                    if (angle <= 7*Math.PI/4){
//                        //Works
//                        temp = 1/temp;
////                        mecanumSubsystem.move(true, -pow*temp, -pow, 0);
//                        mecanumSubsystem.move(true, -pow*temp, -pow, 0);
//                    } else {
////                        mecanumSubsystem.move(true, pow, pow*temp, 0);
//                        mecanumSubsystem.move(true, pow, pow*temp, 0);
//
//                    }
//                }
//            }
//        }
//    }
//
//    public void toCoordMixed(boolean run, double pow, double xf, double yf){
//        if (run){
//            double dx = xf - gyroOdometry.x;
//            double dy = yf - gyroOdometry.y;
//            double theta = gyroOdometry.theta;
//            while (theta > 2*Math.PI && opMode.opModeIsActive()){
//                theta = theta - (2*Math.PI);
//            }
//            while (theta < 0 && opMode.opModeIsActive()){
//                theta = theta + (2*Math.PI);
//            }
//            double angle = Math.atan2(dy, dx)- (2*Math.PI) + theta;
//            while (angle < 0 && opMode.opModeIsActive()){
//                angle = angle + (2*Math.PI);
//            }
//            double temp = Math.tan(angle);
////            Log.d("tempLogs", "Current ratio: " + Double.toString(temp));
//            Log.d("Auto2Logs", "Angle from endpoint: " + angle);
//
//            if (angle < Math.PI){
//                if (angle < Math.PI/2){
//                    if (angle <= Math.PI/4){
//                        //Works
////                        mecanumSubsystem.move(true, pow, pow*temp, 0);
//                        mecanumSubsystem.move(true, pow, pow*temp, 0);
//                    } else {
//                        //Works
//                        temp = 1/temp;
////                        mecanumSubsystem.move(true, pow*temp, pow, 0);
//                        mecanumSubsystem.move(true, pow*temp, pow, 0);
//                    }
//                } else {
//
//                    if (angle <= 3*Math.PI/4){
//                        //Works
//                        temp = 1/temp;
////                        mecanumSubsystem.move(true, pow*temp, pow, 0);
//                        mecanumSubsystem.move(true, pow*temp, pow, 0);
//                    } else {
//                        //Works
////                        mecanumSubsystem.move(true, -pow, -pow*temp, 0);
//                        mecanumSubsystem.move(true, -pow, -pow*temp, 0);
//                    }
//                }
//            } else {
//                if (angle < 3*Math.PI/2){
//                    //Works
//                    if (angle <= 5*Math.PI/4){
////                        mecanumSubsystem.move(true, -pow, -pow*temp, 0);
//                        mecanumSubsystem.move(true, -pow, -pow*temp, 0);
//                    } else {
//                        temp = 1/temp;
////                        mecanumSubsystem.move(true, -pow*temp, -pow, 0);
//                        mecanumSubsystem.move(true, -pow*temp, -pow, 0);
//                    }
//                } else {
//                    if (angle <= 7*Math.PI/4){
//                        //Works
//                        temp = 1/temp;
////                        mecanumSubsystem.move(true, -pow*temp, -pow, 0);
//                        mecanumSubsystem.move(true, -pow*temp, -pow, 0);
//                    } else {
////                        mecanumSubsystem.move(true, pow, pow*temp, 0);
//                        mecanumSubsystem.move(true, pow, pow*temp, 0);
//
//                    }
//                }
//            }
//        }
//    }

//    //important
//    public void toPosPIDGyroOdometry(double vel, double xf, double yf, double thetaf){
//        globalXController.integralReset();
//        globalYController.integralReset();
//        globalThetaController.integralReset();
//        while ((Math.abs(xf-gyroOdometry.x) > 5 || Math.abs(yf-gyroOdometry.y) > 5 || Math.abs(thetaf-gyroOdometry.theta) > 0.05)&&opMode.opModeIsActive()){
//            if (Math.abs(xf-gyroOdometry.x) > 2){
//                ex = globalXController.outputPositional(xf, gyroOdometry.x);
//            } else {
//                ex = 0;
//            }
//            if (Math.abs(yf - gyroOdometry.y) > 2){
//                ey = globalYController.outputPositional(yf, gyroOdometry.y);
//            } else {
//                ey = 0;
//            }
//            if (Math.abs(thetaf - gyroOdometry.theta) > 0.02){
//                etheta = globalThetaController.outputPositional(thetaf, gyroOdometry.theta);
//            } else {
//                etheta = 0;
//            }
//            if (Math.abs(ex) > vel || Math.abs(ey) > vel){
//                double max = Math.max(Math.abs(ex), Math.abs(ey));
//                ex = ex / max * vel;
//                ey = ey / max * vel;
//                etheta = etheta / max * vel;
//            }
//            moveGlobalPartial(true, ex, ey, etheta);
//        }
//        moveGlobalPartial(true, 0, 0, 0);
//    }
//
//    public void toPosVelPIDGyroOdometry(double vel, double xf, double yf, double thetaf){
//        globalXController.integralReset();
//        globalYController.integralReset();
//        globalThetaController.integralReset();
//        while ((Math.abs(xf-gyroOdometry.x) > 5 || Math.abs(yf-gyroOdometry.y) > 5 || Math.abs(thetaf-gyroOdometry.theta) > 0.05)&&opMode.opModeIsActive()){
//            if (Math.abs(xf-gyroOdometry.x) > 2){
//                ex = globalXController.outputPositional(xf, gyroOdometry.x);
//            } else {
//                ex = 0;
//            }
//            if (Math.abs(yf - gyroOdometry.y) > 2){
//                ey = globalYController.outputPositional(yf, gyroOdometry.y);
//            } else {
//                ey = 0;
//            }
//            if (Math.abs(thetaf - gyroOdometry.theta) > 0.02){
//                etheta = globalThetaController.outputPositional(thetaf, gyroOdometry.theta);
//            } else {
//                etheta = 0;
//            }
//            if (ex > vel || ey > vel){
//                double max = Math.max(ex, ey);
//                ex = ex / max * vel;
//                ey = ey / max * vel;
//                etheta = etheta / max * vel;
//            }
//            ex = globalVXController.outputPositional(ex, gyroOdometry.vxGlobal);
//            ey = globalVXController.outputPositional(ey, gyroOdometry.vyGlobal);
//            etheta = globalVXController.outputPositional(etheta, gyroOdometry.vTheta);
//            moveGlobalPartial(true, ex, ey, etheta);
//        }
//        moveGlobalPartial(true, 0, 0, 0);
//    }
//
//    public void toPosPIDGyroOdometryPreciseNoTheta(double vel, double xf, double yf){
//        double ex;
//        double ey;
//        double etheta;
//        globalXController.integralReset();
//        globalYController.integralReset();
//        globalThetaController.integralReset();
//        while ((Math.abs(xf-gyroOdometry.x) > 1 || Math.abs(yf-gyroOdometry.y) > 1 &&opMode.opModeIsActive())){
//            if (Math.abs(xf-gyroOdometry.x) > 1){
//                ex = globalXController.outputPositional(xf, gyroOdometry.x);
//            } else {
//                ex = 0;
//            }
//            if (Math.abs(yf - gyroOdometry.y) > 1){
//                ey = globalYController.outputPositional(yf, gyroOdometry.y);
//            } else {
//                ey = 0;
//            }
//            if (ex > vel || ey > vel){
//                double max = Math.max(ex, ey);
//                ex = ex / max * vel;
//                ey = ey / max * vel;
//            }
//            moveGlobalPartial(true, ex, ey, 0);
//        }
//        moveGlobalPartial(true, 0, 0, 0);
//    }
//
//    public void toPosPIDGyroOdometryPrecise(double error, double vel, double xf, double yf, double thetaf){
//        double ex;
//        double ey;
//        double etheta;
//        globalXController.integralReset();
//        globalYController.integralReset();
//        globalThetaController.integralReset();
//        while ((Math.abs(xf-gyroOdometry.x) > error || Math.abs(yf-gyroOdometry.y) > error || Math.abs(thetaf-gyroOdometry.theta) > 0.02)&&opMode.opModeIsActive()){
//            if (Math.abs(xf-gyroOdometry.x) > error){
//                ex = globalXController.outputPositional(xf, gyroOdometry.x);
//            } else {
//                ex = 0;
//            }
//            if (Math.abs(yf - gyroOdometry.y) > error){
//                ey = globalYController.outputPositional(yf, gyroOdometry.y);
//            } else {
//                ey = 0;
//            }
//            if (Math.abs(thetaf - gyroOdometry.theta) > 0.005){
//                etheta = globalThetaController.outputPositional(thetaf, gyroOdometry.theta);
//            } else {
//                etheta = 0;
//            }
//            if (ex > vel || ey > vel){
//                double max = Math.max(ex, ey);
//                ex = ex / max * vel;
//                ey = ey / max * vel;
//                etheta = etheta / max * vel;
//            }
//            moveGlobalPartial(true, ex, ey, etheta);
//        }
//        moveGlobalPartial(true, 0, 0, 0);
//    }
//
//    public void toPosPIDGyroOdometryPrecise(double error, double vel, double xf, double yf, double thetaf, double maxTime){
//        double ex;
//        double ey;
//        double etheta;
//        globalXController.integralReset();
//        globalYController.integralReset();
//        globalThetaController.integralReset();
//        elapsedTime.reset();
//        while ((Math.abs(xf-gyroOdometry.x) > error || Math.abs(yf-gyroOdometry.y) > error || Math.abs(thetaf-gyroOdometry.theta) > 0.02) && elapsedTime.time() < maxTime && opMode.opModeIsActive()){
//            if (Math.abs(xf-gyroOdometry.x) > error){
//                ex = globalXController.outputPositional(xf, gyroOdometry.x);
//            } else {
//                ex = 0;
//            }
//            if (Math.abs(yf - gyroOdometry.y) > error){
//                ey = globalYController.outputPositional(yf, gyroOdometry.y);
//            } else {
//                ey = 0;
//            }
//            if (Math.abs(thetaf - gyroOdometry.theta) > 0.005){
//                etheta = globalThetaController.outputPositional(thetaf, gyroOdometry.theta);
//            } else {
//                etheta = 0;
//            }
//            if (ex > vel || ey > vel){
//                double max = Math.max(ex, ey);
//                ex = ex / max * vel;
//                ey = ey / max * vel;
//                etheta = etheta / max * vel;
//            }
//            moveGlobalPartial(true, ex, ey, etheta);
//        }
//        moveGlobalPartial(true, 0, 0, 0);
//    }
//
//    public void toPosPIDGyroOdometryPreciseErrorTheta(double time, double error, double vel, double xf, double yf, double thetaf){
//        double ex;
//        double ey;
//        double etheta;
//        globalXController.integralReset();
//        globalYController.integralReset();
//        globalThetaController.integralReset();
//        while ((Math.abs(xf-gyroOdometry.x) > error || Math.abs(yf-gyroOdometry.y) > error || Math.abs(thetaf-gyroOdometry.theta) > 0.02)&&opMode.opModeIsActive() && positionTime < time){
//            if (Math.abs(xf-gyroOdometry.x) > error){
//                ex = globalXController.outputPositional(xf, gyroOdometry.x);
//            } else {
//                ex = 0;
//            }
//            if (Math.abs(yf - gyroOdometry.y) > error){
//                ey = globalYController.outputPositional(yf, gyroOdometry.y);
//            } else {
//                ey = 0;
//            }
//            if (Math.abs(thetaf - gyroOdometry.theta) > 0.02){
//                etheta = globalThetaController.outputPositional(thetaf, gyroOdometry.theta);
//            } else {
//                etheta = 0;
//            }
//            if (ex > vel || ey > vel){
//                double max = Math.max(ex, ey);
//                ex = ex / max * vel;
//                ey = ey / max * vel;
//                etheta = etheta / max * vel;
//            }
//            moveGlobalPartial(true, ex, ey, etheta);
//        }
//        moveGlobalPartial(true, 0, 0, 0);
//    }

    public void purePursuit(){
        if (runPurePursuit){
            currentTargetPoint = purePursuit.getTarget(gyroOdometry.y, gyroOdometry.x);
            setFinalPosition(true, velocity, currentTargetPoint.y, currentTargetPoint.x, thetaFinal);
        }
    }

    public void setFinalPosition(boolean run, double velocity, double x, double y, double theta){
        if (run){
            xFinal = x;
            yFinal = y;
            thetaFinal = theta;
            this.velocity = velocity;
        }
    }

    public void startPurePursuit(double velocity){
        runPurePursuit = true;
        this.velocity = velocity;
    }

    public void stopPurePursuit(){
        runPurePursuit = false;
    }

    public boolean isPurePursuitRunning(){
        return runPurePursuit;
    }

    public void nextPoint(){
        purePursuit.nextPoint();
    }

    public boolean isFinalPositionReached(){
        return Math.abs(gyroOdometry.x - purePursuit.finalPoints.get(purePursuit.currentFinalTargetPoint).getX()) < 2 && Math.abs(gyroOdometry.y - purePursuit.finalPoints.get(purePursuit.currentFinalTargetPoint).getY()) < 2;
    }

    public double getXFinal() {
        return xFinal;
    }

    public double getYFinal() {
        return yFinal;
    }

    public double getThetaFinal() {
        return thetaFinal;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getXDifference(){
        return Math.abs(xFinal - gyroOdometry.x);
    }

    public double getYDifference(){
        return Math.abs(yFinal - gyroOdometry.y);
    }

    public double getThetaDifference(){
        return Math.abs(thetaFinal - gyroOdometry.theta);
    }

    public boolean isPositionPassed(){
        return getXDifference() < 5 && getYDifference() < 5 && getThetaDifference() < 0.05;
    }

    public boolean isPositionReached(){
        return getXDifference() < 1 && getYDifference() < 1 && getThetaDifference() < 0.02;
    }

    public boolean isCoordinateReached(){
        return getXDifference() < 2.5 && getYDifference() < 2.5;
    }

    public boolean isCoordinatePassed(){
        return getXDifference() < 5 && getYDifference() < 5;
    }

    public boolean isYReached(){
        return getYDifference() < 1;
    }

    public boolean isYPassed(){
        return getYDifference() < 5;
    }

    public boolean isXReached(){
        return getXDifference() < 1;
    }

    public boolean isXPassed(){
        return getXDifference() < 3;
    }

    public boolean isInGeneralVicinity() {
        return getXDifference() < 10 && getYDifference() < 10 && getThetaDifference() < 0.2;
    }
}