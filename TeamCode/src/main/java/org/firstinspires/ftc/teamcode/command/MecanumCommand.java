package org.firstinspires.ftc.teamcode.command;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.PIDCore;

@Config
public class MecanumCommand {
    private MecanumSubsystem mecanumSubsystem;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private boolean run;
    private ElapsedTime elapsedTime;
    public PIDCore globalXController;
    public PIDCore globalYController;
    public PIDCore globalThetaController;
    private static double kpx = 0.07;
    private static double kdx = 0.01;
    private static double kix = 0.0075/2;
    private static double kpy = 0.055;
    private static double kdy = 0.0005;
    private static double kiy = 0.0075/2;
    private static double kptheta = 2;
    private static double kdtheta = 0.05;
    private static double kitheta = 0.0;
    private double ex = 0;
    private double ey = 0;
    private double etheta = 0;
    private double xFinal;
    private double yFinal;
    private double thetaFinal;
    private double velocity;

    private LinearOpMode opMode;

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

    public MecanumCommand(MecanumSubsystem mecanumSubsystem, OdometrySubsystem odometrySubsystem, GyroOdometry gyroOdometry, LinearOpMode opMode) {
        this.mecanumSubsystem = mecanumSubsystem;
        this.odometrySubsystem = odometrySubsystem;
        this.gyroOdometry = gyroOdometry;
        this.opMode = opMode;
        globalXController = new PIDCore(kpx, kdx, kix);
        globalYController = new PIDCore(kpy, kdy, kiy);
        globalThetaController = new PIDCore(kptheta, kdtheta, kitheta);
        elapsedTime = new ElapsedTime();
        xFinal = gyroOdometry.x;
        yFinal = gyroOdometry.y;
        thetaFinal = gyroOdometry.theta;
        velocity = 0;
    }
    public void turnOffInternalPID(){
        mecanumSubsystem.turnOffInternalPID();
    }

    public void pidProcess(){
        ex = -globalXController.outputPositional(xFinal, gyroOdometry.x);
        ey = -globalYController.outputPositional(yFinal, gyroOdometry.y);
        etheta = globalThetaController.outputPositional(thetaFinal, gyroOdometry.theta);
        if (Math.abs(ex) > velocity || Math.abs(ey) > velocity){
            double max = Math.max(Math.abs(ex), Math.abs(ey));
            ex = ex / max * velocity;
            ey = ey / max * velocity;
            etheta = etheta / max * velocity;
        }
        moveGlobalPartial(true, ex, ey, etheta);
    }

//    public void pidProcessPurePursuit(){
//        e = globalPositionController.outputPositional(Math.sqrt(Math.pow((purePursuit.getPoints().get(purePursuit.getCurrentTargetPoint()).getX() - gyroOdometry.x), 2) + Math.pow((purePursuit.getPoints().get(purePursuit.getCurrentTargetPoint()).getY() - gyroOdometry.y), 2)));
//        heading = Math.atan2(xFinal - gyroOdometry.x, yFinal - gyroOdometry.y);
//        ex = e * Math.sin(heading);
//        ey = e * Math.cos(heading);
//        etheta = globalThetaController.outputPositional(thetaFinal, gyroOdometry.theta);
//        if (Math.abs(ex) > velocity || Math.abs(ey) > velocity){
//            double max = Math.max(Math.abs(ex), Math.abs(ey));
//            ex = ex / max * velocity;
//            ey = ey / max * velocity;
//            etheta = etheta / max * velocity;
//        }
//        moveGlobalPartial(true, ex, ey, etheta);
//    }

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

    // this method is designed to be used in an autonomous (linear) opmode
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
//
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

    public void moveIntegralReset(){
        globalYController.integralReset();
        globalXController.integralReset();
        globalThetaController.integralReset();
    }


    public void moveToGlobalPosition(double targetX, double targetY, double targetTheta) throws InterruptedException { //When calling stop function, does interrupted exception happen?
        globalYController.integralReset();
        globalXController.integralReset();
        globalThetaController.integralReset();

        // stop moving if within 5 ticks or 0.2 radians from the position
    }

    public void moveToGlobalPos(double targetX, double targetY, double targetTheta){
         //if within 0.15 radians of target position
            double moveX = globalXController.outputPositional(targetX, gyroOdometry.x);
            double moveY = globalYController.outputPositional(targetY, gyroOdometry.y);
            double moveTheta = -globalThetaController.outputPositional(targetTheta, gyroOdometry.theta);
            // moveX = -globalXController.outputPositional(targetX, gyroOdometry.x);
            /*
            globalYController is set to neg bc if pos, it will adjust in the wrong way, increasing
            distance away, resulting in moving left infinitely
             */
        //-globalYController.outputPositional(targetY, gyroOdometry.y);
            //double moveTheta = -globalThetaController.outputPositional(targetTheta, gyroOdometry.theta); // fieldOriented theta values set to opposite1
            mecanumSubsystem.fieldOrientedMove(moveX, moveY, moveTheta,gyroOdometry.theta);
    }

    /**
     * moveToGlobalPosition except only a passby. Longer the movement the less accurate it will be.
     *
     * @param targetX target x pos in cm
     * @param targetY target y pos in cm
     * @param targetTheta target angle in rad
     * @param moveTolerance percent tolerance of movement length (e.g. if movement = 10cm with percentTolerance 0.2, tolerance is 2cm)
     */
    public void moveToCheckpoint(double targetX, double targetY, double targetTheta, double moveTolerance) throws InterruptedException{
        // stop moving if within 5 ticks or 0.2 radians from the position
        globalYController.integralReset();
        globalXController.integralReset();
        globalThetaController.integralReset();

        //calculate tolerances
        double minTickTolerance = 5; // basically the absolute lowest tolerance
        double maxTickTolerance = 15;
        double tolX = Math.max(Math.min(moveTolerance*gyroOdometry.x, maxTickTolerance), minTickTolerance); //clamp tolerance to within 15 - 5
        double tolY = Math.max(Math.min(moveTolerance*gyroOdometry.y, maxTickTolerance), minTickTolerance); //clamp tolerance to within 15 - 5


        // stop moving if within 5 ticks or 0.2 radians from the position
        while (Math.abs(targetX - gyroOdometry.x) > tolX   //if within 2.5 ticks of target X position
                || Math.abs(targetY - gyroOdometry.y) > tolY//if within 2.5 ticks of target y position
                || Math.abs(targetTheta - gyroOdometry.theta) > 0.2) { //if within 0.15 radians of target position

            double moveX = globalXController.outputPositional(targetX, gyroOdometry.x);
            double moveY = globalYController.outputPositional(targetY, gyroOdometry.y);
            double moveTheta = globalThetaController.outputPositional(targetTheta, gyroOdometry.theta);
            mecanumSubsystem.fieldOrientedMove(moveX, moveY, moveTheta, gyroOdometry.theta);
        }
        mecanumSubsystem.stop(true);
    }

    public boolean isPositionReached(boolean xtol, boolean ytol){
        boolean withinXRange = (getXDifference() < 1.5);
        if (xtol){
            withinXRange = (getXDifference() < 3);
        }
        boolean withinYRange = (getYDifference() < 1.5);
        if (ytol){
            withinXRange = (getXDifference() < 3);
        }
        return withinXRange && withinYRange && getThetaDifference() < 0.02;
    }

    public boolean NothingToDoCrashPrevention(){
        //TODO: Extremely experimental! Do not use right now
            // Example sensor readings - replace with actual sensor code
            double frontSensorDistance = getFrontSensorDistance();
            double safeDistance = 10; // 10 cm as a safe distance - adjust as needed

            // Check if there's an obstacle too close in front
            if (frontSensorDistance < safeDistance) {
                // Stop or slow down the robot
                mecanumSubsystem.stop(true);
                return true; // Indicate that crash prevention was triggered
            }

            // No obstacles detected within the safe distance
            return false;
        }

        private double getFrontSensorDistance() {
            // Placeholder for actual sensor reading code
            // Replace this with code to read distance code
            return Integer.MAX_VALUE; // Example value, replace with actual sensor reading
        }



    public void moveRotation(double targetTheta) {
        // stop moving if within 5 ticks or 0.2 radians from the position
        while (Math.abs(targetTheta - odometrySubsystem.theta) > 0.1) {

            mecanumSubsystem.fieldOrientedMove(
                    0, 0,
                    -globalThetaController.outputPositional(targetTheta, odometrySubsystem.theta),
                    0);
        }
        mecanumSubsystem.stop(true);
    }

    public void setFinalPosition(boolean run, double velocity, double x, double y, double theta){
        if (run){
            xFinal = x;
            yFinal = y;
            thetaFinal = theta;
            this.velocity = velocity;
        }
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