package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="coordinate testing")
public class CoordinateTesting extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private AprilCamSubsystem aprilCamSubsystem;
    private ArrayList<AprilTagDetection> detections;
    private HashMap<Integer, AprilTagDetection> detectionMap;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private double targetX = 0;
    private double targetY = 0;

    private boolean goToAprilTag = true;

    private Integer aprilID = 2;


    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);


        mecanumCommand.turnOffInternalPID();
        imu.resetAngle();
        odometrySubsystem.reset();

        dashboard = FtcDashboard.getInstance();

        waitForStart();
        odometrySubsystem.reset();

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);

//        sleep(8000);
//        mecanumCommand.moveRotation(Math.PI);
        mecanumCommand.moveToGlobalPosition(0, 0, Math.PI);
        sleep(4000);
//        mecanumCommand.moveToGlobalPosition(100, 100, Math.PI);
//        sleep(4000);
//        mecanumCommand.moveToGlobalPosition(100, 100, 2*Math.PI);

    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
            telemetry.addData("theta", gyroOdometry.theta);
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            packet.put("x", gyroOdometry.x);
            packet.put("y", gyroOdometry.y);
            packet.put("output", mecanumCommand.globalXController.getOutputPositionalValue());
            packet.put("y output", mecanumCommand.globalYController.getOutputPositionalValue());
            packet.put("theta", gyroOdometry.theta);
            packet.put("theta output", mecanumCommand.globalThetaController.getOutputPositionalValue());
            packet.put("lfvel", mecanumSubsystem.lfvel);
            packet.put("lbvel", mecanumSubsystem.lbvel);
            packet.put("rfvel", mecanumSubsystem.rfvel);
            packet.put("rbvel", mecanumSubsystem.rbvel);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("xintegral", mecanumCommand.globalXController.getIntegralSum());
            telemetry.addData("output", mecanumCommand.globalXController.getOutputPositionalValue());
            telemetry.addData("apriltagYdistance", aprilCamSubsystem.getAprilYDistance(1, 0));
            telemetry.addData("apriltagXdistance", aprilCamSubsystem.getAprilXDistance(1, 0));
            telemetry.addData("targetX", targetX);
            telemetry.addData("targetY", targetY);
            telemetry.update();
            dashboard.sendTelemetryPacket(packet);
        }
    }

    public void tagDetectionProcess(){
        while(opModeIsActive()) {
            aprilCamSubsystem.runDetections();
        }
    }

    public void setTagTargets(){
        while(opModeIsActive() && !isStopRequested()) {
            if(goToAprilTag) {
                if(aprilCamSubsystem.getHashmap().containsKey(aprilID)) {
                    targetY = gyroOdometry.y + (-1) * aprilCamSubsystem.getAprilXDistance(aprilID, 20);
                    moveToPos(0, targetY, 0, 2.5, 2.5,0.05 );
                }
                sleep(5000);
                if(aprilCamSubsystem.getHashmap().containsKey(aprilID)){
                    targetX = gyroOdometry.x + (-1)* aprilCamSubsystem.getAprilYDistance(aprilID, 0);
                    moveToPos(targetX, targetY, 0, 2.5,2.5,0);
                }
                sleep(5000);
            }
        }
    }

    public void moveToPos(double x, double y, double theta, double toleranceX, double toleranceY, double toleranceTheta){
        mecanumCommand.moveIntegralReset();
        // stop moving if within 5 ticks or 0.2 radians from the position
        while ((Math.abs(x - gyroOdometry.x) > toleranceX  //if within 2.5 ticks of target X position
                || Math.abs(y - gyroOdometry.y) > toleranceY //if within 2.5 ticks of target y position
                || Math.abs(theta - gyroOdometry.theta) > toleranceTheta)
                && this.opModeIsActive() && !this.isStopRequested()) {
            mecanumCommand.moveToGlobalPos(x, y, theta);
        }
        mecanumSubsystem.stop(true);
    }

}