package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

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
    FtcDashboard dashboard;
    TelemetryPacket packet;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);

        mecanumCommand.turnOffInternalPID();
        imu.resetAngle();
        odometrySubsystem.reset();

        dashboard = FtcDashboard.getInstance();

        waitForStart();

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);

//        sleep(8000);
        mecanumCommand.moveToGlobalPosition(100, 100, 0);
        sleep(4000);
        mecanumCommand.moveToGlobalPosition(100, 100, Math.PI);
        sleep(4000);
        mecanumCommand.moveToGlobalPosition(100, 100, 2*Math.PI);

    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            telemetry.addData("x", gyroOdometry.x);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}