package org.firstinspires.ftc.teamcode.drive.opmode;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Autonomous Back Red")
public class AutonomousBackRed extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private IntakeCommand intakeCommand;
    private WebcamSubsystem webcamSubsystem;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;


    @Override
    public void runOpMode() throws InterruptedException {
        //contour location before 60
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        intakeCommand = new IntakeCommand(hardwareMap);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        timer = new ElapsedTime();

        mecanumCommand.turnOffInternalPID();
        imu.resetAngle();
        odometrySubsystem.reset();

        dashboard = FtcDashboard.getInstance();

        intakeCommand.raiseIntake();
        waitForStart();
        odometrySubsystem.reset();

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);

        double propPosition = 0;
        timer.reset();
        while(opModeInInit()) {
            propPosition = webcamSubsystem.getXProp();
        }
        timer.reset();
        while (timer.time() > 3500) {
            mecanumCommand.moveToGlobalPosition(-95, 0, 0);
            sleep(500);
            mecanumCommand.moveToGlobalPosition(-95, 0, -1.1);
            sleep(1000);
        }
        //        sleep(8000);
        /*
        if(propPosition > 175){ //if middle
            //middle
            mecanumCommand.moveToGlobalPosition(-125, 0, 0);
            sleep(1000);
        }
        else if(propPosition <= 175 && propPosition > 0){  // if right
            //right
            mecanumCommand.moveToGlobalPosition(-95, 0, 0);
            sleep(500);
            mecanumCommand.moveToGlobalPosition(-95, 0, -1.1);
            sleep(1000);
        }
        else{ //if left
            position = "left";
            mecanumCommand.moveToGlobalPosition(-107, 0, 0);
            sleep(500);
            mecanumCommand.moveToGlobalPosition(-107, 0, 0.8);
            sleep(500);
        }
//        mecanumCommand.moveToGlobalPosition(65, -3.5, 0);
        sleep(3000);
         */




        timer.reset();
        while(timer.milliseconds() < 2500) {
            intakeCommand.intakeOut(0.5);
        }
        intakeCommand.stopIntake();


    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("xprop", webcamSubsystem.getXProp());
            telemetry.update();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            packet.put("x", gyroOdometry.x);
            packet.put("y", gyroOdometry.y);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            packet.put("x", gyroOdometry.x);
            packet.put("y", gyroOdometry.y);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
}