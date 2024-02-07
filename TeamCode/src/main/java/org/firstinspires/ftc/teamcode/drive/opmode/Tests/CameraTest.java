package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;


@TeleOp(name="camera test")
public class CameraTest extends LinearOpMode {
    //https://docs.wpilib.org/en/stable/docs/software/vision-processing/grip/introduction-to-grip.html
    private WebcamSubsystem camera;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;


    @Override
    public void runOpMode(){
        camera = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_RED);
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera.webcam, 24);
        packet = new TelemetryPacket();

        waitForStart();
        while(opModeIsActive()){
            dashboard.sendTelemetryPacket(packet);
            packet.put("x", camera.getXProp());
            packet.put("y", camera.getContourProcessor().largestContourCenter().y);
            packet.put("area", camera.getContourProcessor().largestContourArea());
            telemetry.addData("x", camera.getXProp());
            telemetry.addData("y", camera.getContourProcessor().largestContourCenter().y);
            telemetry.addData("area", camera.getContourProcessor().largestContourArea());
            sleep(20);
        }
    }
}
