package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name="camera test")
public class CameraTest extends LinearOpMode {
    private WebcamSubsystem camera;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    @Override
    public void runOpMode(){
        camera = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR);
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera.webcam, 30);
        packet = new TelemetryPacket();

        waitForStart();
        while(opModeIsActive()){
            dashboard.sendTelemetryPacket(packet);

            sleep(20);
        }
    }
}
