package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.threadopmode.subsystems.WebcamSubsystem;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

public class CameraTest extends LinearOpMode {
    private WebcamSubsystem camera;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    @Override
    public void runOpMode(){
        camera = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.APRIL_TAG);
        dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(camera.webcam, 30);
        packet = new TelemetryPacket();
        ArrayList<AprilTagDetection> detections = camera.aprilTagPipeline.getDetectionsUpdate();

        waitForStart();
        while(opModeIsActive()){
            dashboard.sendTelemetryPacket(packet);

            telemetry.addData("detections", detections.size());

            sleep(20);
        }
    }
}
