package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.ArrayList;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicReference;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp (name = "AprilTagCameraTest", group = "Tests")
public class AprilTagCameraTest extends LinearOpMode {

    AprilCamSubsystem aprilCamSubsystem;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;
    private OpenCvCamera webcam;
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;

    @Override
    public void runOpMode() throws InterruptedException {
        aprilCamSubsystem = new AprilCamSubsystem(hardwareMap);

        packet = new TelemetryPacket();
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()) {
            aprilCamSubsystem.runDetections();
            if(aprilCamSubsystem.getDetections().size() > 0) {
                ArrayList<AprilTagDetection> detections = aprilCamSubsystem.getDetections();
                telemetry.addData("Detections", detections);

                //int closestnum = detections.get(0).id;
                /*
                for (int i = 0; i < detections.size(); i++) {

                }
                if(){
                    double targetX = gyroOdometry.x + detections.get(i).ftcPose.x;
                    double targetY =  gyroOdometry.y + detections.get(i).ftcPose.y;
                    double targetTheta = math.pi/2;
                    mecanumCommand.setFinalPosition(true,30 , targetX, targetY, targetTheta);
                    while(!mecanumCommand.isCoordinatePassed()) {};
                }
                else{

                }
                */

                for (int i = 0; i < detections.size(); i++) {
                    telemetry.addData("x" + i, detections.get(i).ftcPose.x);
                    telemetry.addData("y" + i, detections.get(i).ftcPose.y);
                    telemetry.addData("z" + i, detections.get(i).ftcPose.z);
                    telemetry.addData("yaw" + i, detections.get(i).ftcPose.yaw);
                    telemetry.addData("pitch" + i, detections.get(i).ftcPose.pitch);
                    telemetry.addData("roll" + i, detections.get(i).ftcPose.roll);
                }
            }
            telemetry.update();
        }

    }

}
