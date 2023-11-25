package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp(name="Reading")
public class Reading extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private GyroOdometry odo;
    private IMUSubsystem imu;
    private FtcDashboard dash;
    private TelemetryPacket packet;

    @Override
    public void runOpMode() throws InterruptedException {
        odo = new GyroOdometry(hardwareMap);
        imu = new IMUSubsystem(hardwareMap);

        frontLeft = hardwareMap.get(DcMotor.class, "leftForward");
        frontRight = hardwareMap.get(DcMotor.class, "rightForward");
        backLeft = hardwareMap.get(DcMotor.class, "leftBack");
        backRight = hardwareMap.get(DcMotor.class, "rightBack");

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.resetAngle();
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // allows telemetry to output to phone and dashboard

        odo.reset();
        waitForStart();
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.N) {
            Executor executor = Executors.newFixedThreadPool(4);
            CompletableFuture.runAsync(this::updateOdometry, executor);
        }

        while (opModeIsActive()) {
            if(gamepad1.a){
                frontLeft.setPower(1);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
            else if(gamepad1.b){
                frontRight.setPower(1);
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
            else if(gamepad1.y){
                backLeft.setPower(1);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);
            }
            else if(gamepad1.x){
                backRight.setPower(1);
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
            }
            else{
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }
            // reporting turret data
            telemetry.addLine();
            telemetry.addData("x", odo.getXPos());
            telemetry.addData("y", odo.getYPos());
//            telemetry.addData("heading", imu.getHeadingRAD());
            telemetry.addData("odoHeading", odo.getHeading());
            telemetry.addData("leftEncoder", odo.getLeftTicks());
            telemetry.addData("rightEncoder", odo.getRightTicks());
            telemetry.addData("auxEncoder", odo.getAuxTicks());
            packet.put("leftEncoder", odo.getLeftTicks());
            packet.put("rightEncoder", odo.getRightTicks());
            packet.put("auxEncoder", odo.getAuxTicks());
            packet.put("frontLeft", frontLeft.getCurrentPosition());
            packet.put("frontRight", frontRight.getCurrentPosition());
            packet.put("backLeft", backLeft.getCurrentPosition());
            packet.put("backRight", backRight.getCurrentPosition());
            packet.put("x", odo.getXPos());
            packet.put("y", odo.getYPos());
            packet.put("heading", imu.getTheta());
            telemetry.addData("odoHeading", odo.getHeading());
            dash.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void updateOdometry() {
        while (opModeIsActive()) {
            odo.updatePosition();
        }
    }
}