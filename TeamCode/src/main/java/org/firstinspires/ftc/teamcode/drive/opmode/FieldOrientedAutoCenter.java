package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Config
@TeleOp (name = "fieldoriented")
public class FieldOrientedAutoCenter extends LinearOpMode {


    private OdometrySubsystem odometrySubsystem;

    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private GyroOdometry gyroOdometry;
    private IMUSubsystem imuSubsystem;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    static public volatile double kp = 0.6;
    static public volatile double ki = 0;
    static public volatile double kd = 0;



    private GridAutoCentering gridAutoCentering;

    private boolean right_stick_pressed = false;
    private boolean left_stick_pressed = false;

    private boolean doCentering = false;
    private double autoCenterAngle = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        imuSubsystem = new IMUSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);
        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

        packet = new TelemetryPacket(true);


        Executor executor = Executors.newFixedThreadPool(5);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket(true);

        waitForStart();

        CompletableFuture.runAsync(this::processDriveMotor, executor);
        CompletableFuture.runAsync(this::processIMU, executor);
        CompletableFuture.runAsync(this::processDriveController);

        MultipleTelemetry multiTele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while(opModeIsActive()) {
            gridAutoCentering.setConstants(kp,ki,kd);
            packet = new TelemetryPacket(true);
            packet.put("x", odometrySubsystem.x);
            packet.put("y", odometrySubsystem.y);
            packet.put("theta", odometrySubsystem.theta);

            double posX = odometrySubsystem.x;
            double posY = odometrySubsystem.y;
            double posTheta = odometrySubsystem.theta;
            double robotWidth = 16.5;
            double robotLength = 18;


            //up down
            final double v2 = 0 - (robotWidth / 2) * Math.cos(posTheta) + posY;
            double v3 = 0 + (robotWidth / 2) * Math.cos(posTheta) + posY;
            double[] xcoords = {
                    v2,
                    v3,
                    v2,
                    v3,
            };

            //up down
            final double v = 0 + (robotLength / 2) * Math.cos(posTheta) + posX;
            final double v1 = 0 - (robotLength / 2) * Math.cos(posTheta) + posX;
            double[] ycoords = {
                    v1,
                    v,
                    v,
                    v1,
            };

            packet.fieldOverlay() //in inches
                    .setFill("blue")
                    .setAlpha(0.4)
                    .fillRect(odometrySubsystem.y, odometrySubsystem.x, 16.5, 18)
                    .fillPolygon(xcoords, ycoords)
                    .strokeLine(odometrySubsystem.y+16.5/2, odometrySubsystem.x,odometrySubsystem.y + 16.5/2,odometrySubsystem.x + 9)
                    .strokeLine(odometrySubsystem.y, odometrySubsystem.x, odometrySubsystem.x + 9*Math.sin(posTheta), odometrySubsystem.y + Math.cos(posTheta));
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }


    private void runMovement(){
        /**
         * These two will reset angle headings of the IMU, both field oriented and autocenter
         */
        //dont enable resetting if in "field is fucked up" state
        //This means that backdrop is to to the LEFT (meaning you are on BLUE side)
        if (gamepad1.dpad_left) {
            autoCenterAngle = Math.PI/2; //set autocenter to left 90 degrees
            gridAutoCentering.offsetTargetAngle(autoCenterAngle);

        }
        if(gamepad1.dpad_up){
            gridAutoCentering.reset(); //reset grid heading
        }

        if (gamepad1.dpad_right) {
            autoCenterAngle = Math.PI/2; //set autocenter to right 90 degrees
            gridAutoCentering.offsetTargetAngle(autoCenterAngle);

        }

        doCentering = gamepad1.left_trigger > 0.5;

        mecanumSubsystem.partialMove(true, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    private void processDriveMotor(){
        while(opModeIsActive()) {
            gridAutoCentering.process(doCentering);
            mecanumSubsystem.motorProcessTeleOp();
        }
    }

    private void processIMU() {
        while(opModeIsActive()) {
            imuSubsystem.gyroProcess();
            gyroOdometry.angleProcess();
            odometrySubsystem.process();
        }
    }

    private void processDriveController(){
        while(opModeIsActive()){
            runMovement();
        }
    }

}