package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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

    static private volatile double kp = 0.6;
    static private volatile double ki = 0;
    static private volatile double kd = 0;

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

        Executor executor = Executors.newFixedThreadPool(5);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket(true);

        waitForStart();

        CompletableFuture.runAsync(this::processDriveMotor, executor);
        CompletableFuture.runAsync(this::processIMU, executor);
        CompletableFuture.runAsync(this::processDriveController);

        while(opModeIsActive()) {
            gridAutoCentering.setConstants(kp,ki,kd);

            packet.put("x", odometrySubsystem.x);
            packet.put("y", odometrySubsystem.y);
            packet.put("theta", odometrySubsystem.theta);
            packet.fieldOverlay() //in inches
                    .setFill("blue")
                    .setAlpha(0.4)
                    .fillRect(odometrySubsystem.x, odometrySubsystem.y, 16.5, 18)
                    .strokeLine(odometrySubsystem.x,odometrySubsystem.y,odometrySubsystem.x,odometrySubsystem.y + 9);
            dashboard.sendTelemetryPacket(packet);
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