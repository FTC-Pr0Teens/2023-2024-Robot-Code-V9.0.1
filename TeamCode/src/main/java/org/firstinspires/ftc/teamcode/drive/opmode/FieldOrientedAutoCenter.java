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

    static public volatile double kp = -1.5;
    static public volatile double ki = 0;
    static public volatile double kd = 0;
    static public volatile double ff = 0;

    static public volatile String startLocation = "backBlue";



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


        Executor executor = Executors.newFixedThreadPool(3);

        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket(true);

        waitForStart();

        CompletableFuture.runAsync(this::processDriveMotor, executor);
        CompletableFuture.runAsync(this::processIMU, executor);

        MultipleTelemetry multiTele = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        while(opModeIsActive()) {
            gridAutoCentering.setConstants(kp,ki,kd, ff);
            packet = new TelemetryPacket(true);
            packet.put("x", odometrySubsystem.x);
            packet.put("y", odometrySubsystem.y);
            packet.put("theta", odometrySubsystem.theta);
            double robotWidth = 16.5/2;
            double robotLength = 18/2;

            //backBlue
            double cmToInch = 0.3937;
            double posX = odometrySubsystem.y * cmToInch;
            double posY = odometrySubsystem.x* cmToInch;
            double posTheta = Math.PI + odometrySubsystem.theta;
            double offsetX = (-0.5) + (-1 * 24) - (robotWidth); //1 * 24 inch tile down + (0.5) inch mat side tab distance + 9 inch robot radius
            double offsetY = -(4*24) + (robotLength);             //3 * 24inch tiles right - 9 inches robot radius
            switch(startLocation) {
                case "backBlue":
                    posX = odometrySubsystem.y* cmToInch;
                    posY = odometrySubsystem.x* cmToInch;
                    posTheta = odometrySubsystem.theta;
                    offsetX = (-0.5) + (-2 * 24) + (robotWidth);
                    offsetY = (3*24) - (robotLength);
                    //start position = 90, 10 or something (fix later)
                    break;
                case "frontBlue":
                    posX = odometrySubsystem.y* cmToInch;
                    posY = odometrySubsystem.x* cmToInch;
                    posTheta = odometrySubsystem.theta;
                    offsetX = (-0.5) + (1 * 24) - (robotWidth*2);
                    offsetY = (3*24) - (robotLength);
                    break;
                case "frontRed":
                    posX = -odometrySubsystem.y* cmToInch;
                    posY = -odometrySubsystem.x* cmToInch;
                    posTheta = Math.PI + odometrySubsystem.theta;
                    offsetX = (-0.5) + (1 * 24) - (robotWidth*2); //1 * 24 inch tile down + (0.5) inch mat side tab distance + 9 inch robot radius
                    offsetY = -(3*24) - (robotLength);
                    break;
                case "backRed":
                    posX = -odometrySubsystem.y* cmToInch;
                    posY = -odometrySubsystem.x* cmToInch;
                    posTheta = Math.PI + odometrySubsystem.theta;
                    offsetX = (-0.5) + (-2 * 24) + (robotWidth); //1 * 24 inch tile down + (0.5) inch mat side tab distance + 9 inch robot radius
                    offsetY = -(3*24) + (robotLength);
                    break;
                default:
                    posX = -odometrySubsystem.y* cmToInch;
                    posY = -odometrySubsystem.x* cmToInch;
                    posTheta = Math.PI + odometrySubsystem.theta;
                    break;
            }


            packet.put("target", autoCenterAngle);
            packet.put("rotVel", gridAutoCentering.getGraphRotationalVel());
            packet.put("right encoder", odometrySubsystem.rightEncoder());
            packet.put("left encoder", odometrySubsystem.leftEncoder());
            packet.put("back encoder", odometrySubsystem.backEncoder());

            packet.fieldOverlay() //in inches
                    .setTranslation(offsetX, offsetY)
                    //3 * 24inch tiles left - 9 inches robot radius
                    //1 * 24 inch tile down + (0.5) inch mat side tab distance + 9 inch robot radius
                    .setFill("blue")
                    .setAlpha(0.4)
                    .fillCircle(posX,posY,9)
                    .setFill("black")
                    .setAlpha(1)
                    .strokeLine(posX,posY, posX + (9*Math.sin(posTheta)), posY - (9 * Math.cos(posTheta)));

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
            imuSubsystem.resetAngle();
        }
        if(gamepad1.dpad_down){
            autoCenterAngle = 0;
            gridAutoCentering.offsetTargetAngle(autoCenterAngle);
        }

        if (gamepad1.dpad_right) {
            autoCenterAngle = -Math.PI/2; //set autocenter to right 90 degrees
            gridAutoCentering.offsetTargetAngle(autoCenterAngle);

        }

        doCentering = gamepad1.left_trigger > 0.5;

        mecanumCommand.moveGlobalPartial(true, -gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
    }

    private void processDriveMotor(){
        while(opModeIsActive()) {
            runMovement();
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



}