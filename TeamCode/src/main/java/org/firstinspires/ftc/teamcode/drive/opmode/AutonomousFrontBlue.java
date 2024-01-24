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
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;

@Autonomous(name="Autonomous Front Blue")
public class AutonomousFrontBlue extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private IntakeCommand intakeCommand;
    private WebcamSubsystem webcamSubsystem;
    private OutputCommand outputCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;
    //67, -3, 0
    //54, 24, 0
    //57, -22, -0.832
    //38, 80, -1.58
    private int level = -1;


    @Override
    public void runOpMode() throws InterruptedException {
        //contour location before 60
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        timer = new ElapsedTime();

        odometrySubsystem.reset();
        imu.resetAngle();

        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        waitForStart();

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);

        String position = "left";

        //find prop position
        double propPosition = 0;
        timer.reset();
        while(timer.milliseconds() < 1000) {
            propPosition = webcamSubsystem.getXProp();
        }
//        sleep(8000);
        /*
        timer.reset();
        while(timer.milliseconds() < 3500) {
            if (propPosition < 100 && propPosition > 0) {
                //pos 2
                position = "left";
            } else if (propPosition > 100) {
                position = "right";
                sleep(1000);
            } else {
                position = "middle";
            }
        }
        */

        //go to spike
        if (position == "left"){
            mecanumCommand.moveToGlobalPosition(67, -3, 0);
        }
        else if (position == "middle"){
            mecanumCommand.moveToGlobalPosition(54, 24, 0);
        }
        else if (position == "right"){
            mecanumCommand.moveToGlobalPosition(57, 0, 0);
            sleep(1500);
            mecanumCommand.moveToGlobalPosition(55, -17, -0.832);
        }

        //output prop
        timer.reset();
        while(timer.milliseconds() < 1000) {
            intakeCommand.intakeOut(0.3);
        }
        intakeCommand.stopIntake();

        /*
        level = 1;
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();


        timer.reset();
        while(timer.milliseconds() < 3500) {
            if (target == "left") {
                //pos 2
                mecanumCommand.moveToGlobalPosition(53, 81, -1.58);
            } else if (target == "right") {
                mecanumCommand.moveToGlobalPosition(62.5, 81.5, -1.58);
            } else if (target == "middle"){
                mecanumCommand.moveToGlobalPosition(38, 78.5, -1.58);
            }
        }



        timer.reset();
        while (timer.milliseconds() < 500){
            outputCommand.openGate();
        }
        outputCommand.closeGate();
        outputCommand.tiltToIdle();
        outputCommand.armToIdle();
        sleep(6000);
        level = 0;
        mecanumCommand.moveToGlobalPosition(0, 84, -1.58);

         */

    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
//            packet.put("x", gyroOdometry.x);
//            packet.put("y", gyroOdometry.y);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
//            packet.put("x", gyroOdometry.x);
//            packet.put("y", gyroOdometry.y);
//            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
    }
    public void liftProcess() {
        while(opModeIsActive()) {
            multiMotorCommand.LiftUp(true, level);
        }
    }

}
