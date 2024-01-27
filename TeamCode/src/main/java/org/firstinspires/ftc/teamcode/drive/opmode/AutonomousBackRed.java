package org.firstinspires.ftc.teamcode.drive.opmode;

// Android and FTC SDK imports for robot operation and telemetry
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

@Autonomous(name="Autonomous Back Red")
public class AutonomousBackRed extends LinearOpMode {

    //Custom imports
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
    private int level = -1;
    private String position = "initalized";

    @Override
    public void runOpMode() throws InterruptedException {
        //initializing the subsystems
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        //Note different for autonomous front red --> kpy
        mecanumCommand.setConstants(0.07, 0.01, 0.0075 / 2, 0.05, 0.005, 0.0075 / 2, 2, 0.05, 0.0);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_RED);
        timer = new ElapsedTime();

        double intakePower = 0;

        //resets the different subsystems to for preparation
        odometrySubsystem.reset();
        imu.resetAngle();

        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        waitForStart();

        Executor executor = Executors.newFixedThreadPool(4);


        mecanumCommand.setFinalPosition(true, 3, -10, 0, 0);
//        webcamSubsystem.getXProp();
//        double propPosition = 0; //propPosition - using the prop the identify the place of he robot
//        timer.reset();
//
//        while(opModeInInit()) {
//            propPosition = webcamSubsystem.getXProp();
//        }
//        intakeCommand.raiseIntake();
////        sleep(8000);
////        sleep(8000);
////        sleep(8000);
//        timer.reset();
//            //TODO: tune
//        if (propPosition > 175) {
//            position = "middle";
//            mecanumCommand.moveToGlobalPosition(-116, 0, 0);
//            sleep(1000);
//            intakePower = 0.4;
//        } else if (propPosition <= 175 && propPosition > 0) {
//            position = "left";
//            mecanumCommand.moveToGlobalPosition(-95, 0, 0);
//            sleep(1100);
//            mecanumCommand.moveToGlobalPosition(-95, 0, 0.75);
//            intakePower = 0.5;
//        } else {
//            mecanumCommand.moveToGlobalPosition(-80, 0, 0);
//            sleep(1880);
//            mecanumCommand.moveToGlobalPosition(-80, 0, -0.96);
//            intakePower = 0.62;
//            //move to board
//            //mecanumCommand.moveToGlobalPosition(-400.5, 17.5, 0.2);
//            // changing theta to either more negative or more positive causes the robot to strafe / act weird
//
//        }
        timer.reset();
        while (timer.milliseconds() < 2500) {
            intakeCommand.intakeOut(intakePower);


            setPropPosition();

            position = "left";

            //go to correct spike
            if (position.equals("left")) {

            } else if (position.equals("middle")) {

            } else if (position.equals("right")) {

            }

            //output prop
            timer.reset();
            intakeCommand.raiseIntake();
            while (timer.milliseconds() < 1000) {
                intakeCommand.intakeOut(0.5);

            }
            intakeCommand.stopIntake();

            if (position.equals("left")) {

            } else if (position.equals("middle")) {

            } else if (position.equals("right")) {

            }

            sleep(1000);
            stop();

        }


        while (opModeInInit()) {
            telemetry.addData("prop", webcamSubsystem.getXProp());
            telemetry.addData("position", position);
        }
    }


    private void setPropPosition() {
        double propPosition = 0;
        timer.reset();
        while (opModeInInit()) {
            propPosition = webcamSubsystem.getXProp();
        }
        timer.reset();

        if (propPosition < 100 && propPosition > 0) {
            position = "left";
        } else if (propPosition > 100) {
            position = "right";
            sleep(1000);
        } else {
            position = "middle";
        }
    }
}

//    private void goToRightSpike(){
//        moveToPos(57, 0, 0, 5, 5, 0.2);
//        sleep(1500);
//        moveToPos(55, -17, -0.832, 5, 5, 0.2);
//    }
//
//    private void goToMiddleSpike(){
//        moveToPos(54, 24, 0, 5,5, 0.2);
//    }
//
//    private void goToLeftSpike(){
//        moveToPos(67,-3,0, 5,5, 0.2);
//    }
//
//    private void goToBoardRight(){
//        moveToPos(46, -78.5, 1.65, 5, 5, 0.2); //1.65 radians = 94.53804 degrees
//    }
//
//    private void goToBoardMiddle(){
//        moveToPos(61, -80,1.65, 5,5,0.2);
//    }
//
//    private void goToBoardLeft(){
//        moveToPos(68, -81.5,1.65, 5,5,0.2);
//    }


