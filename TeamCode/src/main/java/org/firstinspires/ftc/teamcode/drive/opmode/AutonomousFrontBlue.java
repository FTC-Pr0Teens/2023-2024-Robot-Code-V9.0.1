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
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;


@Autonomous(name="Autonomous Front Blue")
public class AutonomousFrontBlue extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private IntakeCommand intakeCommand;
    private OutputCommand outputCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private ColorSensorSubsystem colorSensor;
    private WebcamSubsystem webcamSubsystem;

    private TimerList timers = new TimerList();

    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;
    //67, -3, 0
    //54, 24, 0
    //57, -22, -0.832
    //38, 80, -1.58
    private int level = -1;
    private String position = "initialized";
    double propPosition = 0;


    // NOTE:
    // CHANGE THIS DEPENDING ON ALLIANCE AUTO.
    // DEFAULT IS OFF, BUT IF ALLIANCE IS AFK, THEN SET THIS TO TRUE.
    private boolean doStacks = false;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        //Note: different for autonomous front red --> kpy
        mecanumCommand.setConstants(0.10, 0.03, 0.0095/2, 0.059, 0.004, 0.0095/2, 2.15, 0.0, 0.001);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        colorSensor = new ColorSensorSubsystem(hardwareMap);
        timer = new ElapsedTime();

        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_RED);

        //Pre-start
        odometrySubsystem.reset();
        imu.resetAngle();

        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();

        while (opModeInInit()) {
            propPosition = webcamSubsystem.getXProp();
        }

        waitForStart(); //WAIT FOR START

        timers.resetTimer("runTime"); //this will track the current run time of the robot. limit 30 seconds

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);

        if (opModeIsActive()) {
            if (propPosition <= 325 && propPosition > 0) {
                goToLeftSpike();
            } else if (propPosition > 325 && propPosition <= 700) {
                goToMiddleSpike();
            } else {
                goToRightSpike();
            }
            stop();
        }
    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta", gyroOdometry.theta);
            telemetry.addData("position", position);
            telemetry.addData("global x", mecanumCommand.globalXController.getOutputPositionalValue());
            telemetry.addData("global y", mecanumCommand.globalYController.getOutputPositionalValue());
            telemetry.addData("global theta", mecanumCommand.globalThetaController.getOutputPositionalValue());

            telemetry.update();
        }

        while (opModeInInit()){
            telemetry.addData("position", position);
        }
    }

    public void liftProcess() {
        while(opModeIsActive()) {
            multiMotorCommand.LiftUp(true, level);
        }
    }

    public void ThreadStop(){
        while (opModeIsActive()){
            isStopRequested();
        }
    }

    public void moveToPos(double x, double y, double theta, double toleranceX, double toleranceY, double toleranceTheta){
        mecanumCommand.moveIntegralReset();
        // stop moving if within 5 ticks or 0.2 radians from the position
        while ((Math.abs(x - gyroOdometry.x) > toleranceX  //if within 2.5 ticks of target X position
                || Math.abs(y - gyroOdometry.y) > toleranceY //if within 2.5 ticks of target y position
                || Math.abs(theta - gyroOdometry.theta) > toleranceTheta)
                && this.opModeIsActive() && !this.isStopRequested()) {
            mecanumCommand.moveToGlobalPos(x, y, theta);
        }
        mecanumSubsystem.stop(true);
    }

    //TODO: tune these values. This part gets the current position of the prop.


    private void goToRightSpike(){
        moveToPos(-69.48,-14,2.11,2,2,0.025);
        timer.reset();
        intakeCommand.autoPixel(1);

        while (timer.milliseconds() < 550) {
            intakeCommand.intakeOutNoRoller(0.3);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();


        timer.reset();
        while (opModeIsActive() ) {
            if (timer.milliseconds() > 3100){
                maintainPos(-45,80,Math.PI/2,2.5,2.5,0.05);
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
                break;
            } else if (timer.milliseconds() > 2400) {
                outputCommand.openGate();
                outputCommand.outputWheelIn();
            } else if (timer.milliseconds() > 500) {
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
            } else {
                level = 1;
            }
            if(timer.milliseconds() <= 3100) maintainPos(-42,93,Math.PI/2,2.5,2.5,0.05);
        }
        level = 0;
        outputCommand.outputWheelStop();
        outputCommand.closeGate();
        moveToPos(-42,93,Math.PI/2,2.5,2.5,0.05);

//        level = 5;
//        moveToPos(-5,65,Math.PI/2,2.5,2.5,0.05);
//        moveToPos(-5,100,Math.PI/2,2.5,2.5,0.05);
//        level = 5;



//        while (opModeIsActive()){
//            maintainPos(-6,-6,Math.PI/2,2.5,2.5,0.05);
//            outputCommand.armToIdle();
//            outputCommand.tiltToIdle();
//        }
//        level = 0;
//
//        moveToPos(-6,-150,Math.PI/2,2.5,2.5,0.05);
//        moveToPos(-63,190.912,Math.PI/2,2.5,2.5,0.05);
//
//        intakeCommand.autoPixel(2);
//        int need2white = 0;
//        while(opModeIsActive() && timer.milliseconds() < 2500) {
//            intakeCommand.intakeIn(0.85);
//            if (colorSensor.findColor2().equalsIgnoreCase("white")) {
//                if (need2white == 2){
//                    break;
//                } else {
//                    need2white++;
//                }
//            }
//        }
//
//        while(opModeIsActive() && timer.milliseconds() < 1500){
//            intakeCommand.autoPixel(1); //above max stack
//            intakeCommand.intakeOutNoRoller(1);
//            intakeCommand.intakeRollerIn();
//            maintainPos(-63,170,Math.PI/2,2.5,2.5,0.05);
//        }
//
//        moveToPos(-6,-150,Math.PI/2,2.5,2.5,0.05);
//        moveToPos(-6,-6,Math.PI/2,2.5,2.5,0.05);
//
//        timer.reset();
//        while (opModeIsActive() ) {
//            if (timer.milliseconds() > 5000) {
//                outputCommand.armToIdle();
//                outputCommand.tiltToIdle();
//                break;
//            } else if (timer.milliseconds() > 4000) {
//                outputCommand.openGate();
//                outputCommand.outputWheelIn();
//            } else if (timer.milliseconds() > 200) {
//                outputCommand.armToBoard();
//                outputCommand.tiltToBoard();
//            } else {
//                level = 1;
//            }
//            if (timer.milliseconds() <= 5000) maintainPos(-68, 94, Math.PI / 2, 2.5, 2.5, 0.05);
//
//            outputCommand.armToIdle();
//            outputCommand.tiltToIdle();
//            level = 0;
//
//
//        }
    }

    public void maintainPos(double x, double y, double theta, double toleranceX, double toleranceY, double toleranceTheta){
        mecanumCommand.moveIntegralReset();
        // stop moving if within 5 ticks or 0.2 radians from the position
        if ((Math.abs(x - gyroOdometry.x) > toleranceX  //if within 2.5 ticks of target X position
                || Math.abs(y - gyroOdometry.y) > toleranceY //if within 2.5 ticks of target y position
                || Math.abs(theta - gyroOdometry.theta) > toleranceTheta)
                && this.opModeIsActive() && !this.isStopRequested()) {
            mecanumCommand.moveToGlobalPos(x, y, theta);
        } else {
            mecanumSubsystem.stop(true);
        }
    }


    private void goToLeftSpike(){
        moveToPos(-95,43,1.10252,2.5,2.5,0.05); timer.reset();
        intakeCommand.autoPixel(1);

        while (timer.milliseconds() < 550) {
            intakeCommand.intakeOutNoRoller(0.3);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();


        timer.reset();
        while (opModeIsActive() ) {
            if (timer.milliseconds() > 3100){
                maintainPos(-83,80,Math.PI/2,2.5,2.5,0.05);
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
                break;
            } else if (timer.milliseconds() > 2400) {
                outputCommand.openGate();
                outputCommand.outputWheelIn();
            } else if (timer.milliseconds() > 500) {
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
            } else {
                level = 1;
            }
            if(timer.milliseconds() <= 3100) maintainPos(-80,94,Math.PI/2,2.5,2.5,0.05);
        }



//        moveToPos(-5,65,Math.PI/2,2.5,2.5,0.05);
//        level = 5;
//        outputCommand.armToIdle();
//        outputCommand.tiltToIdle();
        level = 0;
        outputCommand.outputWheelStop();
        outputCommand.closeGate();
        moveToPos(-80,94,Math.PI/2,2.5,2.5,0.05);
        //moveToPos(-5,100,Math.PI/2,2.5,2.5,0.05);


//        while (opModeIsActive()){
//            maintainPos(-6,-6,Math.PI/2,2.5,2.5,0.05);
//            outputCommand.armToIdle();
//            outputCommand.tiltToIdle();
//        }
//        level = 0;
//
//        moveToPos(-6,-150,Math.PI/2,2.5,2.5,0.05);
//        moveToPos(-63,190.912,Math.PI/2,2.5,2.5,0.05);
//
//        intakeCommand.autoPixel(2);
//        int need2white = 0;
//        while(opModeIsActive() && timer.milliseconds() < 2500) {
//            intakeCommand.intakeIn(0.85);
//            if (colorSensor.findColor2().equalsIgnoreCase("white")) {
//                if (need2white == 2){
//                    break;
//                } else {
//                    need2white++;
//                }
//            }
//        }
//
//        while(opModeIsActive() && timer.milliseconds() < 1500){
//            intakeCommand.autoPixel(1); //above max stack
//            intakeCommand.intakeOutNoRoller(1);
//            intakeCommand.intakeRollerIn();
//            maintainPos(-63,170,Math.PI/2,2.5,2.5,0.05);
//        }
//
//        moveToPos(-6,-150,Math.PI/2,2.5,2.5,0.05);
//        moveToPos(-6,-6,Math.PI/2,2.5,2.5,0.05);
//
//        timer.reset();
//        while (opModeIsActive() ) {
//            if (timer.milliseconds() > 5000) {
//                outputCommand.armToIdle();
//                outputCommand.tiltToIdle();
//                break;
//            } else if (timer.milliseconds() > 4000) {
//                outputCommand.openGate();
//                outputCommand.outputWheelIn();
//            } else if (timer.milliseconds() > 200) {
//                outputCommand.armToBoard();
//                outputCommand.tiltToBoard();
//            } else {
//                level = 1;
//            }
//            if (timer.milliseconds() <= 5000) maintainPos(-68, 94, Math.PI / 2, 2.5, 2.5, 0.05);
//
//            outputCommand.armToIdle();
//            outputCommand.tiltToIdle();
//            level = 0;
//
//
//        }

    }

    private void goToMiddleSpike(){
        moveToPos(-110,18,1.0139,2.5,2.5,0.025);

        timer.reset();
        intakeCommand.autoPixel(1);

        while (timer.milliseconds() < 550) {
            intakeCommand.intakeOutNoRoller(0.3);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();



        timer.reset();
        while (opModeIsActive() ) {
            if (timer.milliseconds() > 3100){
                maintainPos(-71,80,Math.PI/2,1.5,1.5,0.05);
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
                break;
            } else if (timer.milliseconds() > 2400) {
                outputCommand.openGate();
                outputCommand.outputWheelIn();
            } else if (timer.milliseconds() > 200) {
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
            } else {
                level = 1;
            }
            if(timer.milliseconds() <= 3100) maintainPos(-68,94,Math.PI/2,1.5,1.5,0.05);
        }
        level = 0;
        outputCommand.outputWheelStop();
        outputCommand.closeGate();
        moveToPos(-68,94,Math.PI/2,1.5,1.5,0.05);
//        level = 5;
//        moveToPos(-5,65,Math.PI/2,2.5,2.5,0.05);
//        moveToPos(-5,100,Math.PI/2,2.5,2.5,0.05);

//        while (opModeIsActive()){
//            maintainPos(-6,-6,Math.PI/2,2.5,2.5,0.05);
//            outputCommand.armToIdle();
//            outputCommand.tiltToIdle();
//        }
//        level = 0;
//
//        moveToPos(-6,-150,Math.PI/2,2.5,2.5,0.05);
//        moveToPos(-63,190.912,Math.PI/2,2.5,2.5,0.05);
//
//        intakeCommand.autoPixel(2);
//        int need2white = 0;
//        while(opModeIsActive() && timer.milliseconds() < 2500) {
//            intakeCommand.intakeIn(0.85);
//            if (colorSensor.findColor2().equalsIgnoreCase("white")) {
//                if (need2white == 2){
//                    break;
//                } else {
//                    need2white++;
//                }
//            }
//        }
//
//        while(opModeIsActive() && timer.milliseconds() < 1500){
//            intakeCommand.autoPixel(1); //above max stack
//            intakeCommand.intakeOutNoRoller(1);
//            intakeCommand.intakeRollerIn();
//            maintainPos(-63,170,Math.PI/2,2.5,2.5,0.05);
//        }
//
//        moveToPos(-6,-150,Math.PI/2,2.5,2.5,0.05);
//        moveToPos(-6,-6,Math.PI/2,2.5,2.5,0.05);
//
//        timer.reset();
//        while (opModeIsActive() ) {
//            if (timer.milliseconds() > 5000) {
//                outputCommand.armToIdle();
//                outputCommand.tiltToIdle();
//                break;
//            } else if (timer.milliseconds() > 4000) {
//                outputCommand.openGate();
//                outputCommand.outputWheelIn();
//            } else if (timer.milliseconds() > 200) {
//                outputCommand.armToBoard();
//                outputCommand.tiltToBoard();
//            } else {
//                level = 1;
//            }
//            if (timer.milliseconds() <= 5000) maintainPos(-68, 94, Math.PI / 2, 2.5, 2.5, 0.05);
//
//            outputCommand.armToIdle();
//            outputCommand.tiltToIdle();
//            level = 0;
//
//
//        }

    }




    private void goToBoardRight(){
        moveToPos(46, -78.5, 1.65, 5, 5, 0.2); //1.65 radians = 94.53804 degrees
    }

    private void goToBoardMiddle(){
        moveToPos(61, -80,1.65, 5,5,0.2);
    }

    private void goToBoardLeft(){
        moveToPos(68, -81.5,1.65, 5,5,0.2);
    }
}