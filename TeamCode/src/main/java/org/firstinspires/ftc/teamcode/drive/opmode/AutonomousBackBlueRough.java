package org.firstinspires.ftc.teamcode.drive.opmode;

// Android and FTC SDK imports for robot operation and telemetry

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.AprilCamSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.HashSet;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="Autonomous Back Blue Rough")
public class AutonomousBackBlueRough extends LinearOpMode {

    //Custom imports
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IMUSubsystem imu;
    private OdometrySubsystem odometrySubsystem;
    private GyroOdometry gyroOdometry;
    private IntakeCommand intakeCommand;
    //private WebcamSubsystem webcamSubsystem;
    private OutputCommand outputCommand;
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;

    private ColorSensorSubsystem colorSensor;

    private Servo hangingServoL;
    private Servo hangingServoR;
    FtcDashboard dashboard;
    TelemetryPacket packet;
    private ElapsedTime timer;

    ElapsedTime outputTimer = new ElapsedTime();
    //67, -3, 0
    //54, 24, 0
    //57, -22, -0.832
    //38, 80, -1.58
    private int level = -1;
    private String position = "initalized";
    private String progress = "initalization";
    private int finalX = -125;
    private int finalY = 6;
    private int finalTheta = 0;

    private double targetX = 0;
    private double targetY = 0;

    private Integer aprilID = 2;

    private boolean goToAprilTag = false;
    private String autoColor = "blue";


    private HashSet <LIFT_STATE> liftState = new HashSet<>();


    private enum LIFT_STATE {
        LIFT_IDLE,
        LIFT_MIDDLE,
        LIFT_END,
        HANGSERVO, DROP_PIXEL
    }


    @Override
    public void runOpMode() throws InterruptedException {
        //initializing the subsystems
        imu = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imu);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        hangingServoL = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_L);
        hangingServoL.setDirection(Servo.Direction.REVERSE);
        hangingServoR = hardwareMap.get(Servo.class, Specifications.HANGING_SERVO_R);
        //Note different for autonomous front red --> kpy
        //TODO: constants are set higher than usual, approximately started at 0.07 for kpx, kpy and integral terms were 0.007
        //kp overshoots, causing a lot of back and forth movement, thus making y reach less of its actual position
        //most recent pid constants are for x axis
        //if undershooting, increase integral or all of thje constantst

        //TODO: increase kp to adjust faster, but this also increases oscillations so increase kd a little
        //derivative value is at 0.01

        // mecanumCommand.setConstants(0.07, 0.01, 0.0075/2, 0.059, 0.0005, 0.0075/2, 2.1, 0.0, 0.0);
        mecanumCommand.setConstants(0.1, 0.03, 0.0095/2, 0.059, 0.004, 0.0095/2, 2.15, 0.0, 0.001);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        //webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        timer = new ElapsedTime();
        colorSensor = new ColorSensorSubsystem(hardwareMap);

        //initializing some variables

        //resets the different subsystems to for preparation
        odometrySubsystem.reset();
        imu.resetAngle();

        hangingServoL.setPosition(0.6);
        hangingServoR.setPosition(0.6);

        intakeCommand.raiseIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();




        waitForStart();

        Executor executor = Executors.newFixedThreadPool(6);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);
        CompletableFuture.runAsync(this::processLift,executor);
        CompletableFuture.runAsync(this::checkLiftState,executor);
       // CompletableFuture.runAsync(this::ThreadStop);
        //setPropPosition();






        //TODO: positive x goes towards red side when turning 90 counter clockwise
        //TODO: positive y goes away from the board

        //TODO: when turning clockwise it is the opposite of the text above me
        //TODO: below is left
        //TODO: below is left
//        telemetry.addData("test", gyroOdometry.x);
//        double propPosition = 0;
//
//        timer.reset();
//        while (opModeInInit()) {
//            propPosition = webcamSubsystem.getXProp();
//        }
//
//        if (propPosition > 475) {
//            goToMiddleSpike();
//        } else if (propPosition > 0 && propPosition < 475) {
//            goToLeftSpike();
//        } else {
//            goToRightSpike();
//        }
        goToLeftSpike();
    }

    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }

    public void updateTelemetry() {
        while (opModeIsActive()) {
            packet.put("x", gyroOdometry.x);
            packet.put("y", gyroOdometry.y);
            packet.put("theta", gyroOdometry.theta);
            packet.put("x pos", mecanumCommand.globalXController.getError());
            packet.put("x integral", mecanumCommand.globalXController.getIntegralSum());
            packet.put("x derivative", mecanumCommand.globalXController.getDerivative());
            packet.put("y pos", mecanumCommand.globalYController.getError());
            packet.put("y integral", mecanumCommand.globalYController.getIntegralSum());
            packet.put("y derivative", mecanumCommand.globalYController.getDerivative());
            packet.put("Theta pos", mecanumCommand.globalThetaController.getError());
            packet.put("Theta integral", mecanumCommand.globalThetaController.getIntegralSum());
            packet.put("Theta derivative", mecanumCommand.globalThetaController.getDerivative());
            packet.put("final x", finalX);
            packet.put("final y", finalY);
            packet.put("final theta", finalTheta);
            telemetry.addData("x", gyroOdometry.x);
            telemetry.addData("y", gyroOdometry.y);
            telemetry.addData("theta",  gyroOdometry.theta);
//            telemetry.addData("position", position);
//            telemetry.addData("global x", mecanumCommand.globalXController.getOutputPositionalValue());
//            telemetry.addData("global y", mecanumCommand.globalYController.getOutputPositionalValue());
//            telemetry.addData("global theta", mecanumCommand.globalThetaController.getOutputPositionalValue());
            //telemetry.addData("xprop", webcamSubsystem.getXProp());
            telemetry.addData("back encoder count", odometrySubsystem.backEncoder());
            telemetry.addData("left encoder count", odometrySubsystem.leftEncoder());
            telemetry.addData("right encoder count", odometrySubsystem.rightEncoder());
            telemetry.addData("progress", progress);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
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
        //pos is good
        moveToPos(-69.48,16,-2.11,2,2,0.025);
//        sleep(1000);
//        moveToPos(-80,-24,-Math.PI/2,5,5,0.05);
        timer.reset();
        progress = "intake start";
        intakeCommand.autoPixel(2);

        while (timer.milliseconds() < 660) {
            intakeCommand.intakeOutNoRoller(0.4);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();
        progress = "intake stop";
        progress = "checkpoint 1 start";
        //129, 43.5, -Math.PI/2
        //129, 60.8, -Math.PI/2
        moveToPos(-120,-43,Math.PI/2,3,3,0.015);
        intakeCommand.autoPixel(3);
        timer.reset();
        while(opModeIsActive() && timer.milliseconds() < 3000) {
            intakeCommand.intakeIn(0.7);
            if (colorSensor.findColor2().equalsIgnoreCase("white")) break;
        }

        timer.reset();
//        ejectPixelLoop

        while(opModeIsActive() && timer.milliseconds() < 1500){
            intakeCommand.autoPixel(1); //above max stack
            intakeCommand.intakeOutNoRoller(1);
            intakeCommand.intakeRollerIn();
            maintainPos(-116,-36, Math.PI/2,2.5,2.5,0.015);
        }

        intakeCommand.stopIntake();

        progress = "checkpoint 2 end";
//        moveToPos(-150, -177,Math.PI/2,7,5,0.05);
//        progress = "checkpoint 3 end";
//        sleep(1000);
        moveToPos(-125,160,Math.PI/2,5,5,0.05);
        moveToPos(-48,170,Math.PI/2,2.5,2.5,0.05);
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 3100) {
            if (timer.milliseconds() > 3100){
                moveToPos(-30,200,Math.PI/2,2.5,2.5,0.015);
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
                break;
            } else if (timer.milliseconds() > 2400) {
                outputCommand.openGate();
                outputCommand.outputWheelIn();
            } else if (timer.milliseconds() > 150) {
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
            } else {
                level = 1;
            }
            if(timer.milliseconds() <= 3100) maintainPos(-50,217,Math.PI/2,2.5,2.5,0.05);
        }
        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        sleep(700);
        timer.reset();
        while (opModeIsActive()) {
            level = 0;
            maintainPos(-125,190,Math.PI/2,5,5,0.05);
            if(timer.milliseconds() > 2000)break;
        }

        level = 0;
        outputCommand.closeGate();
        progress = "lift down pls";



    }


    private void goToMiddleSpike(){
        //pos is good
        moveToPos(-121,8.3,0.587,2.5,2.5,0.025);
        timer.reset();
        progress = "intake start";
        intakeCommand.autoPixel(2);
        while (timer.milliseconds() < 500) {
            intakeCommand.intakeOutNoRoller(0.3);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();
//        sleep(1000);
//        moveToPos(-80,-24,-Math.PI/2,5,5,0.05);

        //129, 43.5, -Math.PI/2
        //129, 60.8, -Math.PI/2

        moveToPos(-128,-28.5,Math.PI/2,2.5,2.5,0.05);
        moveToPos(-128,-42, Math.PI/2,2.5,2.5,0.05);
        intakeCommand.autoPixel(2);
        timer.reset();

        while(opModeIsActive() && timer.milliseconds() < 1800) {
            intakeCommand.intakeIn(0.7);
            if (colorSensor.findColor2().equalsIgnoreCase("white")) break;
        }

        timer.reset();
//        ejectPixelLoop

        while(opModeIsActive() && timer.milliseconds() < 1500){
            intakeCommand.autoPixel(1); //above max stack
            intakeCommand.intakeOutNoRoller(1);
            intakeCommand.intakeRollerIn();
            maintainPos(-128,-42,Math.PI/2,2.5,2.5,0.05);
        }

        intakeCommand.stopIntake();

        progress = "checkpoint 2 end";
//        moveToPos(-150, -177,Math.PI/2,7,5,0.05);
//        progress = "checkpoint 3 end";
//        sleep(1000);
        moveToPos(-140,160,Math.PI/2,2.5,2.5,0.25);

        //middle: -82,-235
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 2500) {
            maintainPos(-78, 210, Math.PI / 2, 2.5, 2.5, 0.05);
            level = 1;
            outputCommand.armToBoard();
            outputCommand.tiltToBoard();
        }

        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 3500) {
            double milliseconds = timer.milliseconds();
            //go to board
            if (milliseconds > 3100){
                maintainPos(-90,200,Math.PI/2,2.5,2.5,0.015); //moves back from board
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
                break;
            } else if (milliseconds > 1500) {
                outputCommand.openGate();
                outputCommand.outputWheelIn();
            } else if (milliseconds > 200) {
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
                //ik level = 1 and arm and tilt to board is redundant but bear with me
            } else {
                level = 1;
            }
            if(milliseconds <= 3500) maintainPos(-90,223,Math.PI/2,2.5,2.5,0.05); //go to board
        }
        timer.reset();
        while (opModeIsActive()) {
            outputCommand.armToIdle();
            outputCommand.tiltToIdle();
            maintainPos(-140,160,Math.PI/2,2.5,2.5,0.05);
            if(timer.milliseconds() > 2000)break;
        }
        level = 0;
        outputCommand.closeGate();

        moveToPos(-128,160,Math.PI/2,2.5,2.5,0.05);
        moveToPos(-128,-48.5, Math.PI/2,2.5,2.5,0.05);

        intakeCommand.autoPixel(5);
        timer.reset();

        int need2white = 0;
        while(opModeIsActive() && timer.milliseconds() < 2500) {
            intakeCommand.intakeIn(0.85);
            maintainPos(-130,-48.5,Math.PI/2,2.5,2.5,0.05);

            if (colorSensor.findColor2().equalsIgnoreCase("white")) {
                if (need2white == 2){
                    break;
                } else {
                    need2white++;
                }
            }

            maintainPos(-144,48.5,2.356,2.5,2.5,0.05);
        }

        timer.reset();
//        ejectPixelLoop

        while(opModeIsActive() && timer.milliseconds() < 700){
            intakeCommand.autoPixel(1); //above max stack
            intakeCommand.intakeOutNoRoller(1);
            intakeCommand.intakeRollerIn();
            maintainPos(-140,-50,Math.PI/2,2.5,2.5,0.05);
        }

        intakeCommand.stopIntake();
        moveToPos(-165,160,Math.PI/2,2.5,2.5,0.05);

        //middle: -82,-23
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 2500) {
            moveToPos(-100,180,Math.PI/2,2.5,2.5,0.05);
            level = 1;
            outputCommand.armToBoard();
            outputCommand.tiltToBoard();
        }
        timer.reset();
        while (opModeIsActive() && timer.milliseconds() < 3400) {
            double milliseconds = timer.milliseconds();
            //go to board
            if (milliseconds > 3100){
                maintainPos(-101,200,Math.PI/2,2.5,2.5,0.05); //moves back from board
                outputCommand.armToIdle();
                outputCommand.tiltToIdle();
                break;
            } else if (milliseconds > 1500) {
                outputCommand.openGate();
                outputCommand.outputWheelIn();
            } else if (milliseconds > 100) {
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
                //ik level = 1 and arm and tilt to board is redundant but bear with me
            } else {
                level = 1;
            }
            if(milliseconds <= 3400) maintainPos(-110,220,Math.PI/2,2.5,2.5,0.05); //go to board
        }
        timer.reset();
        while (opModeIsActive()) {
            outputCommand.armToIdle();
            outputCommand.tiltToIdle();
            maintainPos(-140,160,Math.PI/2,2.5,2.5,0.05);
            if(timer.milliseconds() > 2000)break;
        }
        level = 0;
        outputCommand.closeGate();










    }

    private void goToRightSpike() {
        moveToPos(-112, -3.1, 0.818, 2.5, 2.5, 0.05);
        timer.reset();
        progress = "intake start";
        intakeCommand.autoPixel(1);

        while (timer.milliseconds() < 550) {
            intakeCommand.intakeOutNoRoller(0.4);
        }
        intakeCommand.stopIntake();
        intakeCommand.raiseIntake();
        moveToPos(-125, 5, Math.PI / 2, 2.5, 2.5, 0.05);
        moveToPos(-125, -33, Math.PI / 2, 2.5, 2.5, 0.05);
        sleep(100);
        timer.reset();

        while (opModeIsActive() && timer.milliseconds() < 2100) {
            intakeCommand.autoPixel(2);
            intakeCommand.intakeIn(0.7);
            maintainPos(-125, -38, Math.PI / 2, 2.5, 2.5, 0.05);
            if (colorSensor.findColor2().equalsIgnoreCase("white")) break;
        }

        timer.reset();
//        ejectPixelLoop

        while (opModeIsActive() && timer.milliseconds() < 1500) {
            intakeCommand.autoPixel(1); //above max stack
            intakeCommand.intakeOutNoRoller(1);
            intakeCommand.intakeRollerIn();
            maintainPos(-135, -33, Math.PI / 2, 1.5, 1.5, 0.05);
        }

        intakeCommand.stopIntake();

        progress = "checkpoint 2 end";
//        moveToPos(-150, -177,Math.PI/2,7,5,0.05);
//        progress = "checkpoint 3 end";
//        sleep(1000);
        moveToPos(-135, 155, Math.PI / 2, 7, 10, 0.05);

        //middle: -82,-235

        moveToPos(-87, 190, Math.PI / 2, 2.5, 2.5, 0.05);
        timer.reset();
        while (opModeIsActive()) {
            //go to board
            if (timer.milliseconds() > 3100) {
                maintainPos(-87, 190, Math.PI / 2, 2.5, 2.5, 0.05); //moves back from board
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
            if (timer.milliseconds() < 3100)
                maintainPos(-87, 222, Math.PI / 2, 2.5, 2.5, 0.05); //go to board
        }
//        while (opModeIsActive()) {
//            maintainPos(-87,200,Math.PI/2,7,7,0.05);
//            if(timer.milliseconds() > 1500) break;
//        }

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        timer.reset();
        while (timer.milliseconds() < 700);
        progress = "hi";
        level = 0;
        outputCommand.closeGate();
    }





    public void processLift(){
        multiMotorCommand.LiftUp(true, level);
    }


    private void checkLiftState() {
        if (liftState.contains(LIFT_STATE.LIFT_IDLE)) {
            outputCommand.tiltToIdle(); //bring arm down BEFORE bringing lift down
            outputCommand.armToIdle();
            if (outputTimer.milliseconds() > 900) {
                level = 0;
                if (outputTimer.milliseconds() > 1300) liftState.clear(); //moved here
            }
        } else if (liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
            if (!liftState.contains(LIFT_STATE.LIFT_END)) {
                if (outputTimer.milliseconds() > 1000) {
                    liftState.clear();
                } else if (outputTimer.milliseconds() > 250) { //bring lift up BEFORE extending arm
                    outputCommand.armToBoard();
                    outputCommand.tiltToBoard();
                } else {
                    level = 1;
                }
            }
        } else if (liftState.contains(LIFT_STATE.LIFT_END)) {
            if (!liftState.contains(LIFT_STATE.LIFT_MIDDLE)) {
                if (outputTimer.milliseconds() > 1000) {
                    liftState.clear();
                } else if (outputTimer.milliseconds() > 250) { //bring lift up BEFORE extending arm
                    outputCommand.armToBoard();
                    outputCommand.tiltToBoard();
                } else {
                    level = 2;
                }
            }
        }
    }

}