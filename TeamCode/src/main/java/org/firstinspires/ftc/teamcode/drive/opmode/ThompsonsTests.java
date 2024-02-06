package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.Coordinate;
import org.firstinspires.ftc.teamcode.util.CoordinateSet;
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.TimerList;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@Autonomous(name="Thompson's Tests")
public class ThompsonsTests extends LinearOpMode {
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
    private IntakeCommand intakeCommand;
    private IMUSubsystem imuSubsystem;
    private GyroOdometry gyroOdometry;
    private OutputCommand outputCommand;
    private WebcamSubsystem webcamSubsystem;
    private CoordinateSet coordinateSet;
    private int level = -1;
    private boolean running = true;
    private ElapsedTime timer;
    private String parkPlace = "left";

    private String status = "Uninitialized";
    private enum RUNNING_STATE {
        LIFT_STOP,
        RETRACT_LIFT,
        RAISE_LIFT,
        DROP
    }
    private RUNNING_STATE state = RUNNING_STATE.LIFT_STOP;


    @Override
    public void runOpMode() throws InterruptedException {
        instantiateSubsystems();
        readyRobot();

        waitForStart();
        startThreads();

        String position = webcamSubsystem.findSpikePosition();

        //Spike Drop-off
        moveToCheckpoint(CoordinateSet.KeyPoints.SPIKE_CHECKPOINT);
        switch (position) {
            case "left":
                moveTo(CoordinateSet.KeyPoints.SPIKE_LEFT);
                break;
            case "middle":
                moveTo(CoordinateSet.KeyPoints.SPIKE_MIDDLE);
                break;
            case "right":
                moveTo(CoordinateSet.KeyPoints.SPIKE_RIGHT);
                break;
        }
        releaseIntakePixel();


        // Pixel Stack
        moveToCheckpoint(CoordinateSet.KeyPoints.PIXEL_STACK_CHECKPOINT);
        moveTo(CoordinateSet.KeyPoints.PIXEL_STACK);
        intakePixel();


        //Middle Back
        moveToCheckpoint(CoordinateSet.KeyPoints.MIDDLE_BACK);

        //Middle Front
        moveToCheckpoint(CoordinateSet.KeyPoints.MIDDLE_FRONT);


        // Detecting April Tag Code
        //goToAprilTag = true;
        //sleep(1000);
        //
        //while(goToAprilTag && !isStopRequested()) {
        //    if(aprilCamSubsystem.getHashmap().containsKey(aprilID)){
        //        mecanumCommand.setFinalPosition(true, 30, getTargetX(-8.0), getTargetY(-5.0), getTargetTheta());
        //    }
        //    while(!mecanumCommand.isPositionReached(true, true)){}
        //}


        // April Tag Backups
        switch (position) {
            case "left":
                moveTo(CoordinateSet.KeyPoints.APRIL_TAG_LEFT);
                break;
            case "middle":
                moveTo(CoordinateSet.KeyPoints.APRIL_TAG_MIDDLE);
                break;
            case "right":
                moveTo(CoordinateSet.KeyPoints.APRIL_TAG_RIGHT);
                break;
        }
        dropPixel(1);


        //Middle Back
        moveToCheckpoint(CoordinateSet.KeyPoints.MIDDLE_BACK);

        //Middle Front
        moveToCheckpoint(CoordinateSet.KeyPoints.MIDDLE_FRONT);


        // Pixel Stack
        moveToCheckpoint(CoordinateSet.KeyPoints.PIXEL_STACK_CHECKPOINT);
        moveTo(CoordinateSet.KeyPoints.PIXEL_STACK);
        intakePixel();


        //Middle Back
        moveToCheckpoint(CoordinateSet.KeyPoints.MIDDLE_BACK);

        //Middle Front
        moveToCheckpoint(CoordinateSet.KeyPoints.MIDDLE_FRONT);

        // April Tag Backups
        switch (position) {
            case "left":
                moveTo(CoordinateSet.KeyPoints.APRIL_TAG_LEFT);
                break;
            case "middle":
                moveTo(CoordinateSet.KeyPoints.APRIL_TAG_MIDDLE);
                break;
            case "right":
                moveTo(CoordinateSet.KeyPoints.APRIL_TAG_RIGHT);
                break;
        }
        dropPixel(1);


//         Parking
        if (parkPlace.equalsIgnoreCase("left")) {
            // Checkpoint
            moveToCheckpoint(CoordinateSet.KeyPoints.PARKING_LEFT_CHECKPOINT);
            // Park
            moveTo(CoordinateSet.KeyPoints.PARKING_LEFT);
        } else {
            // Checkpoint
            moveToCheckpoint(CoordinateSet.KeyPoints.PARKING_RIGHT_CHECKPOINT);
            // Park
            moveTo(CoordinateSet.KeyPoints.PARKING_RIGHT);
        }

        for (int i = 30; i > 0; i--) {
            status = "Completed Op - Powering off in " + i + " seconds";
            sleep(1000);
        }
        stop();
    }

    // Auto Processes
    public void updateTelemetry(){
        while(opModeIsActive()) {
            telemetry.addData("spike location", webcamSubsystem.findSpikePosition());
            telemetry.addData("april tags", webcamSubsystem.getDetections());
            telemetry.update();
        }
    }


    public void liftProcess() {
        while(opModeIsActive()){
            if (running) {
                multiMotorCommand.LiftUpPositional(level);
                if ((level == 0 &&
                        (multiMotorSubsystem.getDerivativeValue() == 0
                                && multiMotorSubsystem.getPosition() < 5))
                        || (multiMotorSubsystem.getDerivativeValue() < 0
                        && multiMotorSubsystem.getPosition() < -5)) {
                    multiMotorSubsystem.reset();
                    running = false;
                }
            }
        }
    }
    public void updateOdometry() {
        while (opModeIsActive()) {
            gyroOdometry.odometryProcess();
        }
    }
    /**
     * Create the required subsystems.
     */
    private void instantiateSubsystems() {
        coordinateSet = new CoordinateSet(CoordinateSet.StartingPoint.BACK_RED);
        imuSubsystem = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, gyroOdometry, this);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        webcamSubsystem = new WebcamSubsystem(hardwareMap, WebcamSubsystem.PipelineName.CONTOUR_BLUE);
        timer = new ElapsedTime();
    }

    /**
     * Put robot in a ready position.
     */
    private void readyRobot() {
        imuSubsystem.resetAngle();
        odometrySubsystem.reset();
        intakeCommand.raiseIntake();
        outputCommand.closeGate();
        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        multiMotorSubsystem.reset();
    }

    private void startThreads() {
        Executor executor = Executors.newFixedThreadPool(6);
        CompletableFuture.runAsync(this::updateOdometry, executor);
        CompletableFuture.runAsync(this::updateTelemetry, executor);
        CompletableFuture.runAsync(this::liftProcess, executor);
        //CompletableFuture.runAsync(this::tagDetectionProcess);
    }
    private void moveTo(CoordinateSet.KeyPoints keyPoint) {
        Coordinate co = coordinateSet.getCoordinate(keyPoint);
        mecanumCommand.setFinalPosition(true, 30, co.getX(), co.getY(), co.getTheta());
        while (!mecanumCommand.isPositionReached(true, true) && !isStopRequested()) ;
    }
    private void moveTo(double x, double y, double theta) {
        mecanumCommand.setFinalPosition(true, 30, x, y, theta);
        while (!mecanumCommand.isPositionReached(true, true) && !isStopRequested()) ;
    }

    private void moveToCheckpoint(CoordinateSet.KeyPoints keyPoint) {
        Coordinate co = coordinateSet.getCoordinate(keyPoint);
        mecanumCommand.setFinalPosition(true, 30, co.getX(), co.getY(), co.getTheta());
        while (!mecanumCommand.isPositionPassed() && !isStopRequested()) ;
    }

    private void dropPixel(int level) {
        // Lift Slider to ready position
        this.level = 5;

        // Wait until slider is high enough before bringing out the arm
        while (!multiMotorSubsystem.isPositionReached());
        outputCommand.armToBoard();
        outputCommand.tiltToBoard();

        // Wait until the arm is far enough before moving the arm to its desired position
        while (outputCommand.getLeftArmPosition() < .7);
        this.level = level;

        // Wait until in position before dropping the pixel
        while (!multiMotorSubsystem.isPositionReached());
        outputCommand.openGate();

        // Wait until the pixel has (probably) fallen out before resetting
        sleep(300);
        outputCommand.closeGate();
        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        this.level = 5;

        // Wait until the arm has retracted enough before lowering the slider to 0
        while (outputCommand.getLeftArmPosition() > .8);
        this.level = 0;

    }
    private void releaseIntakePixel() {
        //release pixel
        intakeCommand.lowerIntake();
        intakeCommand.intakeOut(0.5);
        timer.reset();
        while(timer.milliseconds() < 1000);
        intakeCommand.raiseIntake();
        intakeCommand.stopIntake();
    }

    private void intakePixel() {
        intakeCommand.lowerIntake();
        intakeCommand.intakeIn(1);
        intakeCommand.intakeRollerIn();
        sleep(700);
        intakeCommand.raiseIntake();
        intakeCommand.stopIntake();
        intakeCommand.intakeRollerStop();
    }

}
