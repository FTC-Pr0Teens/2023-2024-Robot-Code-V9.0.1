package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

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
    //    private OutputCommand outputCommand;
    private IntakeCommand intakeCommand;
    private IMUSubsystem imuSubsystem;
    private GyroOdometry gyroOdometry;
    private GridAutoCentering gridAutoCentering;
    private OutputCommand outputCommand;
    private ColorSensorSubsystem colorSensorSubsystem;
    private ElapsedTime pixelTimer, liftTimer;
    private int level = -1;
    private int pixelCounter;
    private boolean running = true;
    private ElapsedTime timer;

    private String status = "Uninitialized";

    private enum RUNNING_STATE {
        LIFT_STOP,
        RETRACT_LIFT,
        RAISE_LIFT,
        DROP
    }


    private final TimerList timerList = new TimerList();

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        imuSubsystem = new IMUSubsystem(hardwareMap);

        //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);

        odometrySubsystem = new OdometrySubsystem(hardwareMap);

        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);

        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,
                gyroOdometry, this);

        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);

        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        pixelTimer = new ElapsedTime();
        liftTimer = new ElapsedTime();
        timer = new ElapsedTime();


        odometrySubsystem.reset();
        imuSubsystem.resetAngle();
        outputCommand.closeGate();
        outputCommand.armToIdle();
        outputCommand.tiltToIdle();
        pixelCounter = 0;

        level = 2;

        waitForStart();

        Executor executor = Executors.newFixedThreadPool(4);
        CompletableFuture.runAsync(this::liftProcess, executor);

        running = true;
//        status = "level = 1";
//        level = 1;
//        sleep(3000);
//
        status = "level = 2";
        level = 1;
        sleep(3000);

        level = 0;
        sleep(500);
        level = 0;
//
//        status = "level = 3";
//        level = 3;
//        sleep(3000);
//
//        status = "level = 4";
//        level = 4;
//        sleep(3000);


    }

    // Auto Processes
    public void liftProcess() {
        while (opModeIsActive()) {
//            if (running) {
//                multiMotorCommand.LiftUpPositional(level);
//                if ((level == 0 &&
//                        (multiMotorSubsystem.getDerivativeValue() == 0
//                                && multiMotorSubsystem.getPosition() < 5))
//                        || (multiMotorSubsystem.getDerivativeValue() < 0
//                        && multiMotorSubsystem.getPosition() < -5)) {
//                    multiMotorSubsystem.reset();
//                    running = false;
//                }
//            }
//        }
            telemetry.addLine("hi");
            multiMotorCommand.LiftUp(true, 2);
        }
        // COmmand/Helper Functions

    }


    public void updateTelemetry(){
        telemetry.addData("level", level);
        telemetry.update();
    }
}
