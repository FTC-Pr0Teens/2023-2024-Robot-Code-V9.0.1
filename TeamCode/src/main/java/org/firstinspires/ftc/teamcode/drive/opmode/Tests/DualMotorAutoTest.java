package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;

import java.util.concurrent.CompletableFuture;

@Autonomous(name = "liftauto test")
public class DualMotorAutoTest extends LinearOpMode {
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private int level = 0;
    private ElapsedTime elapsedTime;
    @Override
    public void runOpMode() throws InterruptedException{
        ElapsedTime elapsedTime = new ElapsedTime();
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        waitForStart();
        CompletableFuture.runAsync(this::liftProcess);
        CompletableFuture.runAsync(this::runTelemetry);

        elapsedTime.reset();
        while(elapsedTime.milliseconds() < 5000){
            level = 1;
        }
        elapsedTime.reset();
        while(elapsedTime.milliseconds() < 5000){
            level = 2;
        }
        elapsedTime.reset();
        level = 0;
    }

    public void liftProcess() {
        while(opModeIsActive()) {
            multiMotorCommand.LiftUp(true, level);
        }
    }

    public void runTelemetry(){
        while(opModeIsActive()){
            telemetry.addData("level", level);
            telemetry.update();
        }
    }


}
