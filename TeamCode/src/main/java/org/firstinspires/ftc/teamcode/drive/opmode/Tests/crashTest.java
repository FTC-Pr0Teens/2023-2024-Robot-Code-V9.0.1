package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.TimerList;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;

@TeleOp(name="crash test")
public class crashTest extends LinearOpMode {
    //https://docs.wpilib.org/en/stable/docs/software/vision-processing/grip/introduction-to-grip.html

    OutputCommand output = new OutputCommand(hardwareMap);
    private TimerList timers = new TimerList();

    @Override
    public void runOpMode(){

        waitForStart();
        while(opModeIsActive()){
            telemetry.addLine("running");
            telemetry.update();
            handleStop();
        }
    }

    public void handleStop(){
        if(isStopRequested()){
            telemetry.addLine("STOP REQUESTED");
            telemetry.update();
            output.outputWheelIn();
            timers.resetTimer("WAIT");
            while(!timers.checkTimePassed("WAIT", 500)){}
            output.outputWheelStop();
        }
    }
}
