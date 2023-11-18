package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.SingleMotorSubsystem;

public class IntakeTest extends LinearOpMode {
    IntakeCommand intakeCommand;
    double position = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        intakeCommand = new IntakeCommand(hardwareMap);

        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a){
                intakeCommand.intakeIn();
            }
            else if(gamepad1.b){
                intakeCommand.intakeOut();
            }
            else if(gamepad1.y){
                intakeCommand.raiseIntake();
            }
            else if(gamepad1.x){
                intakeCommand.lowerIntake();
            }
            else {
                intakeCommand.stopIntake();
            }
        }
    }
}
