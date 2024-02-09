package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.NotMecanumSubsystem;

@TeleOp(name = "ControlHubTesting")
public class ControlHubTestingClass extends LinearOpMode {
    private NotMecanumSubsystem notMecanumSubsystem;
    @Override
    public void runOpMode() {
        notMecanumSubsystem = new NotMecanumSubsystem(hardwareMap);
        waitForStart();
        while(opModeIsActive()) {
            notMecanumSubsystem.controlArmMotor(0.5);
            telemetry.addLine("This Control hub is working");
            telemetry.update();
        }
    }
}
