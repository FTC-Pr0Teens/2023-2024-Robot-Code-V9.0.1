package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.NotMecanumSubsystem;

@TeleOp(name = "WhyNotBrokenTeleOP")
public class WhyNotBrokenTeleOp extends LinearOpMode {
    private NotMecanumSubsystem notMecanumSubsystem;

    @Override
    public void runOpMode() {
        notMecanumSubsystem = new NotMecanumSubsystem(hardwareMap);

        waitForStart();
        while(opModeIsActive()) {
            double drive = -gamepad1.left_stick_y; // Forward and backward
            double turn  =  gamepad1.left_stick_x; // Left and right
            double leftPower = drive + turn;
            double rightPower = drive - turn;
            double armPower = gamepad1.right_stick_y;
            double claw = gamepad1.right_trigger;

            notMecanumSubsystem.move(leftPower, rightPower, armPower);
            notMecanumSubsystem.setContinuousServoPower(claw);
        }
    }
}
