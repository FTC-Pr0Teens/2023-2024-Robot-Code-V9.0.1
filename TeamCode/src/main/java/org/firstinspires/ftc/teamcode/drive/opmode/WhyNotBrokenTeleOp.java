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
            double leftPower = -gamepad1.left_stick_y; // Inverting to match joystick direction
            double rightPower = -gamepad1.right_stick_y; // Inverting to match joystick direction
            notMecanumSubsystem.move(leftPower, rightPower);
        }
    }
}
