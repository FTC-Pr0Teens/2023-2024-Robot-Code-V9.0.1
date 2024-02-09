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
            double drive = gamepad1.left_stick_y; // Forward and backward
            double turn = gamepad1.left_stick_x; // Left and right
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Adjust arm based on right stick vertical input
            double armAdjustment = gamepad1.right_stick_y;

            // Get claw position from right trigger
            double clawPosition = gamepad1.right_trigger;

            // Drive the robot
            notMecanumSubsystem.move(leftPower, rightPower);

            // Adjust arm position
            notMecanumSubsystem.controlArmMotor(armAdjustment);

            // Set claw positions. Assuming claw positions are mirrored.
            notMecanumSubsystem.setClawPositions(clawPosition, clawPosition);

            // Display arm position telemetry
            telemetry.update(); // Ensure telemetry data is sent to the display
        }
    }
}
