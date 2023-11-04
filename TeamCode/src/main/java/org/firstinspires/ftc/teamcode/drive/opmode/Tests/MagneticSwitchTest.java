package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;

@TeleOp
public class MagneticSwitchTest extends LinearOpMode {
    // Define variables for our touch sensor and motor
    TouchSensor magnet;

    @Override
    public void runOpMode() {
        // Get the touch sensor and motor from hardwareMap
        magnet = hardwareMap.get(TouchSensor.class, "magnet");
        MultiMotorSubsystem multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);

        // Wait for the play button to be pressed
        waitForStart();

        // Loop while the Op Mode is running
        while (opModeIsActive()) {
            double tickNum = multiMotorSubsystem.getPosition();
            if (magnet.isPressed()) {
                telemetry.addData("pressed","");

                multiMotorSubsystem.moveLift(tickNum);
            }
            else {
                telemetry.addData("not pressed","");
                multiMotorSubsystem.moveLift(gamepad1.left_stick_y);
            }

            telemetry.update();
        }
    }
}