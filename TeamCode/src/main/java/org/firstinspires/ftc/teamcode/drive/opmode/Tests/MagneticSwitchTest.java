package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;

@TeleOp
public class MagneticSwitchTest extends LinearOpMode {
    TouchSensor magnet;

    @Override
    public void runOpMode() {
        magnet = hardwareMap.get(TouchSensor.class, "magnet");
        MultiMotorSubsystem multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);

        waitForStart();

        while (opModeIsActive()) {
            double tickNum = multiMotorSubsystem.getPosition();
            if (magnet.isPressed() && gamepad1.left_stick_y <0) {
                multiMotorSubsystem.moveLift(-(-0.498 + 0.22 * Math.log(tickNum)));
            }
            else {
                multiMotorSubsystem.moveLift(gamepad1.left_stick_y);
            }

            telemetry.update();
        }
    }
}