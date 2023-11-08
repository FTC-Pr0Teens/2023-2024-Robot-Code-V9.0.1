package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;

@TeleOp
public class MagneticSwitchTest extends LinearOpMode {
    TouchSensor bottomMagnet;
    TouchSensor topMagnet;

    @Override
    public void runOpMode() {
        bottomMagnet = hardwareMap.get(TouchSensor.class, "magnet");
        topMagnet = hardwareMap.get(TouchSensor.class, "magnet");
        MultiMotorSubsystem multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);

        waitForStart();

        while (opModeIsActive()) {
            double tickNum = multiMotorSubsystem.getPosition();
            if (bottomMagnet.isPressed() && gamepad1.left_stick_y <0) {
                multiMotorSubsystem.moveLift(-(-0.498 + 0.22 * Math.log(tickNum)));
            }
            if (topMagnet.isPressed() && gamepad1.left_stick_y >0) {
                multiMotorSubsystem.moveLift(-0.001*tickNum+4);
            }
            else {
                multiMotorSubsystem.moveLift(gamepad1.left_stick_y);
            }
        }
    }
}