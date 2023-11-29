package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;

@TeleOp(name="drive test")
public class MecanumTest extends LinearOpMode {
    private MecanumSubsystem drive;
    private IMUSubsystem imu;
    @Override
    public void runOpMode() throws InterruptedException {

        drive = new MecanumSubsystem(hardwareMap);
        imu = new IMUSubsystem(hardwareMap);

        imu.resetAngle();

        waitForStart();

        while (opModeIsActive()) {
            //drive.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            drive.move(true, gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0);

//            telemetry.addData("Heading in DEG", imu.getHeadingDEG());
            telemetry.update();
        }
    }
}
