package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

@TeleOp(name="drive test")
public class MecanumTest extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;
    private IMUSubsystem imu;
    private MecanumCommand drive;
    @Override
    public void runOpMode() throws InterruptedException {

        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        imu = new IMUSubsystem(hardwareMap);

        drive = new MecanumCommand(mecanumSubsystem, null, new GyroOdometry(null, null), this);

        imu.resetAngle();

        waitForStart();

        while (opModeIsActive()) {
            //drive.move(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);
            mecanumSubsystem.fieldOrientedMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, 0);

//            telemetry.addData("Heading in DEG", imu.getHeadingDEG());
            telemetry.addData("leftFront", mecanumSubsystem.getLeftForward().getPower());
            telemetry.addData("rightFront", mecanumSubsystem.getRightForward().getPower());
            telemetry.addData("leftBack", mecanumSubsystem.getLeftBack().getPower());
            telemetry.addData("rightBack", mecanumSubsystem.getRightBack().getPower());
            telemetry.addData("gamepad1.left_stick_y", gamepad1.left_stick_y);
            telemetry.addData("gamepad1.left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("gamepad1.right_stick_x", gamepad1.right_stick_x);
            telemetry.update();
        }
    }
}
