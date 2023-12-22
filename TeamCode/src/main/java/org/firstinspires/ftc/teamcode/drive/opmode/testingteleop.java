package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;


@TeleOp(name = "Pr0TeensTeleOpV1")
public class testingteleop extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;

    private IMUSubsystem imuSubsystem;

    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        imuSubsystem = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap); //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM

        waitForStart();

        while (opModeIsActive()) {
            mecanumSubsystem.fieldOrientedMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, imuSubsystem.getTheta());


        }


    }
}