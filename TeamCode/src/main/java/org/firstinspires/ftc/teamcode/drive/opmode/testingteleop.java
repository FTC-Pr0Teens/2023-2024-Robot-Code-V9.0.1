package org.firstinspires.ftc.teamcode.drive.opmode;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Specifications;


@TeleOp(name = "Pr0TeensTeleOpV1")
public class testingteleop extends LinearOpMode {
    private MecanumSubsystem mecanumSubsystem;

    private IMUSubsystem imuSubsystem;

    private DcMotorEx leftSlide;
    private DcMotorEx rightSlide;




    private enum RUNNING_STATE { //mini "threads" to run (is actually run in main thread, just controlled simultaneously)
        LOWER_LIFT, RAISE_LIFT
    }
    @SuppressLint("SuspiciousIndentation")
    @Override
    public void runOpMode() throws InterruptedException {
        leftSlide = hardwareMap.get(DcMotorEx.class, "LeftSlideMotor");
        rightSlide = hardwareMap.get(DcMotorEx.class, "RightSlideMotor");

        leftSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        mecanumSubsystem = new MecanumSubsystem(hardwareMap); //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.dpad_down) {
                leftSlide.setPower(-0.2);
                rightSlide.setPower(-0.2);
            } else if (gamepad1.dpad_up) {
                leftSlide.setPower(0.2);
                rightSlide.setPower(0.2);
            } else {
                leftSlide.setPower(0);
                leftSlide.setPower(0);
            }
            mecanumSubsystem.fieldOrientedMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, imuSubsystem.getTheta());


        }

    }

}