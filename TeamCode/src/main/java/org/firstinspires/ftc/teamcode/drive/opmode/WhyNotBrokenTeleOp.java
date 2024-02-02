package org.firstinspires.ftc.teamcode.drive.opmode;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.DroneShooter;
import org.firstinspires.ftc.teamcode.subsystems.NotMecanumSubsystem;
import org.firstinspires.ftc.teamcode.util.Specifications;

@TeleOp(name = "WhyNotBrokenTeleOP")
public class WhyNotBrokenTeleOp extends LinearOpMode {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private NotMecanumSubsystem NotMecanumSubsystem;

    private DcMotor ArmMotor;

    private DroneShooter DroneShooter;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while(opModeIsActive()){
            NotMecanumSubsystem.move(gamepad1.left_stick_y, gamepad1.right_stick_y);


        }
    }




}
