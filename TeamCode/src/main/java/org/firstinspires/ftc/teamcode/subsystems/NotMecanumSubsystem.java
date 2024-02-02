package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NotMecanumSubsystem {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;

    private Servo leftClaw;
    private Servo rightClaw;

    // Positions for armMotor
    private final int armStartPos;
    private final int armLimitPos;

    public NotMecanumSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // Initialize servos
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // Set motor directions
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set zero power behavior for motors
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders and set initial positions for armMotor
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armStartPos = armMotor.getCurrentPosition();
        armLimitPos = armStartPos + 1000; // Example limit, adjust based on your robot's design

        // Set initial power for motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);
    }

    public void move(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    public void moveArmToPosition(int position) {
        if (position >= armStartPos && position <= armLimitPos) {
            armMotor.setTargetPosition(position);
            armMotor.setPower(0.5); // Set desired motor power
        }
    }

    public void setClawPositions(double leftPosition, double rightPosition) {
        leftClaw.setPosition(leftPosition);
        rightClaw.setPosition(rightPosition);
    }
}
