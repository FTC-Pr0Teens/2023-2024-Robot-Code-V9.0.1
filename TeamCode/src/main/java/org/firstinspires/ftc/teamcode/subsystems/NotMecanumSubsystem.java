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

    // Speed limit variables (example, adjust based on your requirements)
    private final double maxSpeed = 0.5; // Maximum speed (0.0 to 1.0)

    public NotMecanumSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // Initialize servos
        leftClaw = hardwareMap.get(Servo.class, "leftClaw");
        rightClaw = hardwareMap.get(Servo.class, "rightClaw");

        // Set motor directions and modes
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClaw.setDirection(Servo.Direction.FORWARD); // Assuming default direction for leftClaw
        rightClaw.setDirection(Servo.Direction.REVERSE); // Reverse direction for rightClaw
    }

    public void move(double leftPower, double rightPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
    }

    // Use this method to control the arm motor speed directly, without encoder-based positioning
    public void controlArmMotor(double power) {
        // Limit the power to maxSpeed to ensure the motor doesn't exceed this speed
        double limitedPower = Math.signum(power) * Math.min(Math.abs(power), maxSpeed);
        armMotor.setPower(limitedPower);
    }

    public void setClawPositions(double leftPosition, double rightPosition) {
        leftClaw.setPosition(leftPosition);
        rightClaw.setPosition(rightPosition);
    }
}
