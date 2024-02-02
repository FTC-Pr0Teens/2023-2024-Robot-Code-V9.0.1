package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NotMecanumSubsystem {
    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private DcMotor armMotor;

    private CRServo continuousServo;

    public NotMecanumSubsystem(HardwareMap hardwareMap) {
        // Initialize motors
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");

        // Initialize continuous rotation servo
        continuousServo = hardwareMap.get(CRServo.class, "continuousServo");

        // Set motor directions
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set motor modes
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set zero power behavior for motors
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set initial power for motors
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        armMotor.setPower(0);

        // Set initial power for continuous rotation servo
        continuousServo.setPower(0);
    }

    public void move(double leftPower, double rightPower, double armPower) {
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);
        armMotor.setPower(armPower);
    }

    public void setContinuousServoPower(double power) {
        continuousServo.setPower(power);
    }
}
