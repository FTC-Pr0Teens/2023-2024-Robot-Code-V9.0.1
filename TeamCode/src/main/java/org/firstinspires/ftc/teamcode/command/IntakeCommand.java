package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Specifications;

public class IntakeCommand {
    private SingleMotorSubsystem intake;
    private Servo intakeServo;
    private CRServo intakeRoller;

    public IntakeCommand(HardwareMap hardwareMap) {
        intake = new SingleMotorSubsystem(hardwareMap, Specifications.INTAKE_MOTOR);
        intakeServo = hardwareMap.get(Servo.class, Specifications.INTAKE_SERVO);
        intakeRoller = hardwareMap.get(CRServo.class, Specifications.INTAKE_ROLLER);
    }

    public void raiseIntake() {
        intakeServo.setPosition(1);
    }

    public void lowerIntake() {
        intakeServo.setPosition(0);
    }

    public void intakeIn(double power) {
        intake.motorTurnPower(true, Math.abs(power));
        intakeRoller.setPower(-1);
    }

    public void intakeOut(double power) {
        intake.motorTurnPower(true, -Math.abs(power));
    }

    public void stopIntake() {
        intake.motorTurnPower(true, 0);
        intakeRoller.setPower(0);
    }
}