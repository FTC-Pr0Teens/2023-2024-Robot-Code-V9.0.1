package org.firstinspires.ftc.teamcode.command;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.SingleMotorSubsystem;
import org.firstinspires.ftc.teamcode.util.Specifications;

public class IntakeCommand {
    private SingleMotorSubsystem intake;
    private Servo intakeServo;

    public IntakeCommand(HardwareMap hardwareMap) {
        intake = new SingleMotorSubsystem(hardwareMap, Specifications.INTAKE_MOTOR);
        intakeServo = hardwareMap.get(Servo.class, Specifications.INTAKE_SERVO);
    }

    public void raiseIntake() {
        intakeServo.setPosition(1);
    }

    public void lowerIntake() {
        intakeServo.setPosition(0);
    }

    public void intakeIn() {
        intake.motorTurnPower(true, 1);
    }

    public void intakeOut() {
        intake.motorTurnPower(true, -1);
    }

    public void stopIntake() {
        intake.motorTurnPower(true, 0);
    }
}