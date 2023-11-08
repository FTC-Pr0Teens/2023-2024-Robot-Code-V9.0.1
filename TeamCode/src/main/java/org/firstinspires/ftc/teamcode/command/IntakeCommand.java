package org.firstinspires.ftc.teamcode.command;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeCommand {
    private IntakeSubsystem intakeSubsystem;

    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
    }

    public void raiseIntake() {
        intakeSubsystem.setIntakePosition(0.5);
    }

    public void lowerIntake() {
        intakeSubsystem.setIntakePosition(0);
    }

    public void intakeIn() {
        intakeSubsystem.setIntakePower(1);
    }

    public void intakeOut() {
        intakeSubsystem.setIntakePower(-1);
    }

    public void stopIntake() {
        intakeSubsystem.setIntakePower(0);
    }
}