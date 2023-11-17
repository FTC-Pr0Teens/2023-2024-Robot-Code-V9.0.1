package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.subsystems.OdometrySubsystem;

public class MainTeleOp extends LinearOpMode{

    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
//    private OutputCommand outputCommand;
    private IntakeSubsystem intakeSubsystem;
    private IntakeCommand intakeCommand;
    @Override
    public void runOpMode() throws InterruptedException {

        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        odometrySubsystem = new OdometrySubsystem(hardwareMap);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, null, null, null, null, null, this, null);

//        outputCommand = new OutputCommand(hardwareMap);

        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        intakeCommand = new IntakeCommand(intakeSubsystem);

        waitForStart();

        mecanumCommand.moveGlobalPartial();

    }
}
