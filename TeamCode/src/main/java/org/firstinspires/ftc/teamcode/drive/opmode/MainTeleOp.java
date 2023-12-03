package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.TimeUnit;

@TeleOp(name="Main TeleOp")
public class MainTeleOp extends LinearOpMode{

    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;
//    private MecanumCommand mecanumCommand;
//    private OutputCommand outputCommand;
    private IntakeCommand intakeCommand;
    private IMUSubsystem imuSubsystem;
    private GyroOdometry gyroOdometry;
    private GridAutoCentering gridAutoCentering;
    private OutputCommand outputCommand;
    private ElapsedTime timer;

    @Override
    public void runOpMode() throws InterruptedException {

        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        imuSubsystem = new IMUSubsystem(hardwareMap);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);

//        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);

//        outputCommand = new OutputCommand(hardwareMap);
        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);

        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);
        timer = new ElapsedTime();

        odometrySubsystem.reset();
        imuSubsystem.resetAngle();

        //TODO: Initialize

        waitForStart();

        CompletableFuture.runAsync(this::updateOdometry);

        while(opModeIsActive()){
            if(gamepad1.a){
                timer.reset();
                if(timer.milliseconds() < 2000) {
                    outputCommand.armToIdle();
                    outputCommand.tiltToIdle();
                }
                else {
                    multiMotorCommand.LiftUp(true, 0);
                }
            }
            else if(gamepad1.b){
                multiMotorCommand.LiftUp(true, 3);
            }
            else if(gamepad1.y){
                multiMotorCommand.LiftUp(true, 1);
                outputCommand.armToBoard();
                outputCommand.tiltToBoard();
                if(gamepad1.right_bumper){
                    timer.reset();
                    outputCommand.openGate();
                    while(timer.time(TimeUnit.MILLISECONDS) < 100){}
                    outputCommand.closeGate();
                    outputCommand.outputWheelOut();
                    while(timer.time(TimeUnit.MILLISECONDS) < 100){}
                }
            }
            else if(gamepad1.x){
                multiMotorCommand.LiftUp(true, 2);
            }
            else {
                multiMotorSubsystem.moveLift(0);
            }

            mecanumSubsystem.fieldOrientedMove(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x, imuSubsystem.getTheta());

            if(gamepad1.right_trigger > 0.3){
                intakeCommand.intakeIn(0.7);
            }
            else if(gamepad1.left_trigger > 0.3){
                intakeCommand.intakeOut(0.7);
            }
            else{
                intakeCommand.stopIntake();
            }

            if(gamepad1.dpad_left){
                gridAutoCentering.offsetTargetAngle(-Math.PI/2);
                gridAutoCentering.secondaryProcess(true);
            }
            else if(gamepad1.dpad_right){
                gridAutoCentering.offsetTargetAngle(Math.PI/2);
                gridAutoCentering.secondaryProcess(true);
            }
            else if(gamepad1.dpad_up){
                gridAutoCentering.offsetTargetAngle(0);
                gridAutoCentering.secondaryProcess(true);
            }
            else if(gamepad1.dpad_down){
                gridAutoCentering.offsetTargetAngle(Math.PI);
                gridAutoCentering.secondaryProcess(true);
            } else if (gamepad1.right_bumper) {
                gridAutoCentering.reset();
            }
            else{
                gridAutoCentering.process(false);
            }
            telemetry.addData("liftHeight", multiMotorSubsystem.getPosition());
            telemetry.update();
        }
    }

    private void updateOdometry(){
        while(opModeIsActive()){
            gyroOdometry.odometryProcess();
            gridAutoCentering.secondaryProcess(true);
        }
    }
}
