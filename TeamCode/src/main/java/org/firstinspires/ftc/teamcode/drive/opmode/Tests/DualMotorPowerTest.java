package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;

import java.util.concurrent.CompletableFuture;

@Config
@TeleOp
public class DualMotorPowerTest extends LinearOpMode {
    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private MecanumSubsystem mecanumSubsystem;
    private volatile int level = 0;

    public static double kpp = (double) 1/750;;
    public static double kpi = 0;
    public static double kpd = 0;
    public static double kvp = (double) 1/2000;
    public static double kvi = 0;
    public static double kvd = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your hardware components
        FtcDashboard dash = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        ElapsedTime timer = new ElapsedTime();
        double targetPosition = 0;

        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);


        multiMotorSubsystem.reset();
        waitForStart();

        CompletableFuture.runAsync(this::liftProcess);

        while (opModeIsActive()) {
            multiMotorSubsystem.cascadeSetConstants(kpp,kpi,kpd,kvp,kvi,kvd);
            if(gamepad1.a){
                level = 4;
//                multiMotorCommand.LiftUp(true, 4);
            }
            else if(gamepad1.b){
                level = 3;
//                multiMotorCommand.LiftUp(true, 3);
            }
            else if(gamepad1.y){
                level = 2;
            }
            else if(gamepad1.x){
                level = 1;
            }
            else if(gamepad1.dpad_down){
                level = 0;
            }
            else {
                multiMotorSubsystem.moveLift(gamepad1.left_stick_y);
            }
//            mecanumSubsystem.fieldOrientedMove(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 0);
            packet.put("target position", targetPosition);
            packet.put("current position", multiMotorSubsystem.getPosition());
            packet.put("cascade power output", -multiMotorSubsystem.getCascadeOutput());
//            packet.put("power", multiMotorSubsystem.getMainPower());
//            packet.put("auxpower", multiMotorSubsystem.getAux1Power());
//            packet.put("derivativeValue", multiMotorSubsystem.getDerivativeValue());
//            packet.put("errorValue", multiMotorSubsystem.getErrorValue());
//            packet.put("intervalValue", multiMotorSubsystem.getIntervalValue());
//            packet.put("lastErrorValue", multiMotorSubsystem.getLastErrorValue());
//            packet.put("controlleroutput", multiMotorSubsystem.getCascadeOutput());
            packet.put("outputPositionalValue", multiMotorSubsystem.getCascadePositional());
            packet.put("outputPositionalValue", multiMotorSubsystem.getCascadePositional());
            packet.put("outputVelocityValue", multiMotorSubsystem.getCascadeVelocity());
            packet.put("target vel", multiMotorSubsystem.getIntervalValue());
            packet.put("current vel", multiMotorSubsystem.getCascadeVelocity());
            packet.put("level", level);
            packet.put("test boolean", multiMotorCommand.getTestBoolean());

            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("position", multiMotorSubsystem.getPosition());
            telemetry.addData("power", multiMotorSubsystem.getMainPower());
            telemetry.addData("auxpower", multiMotorSubsystem.getAux1Power());
            telemetry.addData("auxpos", multiMotorSubsystem.getAuxPos());
            telemetry.addData("derivativeValue", multiMotorSubsystem.getDerivativeValue());
            telemetry.addData("cascadeOutput", multiMotorSubsystem.getCascadeOutput());
            telemetry.addData("outputPositional", multiMotorSubsystem.getCascadePositional());
            telemetry.addData("outputVelocity", multiMotorSubsystem.getCascadeVelocity());
            telemetry.addData("level", level);
            telemetry.update();
            dash.sendTelemetryPacket(packet);
        }
    }

    public void liftProcess(){
        while(opModeIsActive()){
            multiMotorCommand.LiftUp(true, level);
        }
    }
}
