package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;

@Config
@TeleOp
public class DualMotorPowerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize your hardware components
        FtcDashboard dash = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        ElapsedTime timer = new ElapsedTime();
        double targetPosition = 0;

        MultiMotorSubsystem multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);

        waitForStart();

        while (opModeIsActive()) {


            if(gamepad1.a){
                multiMotorSubsystem.LiftCascadeProcess(4100);
                targetPosition = 4100;
            }
            else {
                multiMotorSubsystem.moveLift(gamepad1.left_stick_y);
            }

            packet.put("position", multiMotorSubsystem.getPosition());
            packet.put("power", multiMotorSubsystem.getMainPower());
            packet.put("auxpower", multiMotorSubsystem.getAux1Power());
            packet.put("derivativeValue", multiMotorSubsystem.getDerivativeValue());
            packet.put("errorValue", multiMotorSubsystem.getErrorValue());
            packet.put("intervalValue", multiMotorSubsystem.getIntervalValue());
            packet.put("lastErrorValue", multiMotorSubsystem.getLastErrorValue());
            packet.put("controlleroutput", multiMotorSubsystem.getCascadeOutput());
            packet.put("outputPositionalValue", multiMotorSubsystem.getCascadePositional());
            packet.put("outputVelocityValue", multiMotorSubsystem.getCascadeVelocity());
            packet.put("Target Position", targetPosition);
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("position", multiMotorSubsystem.getPosition());
            telemetry.addData("power", multiMotorSubsystem.getMainPower());
            telemetry.addData("auxpower", multiMotorSubsystem.getAux1Power());
            telemetry.addData("auxpos", multiMotorSubsystem.getAuxPos());
            telemetry.addData("derivativeValue", multiMotorSubsystem.getDerivativeValue());
            telemetry.update();
            dash.sendTelemetryPacket(packet);
        }
    }
}
