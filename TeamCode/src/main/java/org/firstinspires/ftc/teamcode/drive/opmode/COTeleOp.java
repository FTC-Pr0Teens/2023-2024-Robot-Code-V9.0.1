package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.command.OutputCommand;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;

public class COTeleOp extends LinearOpMode {
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
    private ColorSensorSubsystem colorSensorSubsystem;
    private ElapsedTime timer;
    private int level = -1;

    private enum RUNNING_STATE {
        LIFT_STOP,
        RETRACT_LIFT,
        RAISE_LIFT,
        DROP
    }
    private RUNNING_STATE state = RUNNING_STATE.LIFT_STOP;

    @Override
    public void runOpMode() throws InterruptedException {
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        imuSubsystem = new IMUSubsystem(hardwareMap);

        mecanumSubsystem = new MecanumSubsystem(hardwareMap); //MAKE SURE THIS IS INSTANTIATED BEFORE ODOMETRY SUBSYSTEM

        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);

//        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);

        intakeCommand = new IntakeCommand(hardwareMap);
        outputCommand = new OutputCommand(hardwareMap);

        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

        colorSensorSubsystem = new ColorSensorSubsystem(hardwareMap);

        timer = new ElapsedTime();

        odometrySubsystem.reset();
        imuSubsystem.resetAngle();

        intakeCommand.lowerIntake();
        outputCommand.closeGate();

        outputCommand.armToIdle();
        outputCommand.tiltToIdle();

//        disableAutoLift = false;


        waitForStart();

        //emergency lift change for owen

        while(opModeIsActive()) {
            mecanumSubsystem.fieldOrientedMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x, imuSubsystem.getTheta());

            //setting levels for running lift
            if (state == RUNNING_STATE.LIFT_STOP) {
                //set lift level
                if (gamepad1.a) {
                    level = 1;
                    state = RUNNING_STATE.RAISE_LIFT;
                } else if (gamepad1.b) {
                    level = 2;
                    state = RUNNING_STATE.RAISE_LIFT;
                } else if (gamepad1.y) {
                    level = 3;
                    state = RUNNING_STATE.RAISE_LIFT;
                } else if (gamepad1.x) {
                    level = 4;
                    state = RUNNING_STATE.RAISE_LIFT;
                }
            }

            //when lift is raised
            if (state == RUNNING_STATE.RAISE_LIFT) {
                //TODO: raise and align tilt + arm
                //drop pixel when button is pressed
                if(gamepad2.right_bumper){
                    //drop pixel (one)
                    dropPixel();
                    state = RUNNING_STATE.DROP;
                }
            }

            if(state == RUNNING_STATE.DROP){
                if(gamepad2.left_bumper){
                    //drop second pixel
                    dropPixel();
                    state = RUNNING_STATE.RETRACT_LIFT;
                }
                else if(gamepad2.b){
                    //just retract no drop
                    state = RUNNING_STATE.RETRACT_LIFT;
                }
            }

            if(state == RUNNING_STATE.RETRACT_LIFT && (gamepad2.dpad_down || gamepad1.dpad_down)){
                level = 0;
                if(multiMotorSubsystem.getPosition() < 35){
                    state = RUNNING_STATE.LIFT_STOP;
                }
            }


            //emergency lift controls
            if(Math.abs(gamepad2.left_stick_y) > 0.4){
                state = RUNNING_STATE.LIFT_STOP;
                multiMotorSubsystem.moveLift(gamepad2.left_stick_y);
            }
            if(gamepad2.a){
                state = RUNNING_STATE.RETRACT_LIFT;
            }

            //intake
            if(gamepad2.right_trigger > 0.5){
                intakeCommand.intakeIn(0.7);
            }
            else if(gamepad2.left_trigger > 0.5){
                intakeCommand.intakeOut(0.7);
            }
            else{
                intakeCommand.stopIntake();
            }
        }

    }

    public void LiftProcess(){
        while(opModeIsActive()){
            if(state != RUNNING_STATE.LIFT_STOP) {
                multiMotorCommand.LiftUp(true, level);
            }
        }
    }
    public void dropPixel(){
        //TODO: put code
    }
}
