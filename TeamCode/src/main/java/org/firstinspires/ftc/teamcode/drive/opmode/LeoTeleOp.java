package org.firstinspires.ftc.teamcode.drive.opmode;

//Thread
import org.firstinspires.ftc.teamcode.threadopmode.TaskThread;
import org.firstinspires.ftc.teamcode.threadopmode.ThreadOpMode;


//Command
import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;

//Subsystem
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.threadopmode.subsystems.OdometrySubsystem;

//Util
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.VectorCartesian;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Timer;

public class LeoTeleOp extends ThreadOpMode{

    private MultiMotorSubsystem multiMotorSubsystem;
    private MultiMotorCommand multiMotorCommand;
    private OdometrySubsystem odometrySubsystem;
    private MecanumSubsystem mecanumSubsystem;
    private MecanumCommand mecanumCommand;
//    private OutputCommand outputCommand;
    private IntakeCommand intakeCommand;
    private IMUSubsystem imuSubsystem;
    private GyroOdometry gyroOdometry;
    private GridAutoCentering gridAutoCentering;

    private FtcDashboard dash;
    private TelemetryPacket packet;
    private VoltageSensor voltageSensor;

    //Robot state variables
    private double autoCenterAngle = 0;
    private boolean doCentering = false;
    private boolean right_stick_pressed = false;

    private VectorCartesian controllerVector;
    private double velocityMultiplier;

    private ElapsedTime timer;


    @Override
    public void mainInit() {

        //Lift

        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        //IMU and heading
        imuSubsystem = new IMUSubsystem(hardwareMap, null);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem, null);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);

        //Movement
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, null, null, null, null, gyroOdometry, this, null);

//        outputCommand = new OutputCommand(hardwareMap);
        intakeCommand = new IntakeCommand(hardwareMap);
        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);
        controllerVector = new VectorCartesian(0, 0, 0);
        dash = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        timer = new ElapsedTime();
        timer.reset();

        //Threads
        registerThread(new TaskThread(new TaskThread.Actions() { //process mecanum movement and rotation
            @Override
            public void loop() {
                thread_baseControls();
            }
        }));
    }

    public void thread_baseControls() {
        runMovement();
        gridAutoCentering.process(doCentering);
    }

    public void mainLoop(){

        if(gamepad1.a){
            multiMotorCommand.LiftUp(true, 0);
        }
        else if(gamepad1.b){
            multiMotorCommand.LiftUp(true, 3);
        }
        else if(gamepad1.y){
            multiMotorCommand.LiftUp(true, 1);
        }
        else if(gamepad1.x){
            multiMotorCommand.LiftUp(true, 2);
        }
        else {
            multiMotorSubsystem.moveLift(0);
        }

        mecanumCommand.moveGlobalPartial(true, gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);

        if(gamepad1.right_trigger > 0.5){
            intakeCommand.intakeIn();
        }
        else if(gamepad1.left_trigger > 0.5){
            intakeCommand.intakeOut();
        }
        else{
            intakeCommand.stopIntake();
        }

        if(gamepad1.dpad_left){
            gridAutoCentering.offsetTargetAngle(-Math.PI/2);
            gridAutoCentering.process(true);
        }
        else if(gamepad1.dpad_right){
            gridAutoCentering.offsetTargetAngle(Math.PI/2);
            gridAutoCentering.process(true);
        }
        else if(gamepad1.dpad_up){
            gridAutoCentering.offsetTargetAngle(0);
            gridAutoCentering.process(true);
        }
        else if(gamepad1.dpad_down){
            gridAutoCentering.offsetTargetAngle(Math.PI);
            gridAutoCentering.process(true);
        } else if (gamepad1.right_bumper) {
            gridAutoCentering.reset();
        }
        else{
            gridAutoCentering.process(false);
        }


    }
    public void runMovement(){
        velocityMultiplier = 0.7 + gamepad1.right_trigger * 0.4;

        //change the rotational joystick control scheme if pressed down (does autocentering presets instead)
        if (!gamepad1.right_stick_button) {
            if (right_stick_pressed) //don't enable turning unless stick is reset back to default after pressing stick
                if (gamepad1.right_stick_x == 0) {
                    right_stick_pressed = false;
                }
            if (Math.abs(gamepad1.right_stick_x) > 0.5) {
                doCentering = false;
            }

        } else {
            right_stick_pressed = true;
            doCentering = true;

            if (Math.abs(gamepad1.right_stick_x) < 0.5 && Math.abs(gamepad1.right_stick_y) < 0.5) {
                //set centering angle offset to current angle
                gridAutoCentering.offsetTargetAngle(gyroOdometry.theta);
            } else {
                switch (controllerVector.returnStickPosition(gamepad1.right_stick_x, -gamepad1.right_stick_y)) {
                    case 1: //straight
                        autoCenterAngle = 0;
                        break;
                    case 7: //left
                    case 8: //up left
                    case 6: //down left
                        autoCenterAngle = Math.PI / 2;
                        break;
                    case 3: //right
                    case 2: //up right
                    case 4: //down right
                        autoCenterAngle = -Math.PI / 2;
                        break;
                    case 5: //down
                        autoCenterAngle = Math.PI;
                        break;
                }
                gridAutoCentering.offsetTargetAngle(autoCenterAngle);
            }
        }
        
        if (gamepad1.left_bumper) { //toggle grid
            imuSubsystem.resetAngle(); //for gyro odometry
            imuSubsystem.resetAngleXY(); //for antitip
            gridAutoCentering.reset(); //reset grid heading
        }

    }
}
