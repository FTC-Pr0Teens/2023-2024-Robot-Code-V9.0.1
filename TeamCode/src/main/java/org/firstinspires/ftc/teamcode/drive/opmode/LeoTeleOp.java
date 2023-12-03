package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.IntakeCommand;
import org.firstinspires.ftc.teamcode.command.MecanumCommand;
import org.firstinspires.ftc.teamcode.command.MultiMotorCommand;
import org.firstinspires.ftc.teamcode.subsystems.IMUSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MultiMotorSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.GridAutoCentering;
import org.firstinspires.ftc.teamcode.util.GyroOdometry;
import org.firstinspires.ftc.teamcode.util.Specifications;
import org.firstinspires.ftc.teamcode.util.TimerList;
import org.firstinspires.ftc.teamcode.util.VectorCartesian;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Executor;
import java.util.concurrent.Executors;

@TeleOp(name="LeoOp")
public class LeoTeleOp extends LinearOpMode{

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
    private VectorCartesian controllerVector;

    private enum PIXEL_COLOR{
        white,
        yellow,
        purple,
        green
    }

    //state variables
    private double rotation = 0;
    private double velMultiplier = 0;
    private boolean right_stick_pressed = false;
    private boolean left_stick_pressed = false;

    private boolean doCentering = false;
    private double autoCenterAngle = 0;
    private TimerList timers = new TimerList();

    private ArrayList<PIXEL_COLOR> pixelQueue = new ArrayList<>(); // queue of pixels ready to be deposited

    private RevColorSensorV3 firstColorSensor;
    private RevColorSensorV3 secondColorSensor;
    private byte fieldOrientedleftRight = 0; //LEFT = 2, RIGHT = 1, UNSET = 0
    private boolean altFieldOriented = false;
    private double savedCTheta = 0;



    /* Controls P1 - Leo (i apologize if my controls are pain to get used to it)

    LEFT SIDE:                                                      RIGHT SIDE
        Hold trigger = raise lift                               Trigger = drop pixel
        Bumper = intake                                         Bumper = sprint button

        UP =                                                        Y = increase lift level (when depositing pixels)
   LEFT = reset gyro (BOARD IS ON LEFT SIDE)                    X =   B =
   RIGHT = reset gyro (BOARD IS ON RIGHT SIDE)                       A = decrease lift level
  DOWN =

        stick: movement                                             stick: rotation
        press stick: toggle field oriented by left/right            hold stick: autocenter
      90 degrees (depending on if left or right is pressed

        how to reset gyro: align robot forward (to opponent side) then press the according button.
    */
    /* Controls P2 - Kevin

    any two buttons to raise hang / lower hang
    any two buttons to shoot plane (make them hard to reach)


     */



    @Override
    public void runOpMode() throws InterruptedException {
        //multithread
        Executor executor = Executors.newFixedThreadPool(4);

        //classes
        multiMotorSubsystem = new MultiMotorSubsystem(hardwareMap, true, MultiMotorSubsystem.MultiMotorType.dualMotor);
        multiMotorCommand = new MultiMotorCommand(multiMotorSubsystem);

        imuSubsystem = new IMUSubsystem(hardwareMap);
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        gyroOdometry = new GyroOdometry(odometrySubsystem, imuSubsystem);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);

        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem,  gyroOdometry, this);

//        outputCommand = new OutputCommand(hardwareMap);
        intakeCommand = new IntakeCommand(hardwareMap);

        gridAutoCentering = new GridAutoCentering(mecanumSubsystem, gyroOdometry);

        controllerVector = new VectorCartesian(0,0,0);

        firstColorSensor = hardwareMap.get(RevColorSensorV3.class, Specifications.FIRST_COLOR_SENSOR);
        secondColorSensor = hardwareMap.get(RevColorSensorV3.class, Specifications.SECOND_COLOR_SENSOR);

        waitForStart();
        CompletableFuture.runAsync(this::updateIMU, executor);
        CompletableFuture.runAsync(this::updateMovement, executor);

        while(opModeIsActive()){

            runMovement();
            processFieldOriented();
            runIntake();

            //TODO: add pixel drop (and ensuring that it doesnt snap itself), add lift, test movement, add hang, add drone
            //



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


            if(gamepad1.right_trigger > 0.5){
                intakeCommand.intakeIn(0.7);
            }
            else if(gamepad1.left_trigger > 0.7){
                intakeCommand.intakeOut(0.4);
            }
            else{
                intakeCommand.stopIntake();
            }

            if(gamepad1.dpad_left){
                gridAutoCentering.offsetTargetAngle(-Math.PI/2);

            }
            else if(gamepad1.dpad_right){
                gridAutoCentering.offsetTargetAngle(Math.PI/2);

            }
            else if(gamepad1.dpad_up){
                gridAutoCentering.offsetTargetAngle(0);

            }
            else if(gamepad1.dpad_down){
                gridAutoCentering.offsetTargetAngle(Math.PI);
            }
        }
    }

    private void runIntake(){


    }


    /**
     * Basically will do a bunch of magic fuckery, don't bother having me explain cuz even i dont understand what im doing
     * - leo
     */
    private void processFieldOriented(){
        /**
         * These two will reset angle headings of the IMU, both field oriented and autocenter
         */
        //dont enable resetting if in "field is fucked up" state
        if(!altFieldOriented) {
            //This means that backdrop is to to the LEFT (meaning you are on BLUE side)
            if (gamepad1.dpad_left) {
                imuSubsystem.resetAngle(); //for gyro odometry
                gridAutoCentering.reset(); //reset grid heading
                savedCTheta = imuSubsystem.getCTheta(); //save the current orientation's forward angle (this will be fucked with in a bit, hence why we save)
                fieldOrientedleftRight = 2; //LEFT
            }

            //This means that backdrop is to to the RIGHT (meaning you are on RED side)
            if (gamepad1.dpad_right) {
                imuSubsystem.resetAngle(); //for gyro odometry
                gridAutoCentering.reset(); //reset grid heading
                savedCTheta = imuSubsystem.getCTheta();
                fieldOrientedleftRight = 1; //RIGHT
            }
        }

        //toggle
        if(gamepad1.left_stick_button){
            if(!left_stick_pressed) {
                left_stick_pressed = true;

                if(altFieldOriented){
                    altFieldOriented = false;
                    doCentering = true;

                    imuSubsystem.resetAngle(savedCTheta); //reset field to old angle
                    if(fieldOrientedleftRight == 2){
                        autoCenterAngle = -Math.PI/4; //turns left 45 degrees
                    } else if(fieldOrientedleftRight == 1){
                        autoCenterAngle = Math.PI/4;  //turns left 45 degrees to face other junction
                    }
                }

                //if button is pressed: go alt field
                switch(fieldOrientedleftRight){
                    case 2: //LEFT
                        //basically what's gonna happen now is that we're going to literally change the center heading
                        doCentering = true;
                        altFieldOriented = true;
                        autoCenterAngle = 0; //align to center heading of altField (aka left or right)

                        imuSubsystem.resetAngle(savedCTheta + Math.PI/2); //Change field to face left

                        break;
                    case 1:
                        doCentering = true;
                        altFieldOriented = true;
                        autoCenterAngle = 0; //align to center heading of altField (aka left or right)

                        imuSubsystem.resetAngle(savedCTheta - Math.PI/2); //Change field to face left
                        break;

                }
            }
        } else {
            if(left_stick_pressed) left_stick_pressed = false;
        }

    }

    private void runMovement(){

        rotation = gamepad1.right_stick_x;

        //handle autocenter and turning
        if (!gamepad1.right_stick_button) { //Unpressed control set
            if (right_stick_pressed){ //Keep locked in "pressed" state until reset to 0
                if (gamepad1.right_stick_x == 0) {
                    right_stick_pressed = false;
                }

            }
            if (Math.abs(rotation) > 0.5) {
                doCentering = false;
            }
            if (!right_stick_pressed)  //Normal operation
                rotation = -gamepad1.right_stick_x;
        } else {
            right_stick_pressed = true;
            doCentering = true;
            rotation = 0;

            //turn
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
                    break;
            }
            gridAutoCentering.offsetTargetAngle(autoCenterAngle);
        }

        //movement

        mecanumSubsystem.fieldOrientedMove(gamepad1.left_stick_x, -gamepad1.left_stick_y, rotation, 0);
    }






    /**
     * basically gets the values of subsystems without calling it over and over again per loop
     */
    private void updateSubsystemStates(){
        //basically like get the color sensor values

        if(timers.checkTimePassed("colorSensorUpdate", 200)) { //if 200 milliseconds passed
            timers.resetTimer("colorSensorUpdate");
            //update color sensors and store data in sensordata wrapper class below (im going to sleep im dead)-

        }


    }
    private final class SensorData{
        double red;
        double blue;
        double green;
        
        void setRed(double red){
            this.red = red;
        }
        void setBlue(double blue){
            this.blue = blue;
        }
        void setGreen(double green){
            this.green = green;
        }
    }

    /**
     *
     */
    private void updateIMU(){
        imuSubsystem.gyroProcess(); //potentially redundant as it is processed in the gyroOdometry.angleProcess()
        gyroOdometry.angleProcess();
    }

    /**
     *
     */
    private void updateMovement(){
        gridAutoCentering.process(true);
        mecanumSubsystem.motorProcessTeleOp();
    }
}
