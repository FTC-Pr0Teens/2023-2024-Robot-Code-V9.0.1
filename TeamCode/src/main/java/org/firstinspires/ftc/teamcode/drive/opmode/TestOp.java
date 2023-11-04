//package org.firstinspires.ftc.teamcode.drive.opmode;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
//
//public class TestOp extends LinearOpMode {
//    private MecanumSubsystem mecanumSubsystem;
//    @Override
//    public void runOpMode(){
//        //init
//        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
//        waitForStart();
//
//        //actual code
//        while(opModeIsActive()){
//            mecanumSubsystem.move(true, gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
//            telemetry.update();
//        }
//    }
//}
