package org.firstinspires.ftc.teamcode.drive.opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OdometrySubsystem;
import org.firstinspires.ftc.teamcode.util.ExpandedHardware;
import org.firstinspires.ftc.teamcode.util.TimerList;

@TeleOp(name = "balls")
public class BulkCacheTest extends LinearOpMode {
    ExpandedHardware eh = new ExpandedHardware(hardwareMap);

    OdometrySubsystem odo;
    TimerList tl = new TimerList();


    //TODO: Switch this to true and false, and log the loop times and how long it takes.
    // In addition,
    //call the function that gets movement positions and see if that's accurate as well.
    // Before you do this second step, plug the most important odometry encoders into port 0 and port 3 of the control hub.
    //
    boolean enableCache = false;

    @Override
    public void runOpMode() throws InterruptedException {
        if(enableCache) eh.enableBulkCache();
        else eh.disableBulkCache();

        waitForStart();

        tl.resetTimer("randomtimerlol");
        while(opModeIsActive()){
            telemetry.addData("back encoder", odo.backEncoder());
            telemetry.addData("right encoder", odo.rightEncoder());
            telemetry.addData("left encoder", odo.leftEncoder());
            telemetry.addData("looptime milliseconds", tl.getTimerMillis("randomtimerlol"));
            telemetry.addData("looptime nanoseconds", tl.getTimerNano("randomtimerlol"));
            telemetry.update();
        }


    }
}
