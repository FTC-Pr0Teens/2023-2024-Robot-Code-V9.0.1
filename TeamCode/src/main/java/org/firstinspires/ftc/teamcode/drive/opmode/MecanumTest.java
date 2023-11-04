package org.firstinspires.ftc.teamcode.drive.opmode;

/***
 * 9 September 2023, Dylan Chunyu
 * This class is a test for a basic 4 wheel mecanum robot
 * having said robot make simple movements
 */

    import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
    import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

    //importing classes from team code
    import org.firstinspires.ftc.teamcode.subsystems.MecanumSubsystem;
    import org.firstinspires.ftc.teamcode.command.MecanumCommand;
    import org.firstinspires.ftc.teamcode.threadopmode.subsystems.OdometrySubsystem;
    import org.firstinspires.ftc.teamcode.threadopmode.subsystems.DistanceSensorSubsystem;
    import org.firstinspires.ftc.teamcode.util.GyroOdometry;
    import org.firstinspires.ftc.teamcode.util.PurePursuit;

public class MecanumTest extends LinearOpMode {
    //objects
    MecanumSubsystem mecanumSubsystem;
    MecanumCommand mecanumCommand;
    OdometrySubsystem odometrySubsystem;
    DistanceSensorSubsystem emptyDistanceSensorPlaceholder; //needed a DistanceSensorSubsystem object to run MecanumCommand, this object is empty
    GyroOdometry gyroOdometry;
    PurePursuit purePursuit;


    @Override
    public void runOpMode() throws InterruptedException {
        mecanumSubsystem = new MecanumSubsystem(hardwareMap);
        odometrySubsystem = new OdometrySubsystem(hardwareMap);
        mecanumCommand = new MecanumCommand(mecanumSubsystem, odometrySubsystem, emptyDistanceSensorPlaceholder, emptyDistanceSensorPlaceholder, emptyDistanceSensorPlaceholder, emptyDistanceSensorPlaceholder, gyroOdometry, this, purePursuit);
        mecanumSubsystem.move(true,20,20,0);
    }
}
