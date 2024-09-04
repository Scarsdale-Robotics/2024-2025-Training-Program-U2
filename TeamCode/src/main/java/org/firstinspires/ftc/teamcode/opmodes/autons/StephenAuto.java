package org.firstinspires.ftc.teamcode.opmodes.autons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

@Autonomous(name="Stephen's Autonomous")
public class StephenAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );

        waitForStart();

        // Open-loop control.
        drive.driveRobotCentric(0, 0.5, 0);

        sleep(3000);

        drive.stopController();

        // P Controller
        double kP = 0.001;
        double setpoint = 500;
        while (opModeIsActive()) {
            // Calculate error
            double pv = drive.getRightBackPosition();
            double et = setpoint - pv;

            // Calculate output
            double ut = kP*et;

            drive.driveRobotCentric(0, ut, 0);
        }

    }
}