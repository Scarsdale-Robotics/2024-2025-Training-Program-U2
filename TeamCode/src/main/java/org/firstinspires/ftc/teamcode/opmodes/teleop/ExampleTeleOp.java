package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.HardwareRobot;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

// Name your program here (it will show up as an option when we run it on the robot's Driver Hub)
@TeleOp(name="Totally Amazing TeleOp")
public class ExampleTeleOp extends LinearOpMode {

    // This override exists because we're writing our own implementation for runOpMode().
    @Override
    public void runOpMode() {
        // Initializing the software.
        // Create a HardwareRobot object, which holds all of the robot's Motor, Servo, etc. objects.
        HardwareRobot robot = new HardwareRobot(hardwareMap);
        // Create a DriveSubsystem object, which provides an easy way to drive the robot.
        DriveSubsystem drive = new DriveSubsystem(
                robot.leftFront,
                robot.rightFront,
                robot.leftBack,
                robot.rightBack
        );

        // Wait for Start button press...
        waitForStart();


        // For while loops, you NEED to put opModeIsActive() as the condition, or else the robot
        // won't stop when stopped!
        while (opModeIsActive()) {
            // Read Gamepad inputs.
            double rightPower = gamepad1.left_stick_x;
            double forwardPower = -gamepad1.left_stick_y;
            double turnPower = gamepad1.right_stick_x;

            // Set the drive powers.
            drive.driveRobotCentric(rightPower, forwardPower, turnPower);

            //...and loop!
        }

    }

}