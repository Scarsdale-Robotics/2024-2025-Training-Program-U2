package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;

public class DriveSubsystem extends SubsystemBase {
    private final MecanumDrive controller;
    private final Motor leftFront;
    private final Motor rightFront;
    private final Motor leftBack;
    private final Motor rightBack;

    public DriveSubsystem(Motor leftFront, Motor rightFront, Motor leftBack, Motor rightBack) {
        this.rightBack = rightBack;
        this.leftBack = leftBack;
        this.rightFront = rightFront;
        this.leftFront = leftFront;
        controller = new MecanumDrive(
                leftFront,
                rightFront,
                leftBack,
                rightBack
        );
    }

    /**
     * Drives with directions based on robot pov.
     *
     * @param right     How much right the robot should strafe (negative values = strafe left).
     * @param forward   How much forward the robot should move (negative values = move backwards).
     * @param turn      How much the robot should turn.
     */
    public void driveRobotCentric(double right, double forward, double turn) {
        controller.driveRobotCentric(right, forward, turn);
    }

    /**
     * @return the current position of the robot's back left wheel in ticks.
     */
    public int getLBPosition() {
        return leftBack.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's back right wheel in ticks.
     */
    public int getRBPosition() {
        return rightBack.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's front left wheel in ticks.
     */
    public int getLFPosition() {
        return leftFront.getCurrentPosition();
    }

    /**
     * @return the current position of the robot's front right wheel in ticks.
     */
    public int getRFPosition() {
        return rightFront.getCurrentPosition();
    }

    /**
     * Stop the motors.
     */
    public void stopController() {
        controller.stop();
    }

}