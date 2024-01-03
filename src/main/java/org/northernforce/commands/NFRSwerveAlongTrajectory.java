package org.northernforce.commands;

import java.util.function.Supplier;

import org.northernforce.subsystems.drive.NFRSwerveDrive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

/**
 * A Command designed to provide a swerve drive with the ability to follow a trajectory.
 */
public class NFRSwerveAlongTrajectory extends SwerveControllerCommand
{
    protected final NFRSwerveDrive drive;
    protected final HolonomicDriveController controller;
    /**
     * Creates a new NFRSwerveAlongTrajectory.
     * @param drive the swerve drive subsystem
     * @param setStateCommands the commands to be used for setting the states of the swerve modules
     * @param trajectory the trajectory to follow
     * @param poseSupplier the pose supplier
     * @param controller the controller that is to be used
     */
    public NFRSwerveAlongTrajectory(NFRSwerveDrive drive, NFRSwerveModuleSetState[] setStateCommands, Trajectory trajectory,
        Supplier<Pose2d> poseSupplier, HolonomicDriveController controller)
    {
        super(trajectory, poseSupplier, drive.getKinematics(), controller, states -> {
            for (int i = 0; i < states.length; i++)
            {
                setStateCommands[i].setTargetState(states[i]);
            }
        }, drive);
        this.drive = drive;
        this.controller = controller;
    }
    /**
     * Creates a new NFRSwerveAlongTrajectory.
     * @param drive the swerve drive subsystem
     * @param setStateCommands the commands to be used for setting the states of the swerve modules
     * @param trajectory the trajectory to follow
     * @param poseSupplier the pose supplier
     * @param controller the controller that is to be used
     * @param targetRotation the target rotation that the swerve drive will go to as it follows the translational
     * components of the trajectory.
     */
    public NFRSwerveAlongTrajectory(NFRSwerveDrive drive, NFRSwerveModuleSetState[] setStateCommands, Trajectory trajectory,
        Supplier<Pose2d> poseSupplier, HolonomicDriveController controller, Supplier<Rotation2d> targetRotation)
    {
        super(trajectory, poseSupplier, drive.getKinematics(), controller, targetRotation, states -> {
            for (int i = 0; i < states.length; i++)
            {
                setStateCommands[i].setTargetState(states[i]);
            }
        }, drive);
        this.drive = drive;
        this.controller = controller;
    }
    /**
     * Returns whether at the end of the trajectory.
     * @return whether the estimated time has elapsed AND the controller is at the reference (at the setpoint).
     */
    @Override
    public boolean isFinished()
    {
        return super.isFinished() && controller.atReference();
    }
}
