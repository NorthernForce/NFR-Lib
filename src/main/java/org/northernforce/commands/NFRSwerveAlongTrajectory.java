package org.northernforce.commands;

import java.util.function.Supplier;

import org.northernforce.subsystems.drive.NFRSwerveDrive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class NFRSwerveAlongTrajectory extends SwerveControllerCommand
{
    protected final NFRSwerveDrive drive;
    protected final HolonomicDriveController controller;
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
    @Override
    public boolean isFinished()
    {
        return super.isFinished() && controller.atReference();
    }
}
