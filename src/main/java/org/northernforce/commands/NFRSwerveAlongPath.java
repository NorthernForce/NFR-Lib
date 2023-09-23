package org.northernforce.commands;

import org.northernforce.subsystems.drive.NFRSwerveDrive;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class NFRSwerveAlongPath extends CommandBase
{
    protected final NFRSwerveDrive drive;
    protected final SwerveControllerCommand command;
    protected final NFRSwerveModuleSetState[] setStateCommands;
    protected final HolonomicDriveController controller;
    public NFRSwerveAlongPath(NFRSwerveDrive drive, Trajectory trajectory, HolonomicDriveController controller,
        NFRSwerveModuleSetState[] setStateCommands)
    {
        this.drive = drive;
        this.setStateCommands = setStateCommands;
        this.controller = controller;
        command = new SwerveControllerCommand(trajectory, drive::getEstimatedPose, drive.getKinematics(), controller, this::setModuleStates,
            drive);
    }
    @Override
    public void initialize()
    {
        command.initialize();
        for (var setStateCommand : setStateCommands)
        {
            setStateCommand.initialize();
        }
    }
    @Override
    public void execute()
    {
        command.execute();
        for (var setStateCommand : setStateCommands)
        {
            setStateCommand.execute();
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        command.end(interrupted);
        for (var setStateCommand : setStateCommands)
        {
            setStateCommand.end(interrupted);
        }
    }
    @Override
    public boolean isFinished()
    {
        return command.isFinished() && controller.atReference();
    }
    public void setModuleStates(SwerveModuleState[] states)
    {
        for (int i = 0; i < states.length; i++)
        {
            setStateCommands[i].setTargetState(states[i]);
        }
    }
}