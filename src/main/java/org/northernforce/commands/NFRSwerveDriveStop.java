package org.northernforce.commands;

import org.northernforce.subsystems.drive.NFRSwerveDrive;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NFRSwerveDriveStop extends CommandBase
{
    protected final NFRSwerveDrive drive;
    protected final NFRSwerveModuleSetState[] setStateCommands;
    protected final boolean optimize;
    public NFRSwerveDriveStop(NFRSwerveDrive drive, NFRSwerveModuleSetState[] setStateCommands, boolean optimize)
    {
        addRequirements(drive);
        this.drive = drive;
        this.setStateCommands = setStateCommands;
        this.optimize = optimize;
    }
    @Override
    public void initialize()
    {
        for (NFRSwerveModuleSetState command : setStateCommands)
        {
            command.schedule();
        }
        SwerveModuleState[] states = drive.getStopState();
        for (int i = 0; i < setStateCommands.length; i++)
        {
            setStateCommands[i].setTargetState(optimize ? SwerveModuleState.optimize(states[i],
                drive.getModules()[i].getRotation()) : states[i]);
        }
    }
}
