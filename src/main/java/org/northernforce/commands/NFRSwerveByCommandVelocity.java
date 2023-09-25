package org.northernforce.commands;

import org.northernforce.subsystems.drive.NFRSwerveDrive;
import org.northernforce.subsystems.ros.ROSCoprocessor;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NFRSwerveByCommandVelocity extends CommandBase
{
    protected final NFRSwerveDrive drive;
    protected final NFRSwerveModuleSetState[] commands;
    protected final ROSCoprocessor coprocessor;
    protected final ChassisSpeeds speeds = new ChassisSpeeds();
    public NFRSwerveByCommandVelocity(NFRSwerveDrive drive, NFRSwerveModuleSetState[] commands, ROSCoprocessor coprocessor)
    {
        addRequirements(drive);
        this.drive = drive;
        this.commands = commands;
        this.coprocessor = coprocessor;
    }
    @Override
    public void initialize()
    {
        for (var command : commands)
        {
            command.schedule();
        }
    }
    @Override
    public void execute()
    {
        var states = drive.toModuleStates(speeds);
        for (int i = 0; i < states.length; i++)
        {
            commands[i].setTargetState(states[i]);
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        for (var command : commands)
        {
            command.cancel();
        }
    }
}