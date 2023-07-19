package org.northernforce.commands;

import java.util.function.DoubleSupplier;

import org.northernforce.subsystems.drive.NFRSwerveDrive;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NFRSwerveDriveWithJoystick extends CommandBase
{
    protected final NFRSwerveDrive drive;
    protected final NFRSwerveModuleSetState[] setStateCommands;
    protected final DoubleSupplier xSupplier, ySupplier, thetaSupplier;
    protected final boolean optimize;
    public NFRSwerveDriveWithJoystick(NFRSwerveDrive drive, NFRSwerveModuleSetState[] setStateCommands,
        DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier thetaSupplier, boolean optimize)
    {
        addRequirements(drive);
        this.drive = drive;
        this.setStateCommands = setStateCommands;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.thetaSupplier = thetaSupplier;
        this.optimize = optimize;
    }
    @Override
    public void initialize()
    {
        for (NFRSwerveModuleSetState command : setStateCommands)
        {
            command.schedule();
        }
    }
    @Override
    public void execute()
    {
        ChassisSpeeds speeds = new ChassisSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSupplier.getAsDouble());
        SwerveModuleState[] states = drive.toModuleStates(speeds);
        states = drive.scaleSpeeds(states);
        for (int i = 0; i < states.length; i++)
        {
            setStateCommands[i].setTargetState(optimize ? SwerveModuleState.optimize(states[i],
                drive.getModules()[i].getRotation()) : states[i]);
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        for (NFRSwerveModuleSetState command : setStateCommands)
        {
            command.cancel();
        }
    }
}