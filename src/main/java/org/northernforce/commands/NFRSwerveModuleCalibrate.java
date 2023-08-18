package org.northernforce.commands;

import org.northernforce.subsystems.drive.swerve.NFRSwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
public class NFRSwerveModuleCalibrate extends CommandBase
{
    protected final NFRSwerveModule module;
    protected final Rotation2d angle;
    public NFRSwerveModuleCalibrate(NFRSwerveModule module, Rotation2d angle)
    {
        addRequirements(module);
        this.module = module;
        this.angle = angle;
    }
    @Override
    public void initialize()
    {
        module.resetAngle(angle);
    }
    @Override
    public boolean isFinished()
    {
        return true;
    }
}
