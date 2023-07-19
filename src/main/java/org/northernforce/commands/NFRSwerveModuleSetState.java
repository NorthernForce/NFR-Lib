package org.northernforce.commands;

import org.northernforce.subsystems.drive.swerve.NFRSwerveModule;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NFRSwerveModuleSetState extends CommandBase
{
    protected final NFRSwerveModule module;
    protected final int velocityPidSlot, positionalPidSlot;
    protected final boolean useTrapezoidalPositioning, useVelocityClosedLoop, usePositionalClosedLoop;
    protected final PIDController pidController;
    protected SwerveModuleState state;
    public NFRSwerveModuleSetState(NFRSwerveModule module, int velocityPidSlot, int positionalPidSlot,
        boolean useTrapezoidalPositioning)
    {
        addRequirements(module);
        this.module = module;
        this.velocityPidSlot = velocityPidSlot;
        this.positionalPidSlot = positionalPidSlot;
        this.useTrapezoidalPositioning = useTrapezoidalPositioning;
        useVelocityClosedLoop = true;
        usePositionalClosedLoop = true;
        pidController = null;
        state = null;
    }
    public NFRSwerveModuleSetState(NFRSwerveModule module, int velocityPidSlot, PIDController pidController)
    {
        addRequirements(module);
        this.module = module;
        this.velocityPidSlot = velocityPidSlot;
        this.positionalPidSlot = -1;
        this.useTrapezoidalPositioning = false;
        useVelocityClosedLoop = true;
        usePositionalClosedLoop = false;
        this.pidController = pidController;
        state = null;
    }
    public NFRSwerveModuleSetState(NFRSwerveModule module, PIDController pidController)
    {
        addRequirements(module);
        this.module = module;
        this.velocityPidSlot = -1;
        this.positionalPidSlot = -1;
        this.useTrapezoidalPositioning = false;
        useVelocityClosedLoop = false;
        usePositionalClosedLoop = false;
        this.pidController = pidController;
        state = null;
    }
    public NFRSwerveModuleSetState(NFRSwerveModule module, int positionalPidSlot,
        boolean useTrapezoidalPositioning)
    {
        addRequirements(module);
        this.module = module;
        this.velocityPidSlot = -1;
        this.positionalPidSlot = positionalPidSlot;
        this.useTrapezoidalPositioning = useTrapezoidalPositioning;
        useVelocityClosedLoop = false;
        usePositionalClosedLoop = true;
        pidController = null;
        state = null;
    }
    public void setTargetState(SwerveModuleState state)
    {
        this.state = state;
        if (useVelocityClosedLoop)
        {
            module.setDriveSpeed(state.speedMetersPerSecond, velocityPidSlot);
        }
        else
        {
            module.setDriveSpeed(state.speedMetersPerSecond);
        }
        if (usePositionalClosedLoop)
        {
            module.setTurnPosition(state.angle, positionalPidSlot, useTrapezoidalPositioning);
        }
        else
        {
            pidController.reset();
            pidController.setSetpoint(state.angle.getRotations());
        }
    }
    @Override
    public void execute()
    {
        if (!usePositionalClosedLoop && state != null)
        {
            module.setTurnSpeed(pidController.calculate(module.getRotation().getRotations()));
        }
    }
}
