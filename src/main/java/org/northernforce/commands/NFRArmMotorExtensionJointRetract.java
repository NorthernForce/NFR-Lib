package org.northernforce.commands;

import org.northernforce.subsystems.arm.NFRArmMotorExtensionJoint;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class NFRArmMotorExtensionJointRetract extends CommandBase
{
    protected final NFRArmMotorExtensionJoint arm;
    protected final double speed;
    protected final boolean useClosedLoop;
    protected final int pidSlot;
    public NFRArmMotorExtensionJointRetract(NFRArmMotorExtensionJoint arm, double speed)
    {
        addRequirements(arm);
        this.arm = arm;
        this.speed = speed;
        this.useClosedLoop = false;
        this.pidSlot = -1;
    }
    public NFRArmMotorExtensionJointRetract(NFRArmMotorExtensionJoint arm, double speed, int pidSlot)
    {
        addRequirements(arm);
        this.arm = arm;
        this.speed = speed;
        this.useClosedLoop = true;
        this.pidSlot = pidSlot;
    }
    @Override
    public void initialize()
    {
        if (useClosedLoop)
        {
            arm.setClosedLoop(speed, pidSlot);
        }
    }
    @Override
    public void execute()
    {
        if (!useClosedLoop)
        {
            arm.setOpenLoop(speed);
        }
    }
    @Override
    public boolean isFinished()
    {
        return arm.isRetracted();
    }
    @Override
    public void end(boolean interrupted)
    {
        arm.setOpenLoop(0);
    }
}