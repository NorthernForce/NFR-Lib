package org.northernforce.commands;

import java.util.function.DoubleSupplier;

import org.northernforce.subsystems.arm.NFRArmMotorExtensionJoint;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class NFRArmMotorExtensionJointWithJoystick extends CommandBase
{
    protected final NFRArmMotorExtensionJoint arm;
    protected final DoubleSupplier supplier;
    protected final boolean useClosedLoop;
    protected final int pidSlot;
    public NFRArmMotorExtensionJointWithJoystick(NFRArmMotorExtensionJoint arm, DoubleSupplier supplier)
    {
        addRequirements(arm);
        this.arm = arm;
        this.supplier = supplier;
        this.useClosedLoop = false;
        this.pidSlot = -1;
    }
    public NFRArmMotorExtensionJointWithJoystick(NFRArmMotorExtensionJoint arm, DoubleSupplier supplier, int pidSlot)
    {
        addRequirements(arm);
        this.arm = arm;
        this.supplier = supplier;
        this.useClosedLoop = true;
        this.pidSlot = pidSlot;
    }
    @Override
    public void execute()
    {
        if (useClosedLoop)
        {
            arm.setClosedLoop(supplier.getAsDouble(), pidSlot);
        }
        else
        {
            arm.setOpenLoop(supplier.getAsDouble());
        }
    }
}
