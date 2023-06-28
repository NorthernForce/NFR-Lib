package org.northernforce.util;

import org.northernforce.motors.NFRMotorController;

public class NFRInternalPositionalPIDFeedback implements NFRFeedbackProvider
{
    protected final NFRMotorController motor;
    protected final int pidSlot;
    protected final boolean useTrapezoidalPositioning;
    protected final double tolerance;
    protected double setpoint;
    public NFRInternalPositionalPIDFeedback(NFRMotorController motor, int pidSlot, boolean useTrapezoidalPositioning,
        double tolerance)
    {
        this.motor = motor;
        this.pidSlot = pidSlot;
        this.useTrapezoidalPositioning = useTrapezoidalPositioning;
        this.tolerance = tolerance;
    }
    @Override
    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint;
        if (useTrapezoidalPositioning)
        {
            motor.setPositionTrapezoidal(pidSlot, setpoint);
        }
        else
        {
            motor.setPosition(pidSlot, setpoint);
        }
    }
    @Override
    public boolean atSetpoint()
    {
        return Math.abs(motor.getSelectedEncoder().getPosition() - setpoint) < tolerance;
    }
}