package org.northernforce.util;

import org.northernforce.motors.NFRMotorController;

public class NFRInternalVelocityPIDFeedback implements NFRFeedbackProvider
{
    protected final NFRMotorController motor;
    protected final int pidSlot;
    protected final double tolerance;
    protected double setpoint;
    public NFRInternalVelocityPIDFeedback(NFRMotorController motor, int pidSlot, double tolerance)
    {
        this.motor = motor;
        this.pidSlot = pidSlot;
        this.tolerance = tolerance;
    }
    @Override
    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint;
        motor.setVelocity(pidSlot, setpoint);
    }
    @Override
    public boolean atSetpoint()
    {
        return Math.abs(motor.getSelectedEncoder().getVelocity() - setpoint) < tolerance;
    }
}