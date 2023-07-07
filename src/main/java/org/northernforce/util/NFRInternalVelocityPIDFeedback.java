package org.northernforce.util;

import org.northernforce.motors.NFRMotorController;

/**
 * This is a NFRFeedbackProvider that uses the integrated velocity pid in motor controllers.
 */
public class NFRInternalVelocityPIDFeedback implements NFRFeedbackProvider
{
    protected final NFRMotorController motor;
    protected final int pidSlot;
    protected final double tolerance;
    protected double setpoint;
    /**
     * Creates a new NFRInternalVelocityPIDFeedback.
     * @param motor the motor controller(s) to use.
     * @param pidSlot the pid slot of the motor controller.
     * @param tolerance the tolerance to check whether at the setpoint
     */
    public NFRInternalVelocityPIDFeedback(NFRMotorController motor, int pidSlot, double tolerance)
    {
        this.motor = motor;
        this.pidSlot = pidSlot;
        this.tolerance = tolerance;
    }
    /**
     * Sets the setpoint of the motor.
     * @param setpoint relative to the selected encoder
     */
    @Override
    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint;
        motor.setVelocity(pidSlot, setpoint);
    }
    /**
     * Checks to see whether within tolerance.
     * @return |velocity - setpoint| <= tolerance
     */
    @Override
    public boolean atSetpoint()
    {
        return Math.abs(motor.getSelectedEncoder().getVelocity() - setpoint) <= tolerance;
    }
}