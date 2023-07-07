package org.northernforce.util;

import org.northernforce.motors.NFRMotorController;

/**
 * This is a NFRFeedbackProvider that uses the integrated positional pid in motor controllers.
 */
public class NFRInternalPositionalPIDFeedback implements NFRFeedbackProvider
{
    protected final NFRMotorController motor;
    protected final int pidSlot;
    protected final boolean useTrapezoidalPositioning;
    protected final double tolerance;
    protected double setpoint;
    /**
     * Creates a new NFRInternalPositionalPIDFeedback.
     * @param motor the motor controller(s) to use.
     * @param pidSlot the pid slot of the motor controller.
     * @param useTrapezoidalPositioning whether to use advanced trapezoidal positioning.
     * @param tolerance the tolerance to check whether at the setpoint
     */
    public NFRInternalPositionalPIDFeedback(NFRMotorController motor, int pidSlot, boolean useTrapezoidalPositioning,
        double tolerance)
    {
        this.motor = motor;
        this.pidSlot = pidSlot;
        this.useTrapezoidalPositioning = useTrapezoidalPositioning;
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
        if (useTrapezoidalPositioning)
        {
            motor.setPositionTrapezoidal(pidSlot, setpoint);
        }
        else
        {
            motor.setPosition(pidSlot, setpoint);
        }
    }
    /**
     * Checks to see whether within tolerance.
     * @return |position - setpoint| less than tolerance
     */
    @Override
    public boolean atSetpoint()
    {
        return Math.abs(motor.getSelectedEncoder().getPosition() - setpoint) <= tolerance;
    }
}