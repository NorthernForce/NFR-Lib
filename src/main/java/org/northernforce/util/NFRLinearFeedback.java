package org.northernforce.util;

import java.util.function.DoubleConsumer;

/**
 * Simple linear feedback that multiplies the setpoint by a factor, and then applies that setpoint.
 */
public class NFRLinearFeedback implements NFRFeedbackProvider
{
    private final double factor;
    private final DoubleConsumer action;
    private double setpoint;
    /**
     * Creates a new NFRLinearFeedback.
     * @param factor the factor to multiply the setpoint by.
     * @param action the consumer for the value (ie motor.set)
     */
    public NFRLinearFeedback(double factor, DoubleConsumer action)
    {
        this.factor = factor;
        this.action = action;
    }
    /**
     * Sets the setpoint.
     * @param setpoint to be multiplied by factor
     */
    @Override
    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint * factor;
    }
    /**
     * Runs the feedback.
     * @param currentPosition ignored
     */
    @Override
    public void runFeedback(double currentPosition)
    {
        action.accept(setpoint);
    }
    /**
     * Just returns false as this is mainly meant for joystick control.
     * @return false
     */
    @Override
    public boolean atSetpoint()
    {
        return false;
    }
}