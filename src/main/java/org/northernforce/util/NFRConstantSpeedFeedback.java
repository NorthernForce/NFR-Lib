package org.northernforce.util;

import java.util.function.DoubleConsumer;

/**
 * Creates a new constant speed feedback. Runs a constant speed until at setpoint.
 */
public class NFRConstantSpeedFeedback implements NFRFeedbackProvider
{
    protected final double speed;
    protected double setpoint, position;
    protected final DoubleConsumer action;
    protected final boolean moreThan;
    /**
     * Creates a new NFRConstantSpeedFeedback.
     * @param speed the constant speed to go at.
     * @param action the consumer that is provided with the speed.
     * @param moreThan whether it is to be terminated when position is more than or less than setpoint
     */
    public NFRConstantSpeedFeedback(double speed, DoubleConsumer action, boolean moreThan)
    {
        this.speed = speed;
        this.action = action;
        this.moreThan = moreThan;
    }
    /**
     * Sets the setpoint
     * @param setpoint the target position
     */
    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
    /**
     * Records the current position to check if at setpoint.
     * @param position current position
     */
    @Override
    public void runFeedback(double position)
    {
        this.position = position;
        action.accept(speed);
    }
    /**
     * Checks whether at setpoint.
     * @return moreThan ? position is more than setpoint : position is less than setpoint
     */
    @Override
    public boolean atSetpoint() {
        return moreThan ? position >= setpoint : position <= setpoint;
    }
}