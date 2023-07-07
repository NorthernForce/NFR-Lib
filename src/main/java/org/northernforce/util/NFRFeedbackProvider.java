package org.northernforce.util;

/**
 * This is the base interface for a feedback provider, meant to get a joint or similar thing to a set state. Can be used for
 * either positional or speed.
 */
public interface NFRFeedbackProvider
{
    /**
     * Sets the setpoint of the feedback provider. Should only be called when starting the feedback routine.
     * @param setpoint the target state of the mechanism
     */
    public void setSetpoint(double setpoint);
    /**
     * Updates the feedback. Will send signals to the 
     * @param currentPosition
     */
    public default void runFeedback(double currentPosition)
    {
    }
    public boolean atSetpoint();
}
