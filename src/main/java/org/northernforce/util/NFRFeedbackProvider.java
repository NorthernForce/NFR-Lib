package org.northernforce.util;

public interface NFRFeedbackProvider
{
    public void setSetpoint(double setpoint);
    public default void runFeedback(double currentPosition)
    {
    }
    public boolean atSetpoint();
}
