package org.northernforce.util;

import java.util.function.DoubleConsumer;

public class NFRLinearFeedback implements NFRFeedbackProvider
{
    private final double factor;
    private final DoubleConsumer action;
    private double setpoint;
    public NFRLinearFeedback(double factor, DoubleConsumer action)
    {
        this.factor = factor;
        this.action = action;
    }
    @Override
    public void setSetpoint(double setpoint)
    {
        this.setpoint = setpoint * factor;
    }
    @Override
    public void runFeedback(double currentPosition)
    {
        action.accept(setpoint);
    }
    @Override
    public boolean atSetpoint()
    {
        return false;
    }
}