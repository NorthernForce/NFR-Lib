package org.northernforce.util;

import java.util.function.DoubleConsumer;

public class NFRConstantSpeedFeedback implements NFRFeedbackProvider
{
    protected final double speed;
    protected double setpoint, position;
    protected final DoubleConsumer action;
    public NFRConstantSpeedFeedback(double speed, DoubleConsumer action)
    {
        this.speed = speed;
        this.action = action;
    }
    @Override
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }
    @Override
    public void runFeedback(double position)
    {
        this.position = position;
        action.accept(speed);
    }
    @Override
    public boolean atSetpoint() {
        return position >= setpoint;
    }
}