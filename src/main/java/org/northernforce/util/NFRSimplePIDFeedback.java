package org.northernforce.util;

import java.util.function.DoubleConsumer;

import edu.wpi.first.math.controller.PIDController;

public class NFRSimplePIDFeedback implements NFRFeedbackProvider
{
    private final DoubleConsumer action;
    private final PIDController controller;
    public NFRSimplePIDFeedback(DoubleConsumer action, PIDController controller)
    {
        this.action = action;
        this.controller = controller;
    }
    @Override
    public void setSetpoint(double setpoint)
    {
        controller.setSetpoint(setpoint);
    }
    @Override
    public void runFeedback(double currentPosition)
    {
        action.accept(controller.calculate(currentPosition));
    }
    @Override
    public boolean atSetpoint()
    {
        return controller.atSetpoint();
    }
}
