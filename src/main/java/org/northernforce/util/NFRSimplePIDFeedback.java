package org.northernforce.util;

import java.util.function.DoubleConsumer;

import edu.wpi.first.math.controller.PIDController;

/**
 * A simple pid controller feedback implementation of NFRFeedbackProvider.
 */
public class NFRSimplePIDFeedback implements NFRFeedbackProvider
{
    private final DoubleConsumer action;
    private final PIDController controller;
    /**
     * Creates a new NFRSimplePIDFeedback.
     * @param action the consumer for the output of the pid controller.
     * @param controller the pid controller to be used for feedback
     */
    public NFRSimplePIDFeedback(DoubleConsumer action, PIDController controller)
    {
        this.action = action;
        this.controller = controller;
    }
    /**
     * Sets the pid setpoint, and resets the controller.
     * @param setpoint relative to the pidController
     */
    @Override
    public void setSetpoint(double setpoint)
    {
        controller.reset();
        controller.setSetpoint(setpoint);
    }
    /**
     * Runs the feedback.
     * @param currentPosition calculates the feedback and feeds it to the supplier.
     */
    @Override
    public void runFeedback(double currentPosition)
    {
        action.accept(controller.calculate(currentPosition));
    }
    /**
     * Returns whether the controller is at setpoint.
     * @return controller.atSetpoint()
     */
    @Override
    public boolean atSetpoint()
    {
        return controller.atSetpoint();
    }
}
