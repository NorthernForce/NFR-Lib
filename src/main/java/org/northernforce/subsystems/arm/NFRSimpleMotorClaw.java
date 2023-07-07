package org.northernforce.subsystems.arm;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.northernforce.encoders.NFREncoder;
import org.northernforce.motors.NFRMotorController;
import org.northernforce.util.NFRFeedbackProvider;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This is a simple motor claw joint that is intented for a claw that is controlled by a motor, in which one direction opens,
 * and the other closes.
 */
public class NFRSimpleMotorClaw extends NFRArmJoint
{
    /**
     * This is the configuration for a simple motor claw joint.
     */
    public static class NFRSimpleMotorClawConfiguration extends NFRArmJointConfiguration
    {
        protected Transform3d originOffset, offsetToEndOfClaw;
        protected boolean useLimitSwitches, usePidControl, useTrapezoidalPidControl;
        protected Rotation2d openRotation, closedRotation;
        protected int positionalPidSlot;
        protected double openSpeed, closeSpeed;
        /**
         * Simple constructor that initializes values to default values (ie. 0 for double).
         * @param name the unique name of the motor claw.
         */
        public NFRSimpleMotorClawConfiguration(String name)
        {
            super(name);
            originOffset = new Transform3d();
            offsetToEndOfClaw = new Transform3d();
            useLimitSwitches = false;
            usePidControl = false;
            useTrapezoidalPidControl = false;
            openRotation = new Rotation2d();
            closedRotation = new Rotation2d();
            positionalPidSlot = 0;
            openSpeed = 0;
            closeSpeed = 0;
        }
        /**
         * Constructs the configuration. Takes in all of the possible parameters.
         * @param name the unique name of the motor claw.
         * @param originOffset the offset from the end of the previous joint to the start of this joint.
         * @param offsetToEndOfClaw the offset from the start of the claw to the end of the claw/focal point.
         * @param useLimitSwitches whether to use limit switches to prevent the motor from going too far, or whether
         * to use the encoder.
         * @param usePidControl whether to use pid control to open/close or to simple drive until at a limit. Needs encoder
         * @param useTrapezoidalPidControl whether to use trapezoidal pid control to open/close or to simple drive until at a
         * limit. Needs encoder
         * @param openRotation the open encoder rotation if the claw is open. 
         * @param closedRotation the closed encoder rotation if the claw is closed.
         * @param positionalPidSlot the slot of the internal pid loop, if any.
         * @param openSpeed the speed to open if using consistent speed instead of pid.
         * @param closeSpeed the speed to close if using consistent speed instead of pid.
         */
        public NFRSimpleMotorClawConfiguration(String name, Transform3d originOffset,
            Transform3d offsetToEndOfClaw, boolean useLimitSwitches, boolean usePidControl, boolean useTrapezoidalPidControl,
            Rotation2d openRotation, Rotation2d closedRotation, int positionalPidSlot, double openSpeed, double closeSpeed)
        {
            super(name);
            this.originOffset = originOffset;
            this.offsetToEndOfClaw = offsetToEndOfClaw;
            this.useLimitSwitches = useLimitSwitches;
            this.usePidControl = usePidControl;
            this.useTrapezoidalPidControl = useTrapezoidalPidControl;
            this.openRotation = openRotation;
            this.closedRotation = closedRotation;
            this.positionalPidSlot = positionalPidSlot;
            this.openSpeed = openSpeed;
            this.closeSpeed = closeSpeed;
        }
        /**
         * With origin offset
         * @param originOffset the offset from the end of the previous joint to the start of this joint.
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withOriginOffset(Transform3d originOffset)
        {
            this.originOffset = originOffset;
            return this;
        }
        /**
         * With offset to end of claw
         * @param offsetToEndOfClaw the offset from the start of the claw to the end of the claw/focal point.
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withOffsetToEndOfClaw(Transform3d offsetToEndOfClaw)
        {
            this.offsetToEndOfClaw = offsetToEndOfClaw;
            return this;
        }
        /**
         * With use limit switches
         * @param useLimitSwitches whether to use limit switches to prevent the motor from going too far, or whether
         * to use the encoder.
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withUseLimitSwitches(boolean useLimitSwitches)
        {
            this.useLimitSwitches = useLimitSwitches;
            return this;
        }
        /**
         * With use pid control
         * @param usePidControl whether to use pid control to open/close or to simple drive until at a limit. Needs encoder
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withUsePidControl(boolean usePidControl)
        {
            this.usePidControl = usePidControl;
            return this;
        }
        /**
         * With use trapezoidal pid control
         * @param useTrapezoidalPidControl whether to use trapezoidal pid control to open/close or to simple drive until at a
         * limit. Needs encoder
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withTrapezoidalPidControl(boolean useTrapezoidalPidControl)
        {
            this.useTrapezoidalPidControl = useTrapezoidalPidControl;
            return this;
        }
        /**
         * With open rotation
         * @param openRotation the open encoder rotation if the claw is open. 
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withOpenRotation(Rotation2d openRotation)
        {
            this.openRotation = openRotation;
            return this;
        }
        /**
         * With closed rotation
         * @param closedRotation the closed encoder rotation if the claw is closed.
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withClosedRotation(Rotation2d closedRotation)
        {
            this.closedRotation = closedRotation;
            return this;
        }
        /**
         * With positional pid slot
         * @param positionalPidSlot the slot of the internal pid loop, if any.
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withPositionalPidSlot(int positionalPidSlot)
        {
            this.positionalPidSlot = positionalPidSlot;
            return this;
        }
        /**
         * With open speed
         * @param openSpeed the speed to open if using consistent speed instead of pid.
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withOpenSpeed(double openSpeed)
        {
            this.openSpeed = openSpeed;
            return this;
        }
        /**
         * With close speed
         * @param closeSpeed the speed to close if using consistent speed instead of pid.
         * @return this
         */
        public NFRSimpleMotorClawConfiguration withCloseSpeed(double closeSpeed)
        {
            this.closeSpeed = closeSpeed;
            return this;
        }
    }
    protected final NFRSimpleMotorClawConfiguration config;
    protected final NFRMotorController controller;
    protected final Optional<PIDController> pidController;
    protected final Optional<BooleanSupplier> closedLimitSwitch, openLimitSwitch;
    protected final Optional<NFREncoder> externalEncoder;
    protected NFRFeedbackProvider feedback;
    /**
     * Creates a new NFR Simple Motor Claw
     * @param config the configuration of the claw
     * @param controller the motor controller to control the claw
     * @param externalEncoder the optional external encoder if encoder cannot be directly linked to motor
     * @param closedLimitSwitch the closed limit switch, optional
     * @param openLimitSwitch the open limit switch, optional
     * @param pidController the pid controller, if not using internal
     */
    public NFRSimpleMotorClaw(NFRSimpleMotorClawConfiguration config, NFRMotorController controller,
        Optional<NFREncoder> externalEncoder, Optional<BooleanSupplier> closedLimitSwitch,
        Optional<BooleanSupplier> openLimitSwitch, Optional<PIDController> pidController)
    {
        super(config);
        this.config = config;
        this.controller = controller;
        this.externalEncoder = externalEncoder;
        this.closedLimitSwitch = closedLimitSwitch;
        this.openLimitSwitch = openLimitSwitch;
        this.pidController = pidController;
        this.feedback = null;
    }
    public void setSpeed(double speed)
    {
        if (feedback != null)
        {
            if (!externalEncoder.isPresent() || (speed > 0 && (config.useLimitSwitches ? openLimitSwitch.get().getAsBoolean() :
                externalEncoder.get().getPosition() < config.openRotation.getRotations())) || (speed < 0 &&
                (config.useLimitSwitches ? closedLimitSwitch.get().getAsBoolean() :
                externalEncoder.get().getPosition() > config.closedRotation.getRotations())))
            {
                feedback.setSetpoint(speed);
            }
            else
            {
                feedback.setSetpoint(0);
            }
        }
        else
        {
            controller.set(speed);
        }
    }
    public void setSpeed(NFRFeedbackProvider feedback, double speed)
    {
        this.feedback = feedback;
        if (!externalEncoder.isPresent() || (speed > 0 && (config.useLimitSwitches ? openLimitSwitch.get().getAsBoolean() :
            externalEncoder.get().getPosition() < config.openRotation.getRotations())) || (speed < 0 &&
            (config.useLimitSwitches ? closedLimitSwitch.get().getAsBoolean() :
            externalEncoder.get().getPosition() > config.closedRotation.getRotations())))
        {
            feedback.setSetpoint(speed);
        }
        else
        {
            feedback.setSetpoint(0);
        }
    }
    /**
     * The command to open the claw
     */
    protected class OpenCommand extends CommandBase
    {
        protected final NFRFeedbackProvider feedback;
        public OpenCommand(NFRFeedbackProvider feedback)
        {
            addRequirements(NFRSimpleMotorClaw.this);
            this.feedback = feedback;
        }
        @Override
        public void initialize()
        {
            feedback.setSetpoint(config.openRotation.getRotations());
        }
        @Override
        public void execute()
        {
            if (config.useLimitSwitches)
            {
                feedback.runFeedback(0);
            }
            else if (externalEncoder.isPresent())
            {
                feedback.runFeedback(externalEncoder.get().getPosition());
            }
            else
            {
                feedback.runFeedback(controller.getSelectedEncoder().getPosition());
            }
        }
        @Override
        public boolean isFinished()
        {
            if (config.useLimitSwitches)
            {
                return openLimitSwitch.get().getAsBoolean();
            }
            else
            {
                return feedback.atSetpoint();
            }
        }
        @Override
        public void end(boolean interrupted)
        {
            setSpeed(0);
        }
    }
    /**
     * Returns the command to open the claw. Either goes directly to position or uses pid depending on config.
     * @return the command to open the claw
     */
    public Command getOpenCommand(NFRFeedbackProvider feedback)
    {
        return new OpenCommand(feedback);
    }
    /**
     * The command to close the claw
     */
    protected class CloseCommand extends CommandBase
    {
        protected final NFRFeedbackProvider feedback;
        public CloseCommand(NFRFeedbackProvider feedback)
        {
            this.feedback = feedback;
            addRequirements(NFRSimpleMotorClaw.this);
        }
        @Override
        public void initialize()
        {
            feedback.setSetpoint(config.closedRotation.getRotations());
        }
        @Override
        public void execute()
        {
            if (config.useLimitSwitches)
            {
                feedback.runFeedback(0);
            }
            else if (externalEncoder.isPresent())
            {
                feedback.runFeedback(externalEncoder.get().getPosition());
            }
            else
            {
                feedback.runFeedback(controller.getSelectedEncoder().getPosition());
            }
        }
        @Override
        public boolean isFinished()
        {
            if (config.useLimitSwitches)
            {
                return openLimitSwitch.get().getAsBoolean();
            }
            else
            {
                return feedback.atSetpoint();
            }
        }
        @Override
        public void end(boolean interrupted)
        {
            setSpeed(0);
        }
    }
    /**
     * Returns the command to close the claw. Either goes directly to position or uses pid depending on config.
     * @return the command to close the claw
     */
    public Command getCloseCommand(NFRFeedbackProvider feedback)
    {
        return new CloseCommand(feedback);
    }
    /**
     * Checks whether the claw is closed
     * @return whether the claw is closed
     */
    public boolean isClosed()
    {
        if (config.useLimitSwitches)
        {
            return closedLimitSwitch.get().getAsBoolean();
        }
        else
        {
            if (externalEncoder.isEmpty())
            {
                return controller.getSelectedEncoder().getPosition() <= config.closedRotation.getRotations();
            }
            else
            {
                return externalEncoder.get().getPosition() <= config.closedRotation.getRotations();
            }
        }
    }
    /**
     * Checks whether the claw is open
     * @return whether the claw is open
     */
    public boolean isOpen()
    {
        if (config.useLimitSwitches)
        {
            return openLimitSwitch.get().getAsBoolean();
        }
        else
        {
            if (externalEncoder.isEmpty())
            {
                return controller.getSelectedEncoder().getPosition() >= config.openRotation.getRotations();
            }
            else
            {
                return externalEncoder.get().getPosition() >= config.openRotation.getRotations();
            }
        }
    }
    /**
     * Gets a manual control command that allows someone to manually set the claw to open/closed
     * @param closeSupplier a binary set close
     * @param openSupplier a binary set open
     * @return the manual control command
     */
    public Command getManualControlCommand(BooleanSupplier closeSupplier, BooleanSupplier openSupplier)
    {
        return Commands.run(() -> {
            if (closeSupplier.getAsBoolean() && !isClosed())
            {
                controller.set(config.closeSpeed);
            }
            else if (openSupplier.getAsBoolean() && !isOpen())
            {
                controller.set(config.openSpeed);
            }
            else
            {
                controller.set(0);
            }
        }, this);
    }
    /**
     * Gets a manual control command that allows someone to manually set the claw to open/closed
     * @param closeSupplier a double set close
     * @param openSupplier a double set open
     * @return the manual control command
     */
    public Command getManualControlCommand(DoubleSupplier closeSupplier, DoubleSupplier openSupplier)
    {
        return Commands.run(() -> {
            double speed = closeSupplier.getAsDouble() * config.closeSpeed + openSupplier.getAsDouble() * config.openSpeed;
            if (speed > 0 == config.openSpeed > 0 && !isOpen())
            {
                controller.set(speed);
            }
            else if (speed > 0 == config.closeSpeed > 0 && !isClosed())
            {
                controller.set(speed);
            }
            else
            {
                controller.set(0);
            }
        }, this);
    }
    /**
     * Gets the end state of the claw (static)
     * @return the end state of the claw
     */
    @Override
    public Transform3d getEndState()
    {
        return config.originOffset.plus(config.offsetToEndOfClaw);
    }
    public Rotation2d getSpeed()
    {
        return Rotation2d.fromRotations(externalEncoder.isPresent() ? externalEncoder.get().getVelocity() :
            controller.getSelectedEncoder().getVelocity());
    }
    @Override
    public void periodic()
    {
        if (DriverStation.isEnabled())
        {
            if (feedback != null)
            {
                feedback.runFeedback(getSpeed().getRotations());
            }
        }
    }
}