package org.northernforce.subsystems.arm;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.northernforce.encoders.NFREncoder;
import org.northernforce.motors.NFRMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

public class NFRSimpleMotorClaw extends NFRArmJoint
{
    public static class NFRSimpleMotorClawConfiguration extends NFRArmJointConfiguration
    {
        protected Transform3d originOffset, offsetToEndOfClaw;
        protected boolean useLimitSwitches, usePidControl, useTrapezoidalPidControl;
        protected Rotation2d openRotation, closedRotation;
        protected int positionalPidSlot;
        protected double openSpeed, closeSpeed;
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
        public NFRSimpleMotorClawConfiguration withOriginOffset(Transform3d originOffset)
        {
            this.originOffset = originOffset;
            return this;
        }
        public NFRSimpleMotorClawConfiguration withOffsetToEndOfClaw(Transform3d offsetToEndOfClaw)
        {
            this.offsetToEndOfClaw = offsetToEndOfClaw;
            return this;
        }
        public NFRSimpleMotorClawConfiguration withUseLimitSwitches(boolean useLimitSwitches)
        {
            this.useLimitSwitches = useLimitSwitches;
            return this;
        }
        public NFRSimpleMotorClawConfiguration withUsePidControl(boolean usePidControl)
        {
            this.usePidControl = usePidControl;
            return this;
        }
        public NFRSimpleMotorClawConfiguration withTrapezoidalPidControl(boolean useTrapezoidalPidControl)
        {
            this.useTrapezoidalPidControl = useTrapezoidalPidControl;
            return this;
        }
        public NFRSimpleMotorClawConfiguration withOpenRotation(Rotation2d openRotation)
        {
            this.openRotation = openRotation;
            return this;
        }
        public NFRSimpleMotorClawConfiguration withClosedRotation(Rotation2d closedRotation)
        {
            this.closedRotation = closedRotation;
            return this;
        }
        public NFRSimpleMotorClawConfiguration withPositionalPidSlot(int positionalPidSlot)
        {
            this.positionalPidSlot = positionalPidSlot;
            return this;
        }
        public NFRSimpleMotorClawConfiguration withOpenSpeed(double openSpeed)
        {
            this.openSpeed = openSpeed;
            return this;
        }
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
    }
    protected class OpenCommand extends CommandBase
    {
        public OpenCommand()
        {
            addRequirements(NFRSimpleMotorClaw.this);
        }
        @Override
        public void initialize()
        {
            if (config.usePidControl && externalEncoder.isEmpty() && pidController.isEmpty())
            {
                if (config.useTrapezoidalPidControl)
                {
                    controller.setPositionTrapezoidal(config.positionalPidSlot, config.openRotation.getRotations());
                }
                else
                {
                    controller.setPosition(config.positionalPidSlot, config.openRotation.getRotations());
                }
            }
            else if (pidController.isPresent())
            {
                pidController.get().reset();
                pidController.get().setSetpoint(config.openRotation.getRotations());
            }
        }
        @Override
        public void execute()
        {
            if (config.usePidControl && pidController.isPresent())
            {
                if (externalEncoder.isPresent())
                {
                    controller.set(pidController.get().calculate(externalEncoder.get().getPosition()));
                }
                else
                {
                    controller.set(pidController.get().calculate(controller.getSelectedEncoder().getPosition()));
                }
            }
            else
            {
                controller.set(config.openSpeed);
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
                if (externalEncoder.isPresent())
                {
                    return externalEncoder.get().getPosition() > config.openRotation.getRotations();
                }
                else
                {
                    return controller.getSelectedEncoder().getPosition() > config.openRotation.getRotations();
                }
            }
        }
        @Override
        public void end(boolean interrupted)
        {
            controller.set(0);
        }
    }
    public Command getOpenCommand()
    {
        return new OpenCommand();
    }
    protected class CloseCommand extends CommandBase
    {
        public CloseCommand()
        {
            addRequirements(NFRSimpleMotorClaw.this);
        }
        @Override
        public void initialize()
        {
            if (config.usePidControl && externalEncoder.isEmpty() && pidController.isEmpty())
            {
                if (config.useTrapezoidalPidControl)
                {
                    controller.setPositionTrapezoidal(config.positionalPidSlot, config.closedRotation.getRotations());
                }
                else
                {
                    controller.setPosition(config.positionalPidSlot, config.closedRotation.getRotations());
                }
            }
            else if (pidController.isPresent())
            {
                pidController.get().reset();
                pidController.get().setSetpoint(config.closedRotation.getRotations());
            }
        }
        @Override
        public void execute()
        {
            if (config.usePidControl && pidController.isPresent())
            {
                if (externalEncoder.isPresent())
                {
                    controller.set(pidController.get().calculate(externalEncoder.get().getPosition()));
                }
                else
                {
                    controller.set(pidController.get().calculate(controller.getSelectedEncoder().getPosition()));
                }
            }
            else
            {
                controller.set(config.closeSpeed);
            }
        }
        @Override
        public boolean isFinished()
        {
            if (config.useLimitSwitches)
            {
                return closedLimitSwitch.get().getAsBoolean();
            }
            else
            {
                if (externalEncoder.isPresent())
                {
                    return externalEncoder.get().getPosition() > config.closedRotation.getRotations();
                }
                else
                {
                    return controller.getSelectedEncoder().getPosition() > config.closedRotation.getRotations();
                }
            }
        }
        @Override
        public void end(boolean interrupted)
        {
            controller.set(0);
        }
    }
    public Command getCloseCommand()
    {
        return new CloseCommand();
    }
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
    @Override
    public Transform3d getEndState()
    {
        return config.originOffset.plus(config.offsetToEndOfClaw);
    }
}