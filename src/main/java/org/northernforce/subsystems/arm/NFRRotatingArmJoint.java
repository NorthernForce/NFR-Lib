package org.northernforce.subsystems.arm;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.northernforce.encoders.NFREncoder;
import org.northernforce.motors.NFRMotorController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NFRRotatingArmJoint extends NFRArmJoint
{
    public static class NFRRotatingArmJointConfiguration extends NFRArmJointConfiguration
    {
        protected Transform3d originOffset = new Transform3d();
        protected Rotation2d positiveLimit = null, negativeLimit = null;
        public NFRRotatingArmJointConfiguration(String name)
        {
            super(name);
        }
        public NFRRotatingArmJointConfiguration(String name, Transform3d originOffset, Rotation2d positiveLimit,
            Rotation2d negativeLimit)
        {
            super(name);
            this.originOffset = originOffset;
            this.positiveLimit = positiveLimit;
            this.negativeLimit = negativeLimit;
        }
        public NFRRotatingArmJointConfiguration withOriginOffset(Transform3d originOffset)
        {
            this.originOffset = originOffset;
            return this;
        }
        public NFRRotatingArmJointConfiguration withLimits(Rotation2d positiveLimit, Rotation2d negativeLimit)
        {
            this.positiveLimit = positiveLimit;
            this.negativeLimit = negativeLimit;
            return this;
        }
    }
    protected final NFRRotatingArmJointConfiguration config;
    protected final NFRMotorController controller;
    protected final Optional<NFREncoder> externalEncoder;
    public NFRRotatingArmJoint(NFRRotatingArmJointConfiguration config, NFRMotorController controller,
        Optional<NFREncoder> externalEncoder)
    {
        super(config);
        this.config = config;
        this.controller = controller;
        this.externalEncoder = externalEncoder;
        if (externalEncoder.isEmpty())
        {
            if (config.positiveLimit != null && config.negativeLimit != null)
            {
                controller.setupLimits(config.positiveLimit.getRotations(), config.negativeLimit.getRotations());
            }
        }
    }
    public Rotation2d getRotation()
    {
        if (externalEncoder.isPresent())
            return Rotation2d.fromDegrees(externalEncoder.get().getPosition());
        else
            return Rotation2d.fromDegrees(controller.getSelectedEncoder().getPosition());
    }
    public class DefaultCommand extends CommandBase
    {
        protected final DoubleSupplier supplier;
        public DefaultCommand(DoubleSupplier supplier)
        {
            addRequirements(NFRRotatingArmJoint.this);
            this.supplier = supplier;
        }
        @Override
        public void execute()
        {
            controller.set(supplier.getAsDouble());
        }
    }
    /**
     * Gets the default rotating arm command
     * @param supplier a supplier for the arm movement (ie joystick input)
     * @return the default rotating arm command
     */
    public Command getDefaultRotatingArmCommand(DoubleSupplier supplier)
    {
        return new DefaultCommand(supplier);
    }
    @Override
    public Transform3d getEndState() {
        return config.originOffset.plus(new Transform3d(
            new Translation3d(),
            new Rotation3d(
                0,
                getRotation().getRadians(),
                0
            )
        ));
    }
}
