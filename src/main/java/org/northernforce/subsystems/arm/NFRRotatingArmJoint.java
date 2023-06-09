package org.northernforce.subsystems.arm;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.northernforce.encoders.NFREncoder;
import org.northernforce.motors.NFRMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * A common subsystem for rotating arm joints.
 */
public class NFRRotatingArmJoint extends NFRArmJoint
{
    /**
     * The configuration for the arm. Can be built with builder functions.
     */
    public static class NFRRotatingArmJointConfiguration extends NFRArmJointConfiguration
    {
        protected Transform3d originOffset = new Transform3d();
        protected Rotation2d positiveLimit = null, negativeLimit = null, encoderOffset = new Rotation2d();
        protected int positionalPidSlot = 0;
        protected boolean useTrapezoidalPositioning = false, simulateGravity = false;
        protected DCMotor gearbox = null;
        protected double gearRatio = 1, length = 0, mass = 0;
        /**
         * Basic constructor that initializes values to defaults. Best to use with the build functionality.
         * @param name the unique name of the rotating arm joint.
         */
        public NFRRotatingArmJointConfiguration(String name)
        {
            super(name);
        }
        /**
         * All inclusive constructor that initializes all of the values.
         * @param name the unique name of the rotating arm joint.
         * @param originOffset the offset from the end of the previous joint, or from the center of the chassis.
         * @param positiveLimit the positive limit of the arm joint.
         * @param negativeLimit the negative limit of the arm joint.
         * @param positionalPidSlot the slot index of the position pid configuration
         * @param useTrapezoidalPositioning whether to use trapezoidal positioning
         * @param gearbox the gearbox of the motor. Only necessary if using simulation.
         * @param gearRatio the gear ratio of the motor. Only necessary if using simulation.
         * @param length the length of the arm in meters.
         * @param mass the mass of the arm in kilograms.
         * @param encoderOffset the offset from the encoder value to the arm position such that it is added to the encoder value
         * when getRotation() is called.
         * @param simulateGravity whether or not to simulate gravity.
         */
        public NFRRotatingArmJointConfiguration(String name, Transform3d originOffset, Rotation2d positiveLimit,
            Rotation2d negativeLimit, int positionalPidSlot, boolean useTrapezoidalPositioning, DCMotor gearbox,
            double gearRatio, double length, double mass, Rotation2d encoderOffset, boolean simulateGravity)
        {
            super(name);
            this.originOffset = originOffset;
            this.positiveLimit = positiveLimit;
            this.negativeLimit = negativeLimit;
            this.positionalPidSlot = positionalPidSlot;
            this.useTrapezoidalPositioning = useTrapezoidalPositioning;
            this.gearbox = gearbox;
            this.gearRatio = gearRatio;
            this.length = length;
            this.mass = mass;
            this.simulateGravity = simulateGravity;
        }
        /**
         * With origin offset.
         * @param originOffset the offset from the end of the previous joint, or from the center of the chassis.
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withOriginOffset(Transform3d originOffset)
        {
            this.originOffset = originOffset;
            return this;
        }
        /**
         * With limits.
         * @param positiveLimit the positive limit of the arm joint.
         * @param negativeLimit the negative limit of the arm joint.
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withLimits(Rotation2d positiveLimit, Rotation2d negativeLimit)
        {
            this.positiveLimit = positiveLimit;
            this.negativeLimit = negativeLimit;
            return this;
        }
        /**
         * With position pid slot.
         * @param positionalPidSlot the slot index of the position pid configuration
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withPositionPidSlot(int positionalPidSlot)
        {
            this.positionalPidSlot = positionalPidSlot;
            return this;
        }
        /**
         * With use trapezoidal positioning.
         * @param useTrapezoidalPositioning whether to use trapezoidal positioning
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withUseTrapezoidalPositioning(boolean useTrapezoidalPositioning)
        {
            this.useTrapezoidalPositioning = useTrapezoidalPositioning;
            return this;
        }
        /**
         * With gearbox.
         * @param gearbox the gearbox of the motor. Only necessary if using simulation.
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withGearbox(DCMotor gearbox)
        {
            this.gearbox = gearbox;
            return this;
        }
        /**
         * With gear ratio.
         * @param gearRatio the gear ratio of the motor. Only necessary if using simulation.
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withGearRatio(double gearRatio)
        {
            this.gearRatio = gearRatio;
            return this;
        }
        /**
         * With length.
         * @param length the length of the arm in meters.
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withLength(double length)
        {
            this.length = length;
            return this;
        }
        /**
         * With mass.
         * @param mass the mass of the arm in kilograms.
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withMass(double mass)
        {
            this.mass = mass;
            return this;
        }
        /**
         * With encoder offset.
         * @param encoderOffset the offset from the encoder value to the arm position such that it is added to the encoder value
         * when getRotation() is called.
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withEncoderOffset(Rotation2d encoderOffset)
        {
            this.encoderOffset = encoderOffset;
            return this;
        }
        /**
         * With simulate gravity.
         * @param simulateGravity whether or not to simulate gravity.
         * @return configuration instance
         */
        public NFRRotatingArmJointConfiguration withSimulateGravity(Boolean simulateGravity)
        {
            this.simulateGravity = simulateGravity;
            return this;
        }
    }
    protected final NFRRotatingArmJointConfiguration config;
    protected final NFRMotorController controller;
    protected final Optional<NFREncoder> externalEncoder;
    protected final Optional<PIDController> pidController;
    protected final SingleJointedArmSim armSim;
    /**
     * Creates a new NFRRotatingArmJoint
     * @param config the configuration of the nfr rotating arm joint
     * @param controller the motor controller for the rotating arm joint
     * @param externalEncoder the optional external encoder
     * @param pidController the optional pid controller if external encoder is present
     */
    public NFRRotatingArmJoint(NFRRotatingArmJointConfiguration config, NFRMotorController controller,
        Optional<NFREncoder> externalEncoder, Optional<PIDController> pidController)
    {
        super(config);
        this.config = config;
        this.controller = controller;
        this.externalEncoder = externalEncoder;
        this.pidController = pidController;
        if (externalEncoder.isEmpty())
        {
            if (config.positiveLimit != null && config.negativeLimit != null)
            {
                controller.setupLimits(
                    config.positiveLimit.minus(config.encoderOffset).getRotations(),
                    config.negativeLimit.minus(config.encoderOffset).getRotations()
                );
            }
        }
        if (RobotBase.isSimulation())
        {
            armSim = new SingleJointedArmSim(
                config.gearbox,
                config.gearRatio,
                SingleJointedArmSim.estimateMOI(config.length, config.mass),
                config.length,
                config.negativeLimit.getRadians(),
                config.positiveLimit.getRadians(),
                config.simulateGravity
            );
        }
        else
        {
            armSim = null;
        }
    }
    /**
     * Gets the rotation of the arm
     * @return the rotation of the arm
     */
    public Rotation2d getRotation()
    {
        if (externalEncoder.isPresent())
            return Rotation2d.fromDegrees(externalEncoder.get().getPosition())
                .plus(config.encoderOffset);
        else
            return Rotation2d.fromDegrees(controller.getSelectedEncoder().getPosition())
                .plus(config.encoderOffset);
    }
    /**
     * The default command for the NFRRotatingArmJoint.
     */
    public class DefaultCommand extends CommandBase
    {
        protected final DoubleSupplier supplier;
        /**
         * Creates a new DefaultCommand
         * @param supplier the supplier for the speed of the arm
         */
        public DefaultCommand(DoubleSupplier supplier)
        {
            addRequirements(NFRRotatingArmJoint.this);
            this.supplier = supplier;
        }
        /**
         * Executes and checks whether outside of limits
         */
        @Override
        public void execute()
        {
            double targetSpeed = supplier.getAsDouble();
            if (externalEncoder.isPresent() && config.negativeLimit != null &&
                getRotation().minus(config.negativeLimit).getDegrees() <= 0 &&
                targetSpeed < 0)
            {
                targetSpeed = 0;
            }
            else if (externalEncoder.isPresent() && config.positiveLimit != null &&
                getRotation().minus(config.positiveLimit).getDegrees() >= 0 &&
                targetSpeed > 0)
            {
                targetSpeed = 0;
            }
            controller.set(targetSpeed);
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
    /**
     * The set angle command for the NFRRotatingArmJoint.
     */
    public class SetAngle extends CommandBase
    {
        protected final Rotation2d targetAngle;
        /**
         * Creates a new set angle.
         * @param targetAngle the target angle for the arm to go to.
         */
        public SetAngle(Rotation2d targetAngle)
        {
            addRequirements(NFRRotatingArmJoint.this);
            this.targetAngle = targetAngle;
        }
        /**
         * Initializes the command which resets the pid controller or sets the internal closed-loop control.
         */
        @Override
        public void initialize()
        {
            if (pidController.isPresent())
            {
                pidController.get().reset();
                pidController.get().setSetpoint(targetAngle.getDegrees());
            }
            else
            {
                if (config.useTrapezoidalPositioning)
                {
                    controller.setPositionTrapezoidal(config.positionalPidSlot, targetAngle.minus(config.encoderOffset).getRotations());
                }
                else
                {
                    controller.setPosition(config.positionalPidSlot, targetAngle.minus(config.encoderOffset).getRotations());
                }
            }
        }
        /**
         * If using pid controller, computes closed loop control
         */
        @Override
        public void execute()
        {
            if (pidController.isPresent())
            {
                double targetSpeed = pidController.get().calculate(getRotation().getDegrees());
                if (externalEncoder.isPresent() && config.negativeLimit != null &&
                    getRotation().minus(config.negativeLimit).getDegrees() <= 0 &&
                    targetSpeed < 0)
                {
                    targetSpeed = 0;
                }
                else if (externalEncoder.isPresent() && config.positiveLimit != null &&
                    getRotation().minus(config.positiveLimit).getDegrees() >= 0 &&
                    targetSpeed > 0)
                {
                    targetSpeed = 0;
                }
                controller.set(targetSpeed);
            }
        }
        /**
         * Returns whether within 5 degrees of target angle
         */
        @Override
        public boolean isFinished()
        {
            return Math.abs(getRotation().minus(targetAngle).getDegrees()) <= 5;
        }
    }
    /**
     * Gets the set angle command
     * @param targetAngle target angle for arm to go to
     * @return the set angle command
     */
    public Command getSetAngleCommand(Rotation2d targetAngle)
    {
        return new SetAngle(targetAngle);
    }
    /**
     * Updates the simulation data.
     */
    @Override
    public void simulationPeriodic()
    {
        armSim.setInputVoltage(controller.getSimulationOutputVoltage());
        armSim.update(0.02);
        if (externalEncoder.isPresent())
        {
            externalEncoder.get().setSimulationPosition(
                Rotation2d.fromRadians(armSim.getAngleRads()).minus(config.encoderOffset).getRotations()
            );
            externalEncoder.get().setSimulationVelocity(
                Rotation2d.fromRadians(armSim.getVelocityRadPerSec()).minus(config.encoderOffset).getRotations()
            );
        }
        else
        {
            controller.getSelectedEncoder().setSimulationPosition(
                Rotation2d.fromRadians(armSim.getAngleRads()).minus(config.encoderOffset).getRotations()
            );
            controller.getSelectedEncoder().setSimulationVelocity(
                Rotation2d.fromRadians(armSim.getVelocityRadPerSec()).minus(config.encoderOffset).getRotations()
            );
        }
    }
    /**
     * Returns the end state between the last joint and the end of this joint.
     */
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
