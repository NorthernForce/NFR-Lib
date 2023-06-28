package org.northernforce.subsystems.arm;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.northernforce.encoders.NFREncoder;
import org.northernforce.motors.NFRMotorController;
import org.northernforce.util.NFRFeedbackProvider;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
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
        protected boolean simulateGravity = false;
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
         * @param gearbox the gearbox of the motor. Only necessary if using simulation.
         * @param gearRatio the gear ratio of the motor. Only necessary if using simulation.
         * @param length the length of the arm in meters.
         * @param mass the mass of the arm in kilograms.
         * @param encoderOffset the offset from the encoder value to the arm position such that it is added to the encoder value
         * when getRotation() is called.
         * @param simulateGravity whether or not to simulate gravity.
         */
        public NFRRotatingArmJointConfiguration(String name, Transform3d originOffset, Rotation2d positiveLimit,
            Rotation2d negativeLimit, DCMotor gearbox, double gearRatio, double length, double mass, Rotation2d encoderOffset,
            boolean simulateGravity)
        {
            super(name);
            this.originOffset = originOffset;
            this.positiveLimit = positiveLimit;
            this.negativeLimit = negativeLimit;
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
    protected final SingleJointedArmSim armSim;
    protected NFRFeedbackProvider feedback;
    /**
     * Creates a new NFRRotatingArmJoint
     * @param config the configuration of the nfr rotating arm joint
     * @param controller the motor controller for the rotating arm joint
     * @param externalEncoder the optional external encoder
     */
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
        feedback = null;
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
    public void setSpeed(double speed, NFRFeedbackProvider feedback)
    {
        this.feedback = feedback;
        feedback.setSetpoint(speed);
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
        public DefaultCommand(NFRFeedbackProvider feedback, DoubleSupplier supplier)
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
            setSpeed(supplier.getAsDouble(), feedback);
        }
    }
    /**
     * Gets the default rotating arm command
     * @param supplier a supplier for the arm movement (ie joystick input)
     * @return the default rotating arm command
     */
    public Command getDefaultRotatingArmCommand(NFRFeedbackProvider feedback, DoubleSupplier supplier)
    {
        return new DefaultCommand(feedback, supplier);
    }
    /**
     * The set angle command for the NFRRotatingArmJoint.
     */
    public class SetAngle extends CommandBase
    {
        protected final Rotation2d targetAngle;
        protected final NFRFeedbackProvider positionalFeedback, speedFeedback;
        /**
         * Creates a new set angle.
         * @param targetAngle the target angle for the arm to go to.
         */
        public SetAngle(NFRFeedbackProvider positionalFeedback, NFRFeedbackProvider speedFeedback, Rotation2d targetAngle)
        {
            addRequirements(NFRRotatingArmJoint.this);
            this.targetAngle = targetAngle;
            this.positionalFeedback = positionalFeedback;
            this.speedFeedback = speedFeedback;
        }
        /**
         * Initializes the command which resets the pid controller or sets the internal closed-loop control.
         */
        @Override
        public void initialize()
        {
            positionalFeedback.setSetpoint(targetAngle.getRotations());
        }
        /**
         * If using pid controller, computes closed loop control
         */
        @Override
        public void execute()
        {
            positionalFeedback.runFeedback(getRotation().getRotations());
        }
        /**
         * Returns whether within 5 degrees of target angle
         */
        @Override
        public boolean isFinished()
        {
            return positionalFeedback.atSetpoint();
        }
    }
    /**
     * Gets the set angle command
     * @param targetAngle target angle for arm to go to
     * @return the set angle command
     */
    public Command getSetAngleCommand(NFRFeedbackProvider positionalFeedback, NFRFeedbackProvider speedFeedback,
        Rotation2d targetAngle)
    {
        return new SetAngle(positionalFeedback, speedFeedback, targetAngle);
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
    @Override
    public void periodic()
    {
        if (DriverStation.isEnabled())
        {
            if (feedback != null)
            {
                feedback.runFeedback(getRotation().getRotations());
            }
        }
    }
}
