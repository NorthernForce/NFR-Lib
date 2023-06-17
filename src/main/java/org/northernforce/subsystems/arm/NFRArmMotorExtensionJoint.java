package org.northernforce.subsystems.arm;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.northernforce.encoders.NFREncoder;
import org.northernforce.motors.NFRMotorController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** Creates an NFRArmMotorExtensionJoint. */
public class NFRArmMotorExtensionJoint extends NFRArmJoint{

    private final NFRArmMotorExtensionJointConfiguration config;
    private NFRMotorController motor;
    private Optional<NFREncoder> externalEncoder;
    private Optional<PIDController> pidController;

    /**
     * Creates a new NFRArmMotorExtensionJoint
     * @param config the corresponding config class
     * @param motor the motor to use
     * @param externalEncoder the encoder to use
     * @param pidController the pid controller
     */
    public NFRArmMotorExtensionJoint(NFRArmMotorExtensionJointConfiguration config, NFRMotorController motor, Optional<NFREncoder> externalEncoder, Optional<PIDController> pidController) {
        super(config);
        this.config = config;
        this.externalEncoder = externalEncoder;
        if(externalEncoder.isPresent()) {
            this.externalEncoder.get().setConversionFactor(config.metersPerRotation);
        } else {
            motor.getSelectedEncoder().setConversionFactor(config.metersPerRotation);
        }
    }

    /**
     *  the config for the {@link NFRArmMotorExtensionJoint} class
     */
    public static class NFRArmMotorExtensionJointConfiguration extends NFRArmJointConfiguration {
        /**
         * This represents the static offset from the end of the last component
         * for example if this was a two join rotational arm and this represented the 2nd joint 
         * it would be the transform between the end of the first arm component to this joint
         * 
         * if this was the first component it would be the transform from the center of the robot to the joint
         */
        protected Transform3d originOffset;
        protected double retractedArmLength;
        protected double extendedArmLength;
        protected int pidSlot;
        protected double metersPerRotation;
        protected double positiveLimit;
        protected double negativeLimit;
        protected boolean useTrapezoidalPositioning;
        protected double tolerance;

        /**
         * Creates the configuration for an arm motor extension
         * @param name the name of the subsystem
         */
        public NFRArmMotorExtensionJointConfiguration(String name) {
            super(name);
        }

        /**
         * Creates the configuration for an arm motor extension
         * @param name the name of the subsystem to create
         * @param originOffset the offset from the end of the last component to this joint 
         * more explanation in the {@link NFRArmJointConfiguration} super class
         * @param retractedArmLength The length of the arm while fully retracted
         * @param extendedArmLength The length of the arm while fully extended
         * @param pidSlot the pidSlot to use for the motor, either a encoder or PIDController will be preferred
         * @param metersPerRotation the number of meters the arm will extend or retract per rotation
         * @param useTrapezoidalPositioning set to true to use useTrapezoidalPositioning
         * @param tolerance the acceptable tolerance in meters
         */
        public NFRArmMotorExtensionJointConfiguration(String name, Transform3d originOffset, 
            double retractedArmLength, double extendedArmLength, int pidSlot, double metersPerRotation, boolean useTrapezoidalPositioning, double tolerance) {
            super(name);
            this.originOffset = originOffset;
            this.retractedArmLength = retractedArmLength;
            this.extendedArmLength = extendedArmLength;
            this.pidSlot = pidSlot;
            this.metersPerRotation = metersPerRotation;
            this.useTrapezoidalPositioning = useTrapezoidalPositioning;
            this.tolerance = tolerance;
        }

        /**
         * set the originOffset
         * @param originOffset the offset from the end of the last component to this joint 
         * more explanation in the {@link NFRArmJointConfiguration} super class
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration withStartPosition(Transform3d originOffset) {
            this.originOffset = originOffset;
            return this;
        }

        /**
         * set the arm length
         * @param extendedArmLength The length of the arm while fully extended
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration withExtendedArmLength(double extendedArmLength) {
            this.extendedArmLength = extendedArmLength;
            return this;
        }

        /**
         * set the arm length
         * @param retractedArmLength The length of the arm while fully retracted
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration withRetractedArmLength(double retractedArmLength) {
            this.retractedArmLength = retractedArmLength;
            return this;
        }

        /**
         * set the pidSlot
         * @param pidSlot The length of the arm while fully retracted
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration withPidSlot(int pidSlot) {
            this.pidSlot = pidSlot;
            return this;
        }
        
        /**
         * set the metersPerRotation
         * @param metersPerRotation the meters per one rotation of the motor
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration withMetersPerRotation(double metersPerRotation) {
            this.metersPerRotation = metersPerRotation;
            return this;
        }

        /**
         * set useTrapezoidalPositioning
         * @param useTrapezoidalPositioning whether or not to use Trapezoidal Positioning
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration withUseTrapezoidalPositioning(boolean useTrapezoidalPositioning) {
            this.useTrapezoidalPositioning = useTrapezoidalPositioning;
            return this;
        }
        
        /**
         * set the tolerance
         * @param tolerance the tolerance in meters
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration withTolerance(double tolerance) {
            this.tolerance = tolerance;
            return this;
        }

        /**
         * set the positive limit
         * @param positiveLimit the positive limit
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration withPositiveLimit(double positiveLimit) {
            this.positiveLimit = positiveLimit;
            return this;
        }

        /**
         * set the negative limit
         * @param negativeLimit the negative distance limit
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration withNegativeLimit(double negativeLimit) {
            this.negativeLimit = negativeLimit;
            return this;
        }
    }

    /**
     * the default class for the {@link NFRArmMotorExtensionJoint}
     */
    public class DefaultCommand extends CommandBase {
        /** the supplier to control the speed used in the default command */
        protected DoubleSupplier speedSupplier;

        /**
         * creates a new default command
         * @param speedSupplier the speed to use for extending or retracting the arm
         */
        public DefaultCommand(DoubleSupplier speedSupplier) {
            addRequirements(NFRArmMotorExtensionJoint.this);
            this.speedSupplier = speedSupplier;
        }

        /**
         * Executes and checks whether outside of limits
         */
        @Override
        public void execute()
        {
            double targetSpeed = speedSupplier.getAsDouble();
            if (externalEncoder.isPresent() && getLength() <= config.negativeLimit &&
                targetSpeed < 0)
            {
                targetSpeed = 0;
            }
            else if (externalEncoder.isPresent() && getLength() >= config.positiveLimit &&
                targetSpeed > 0)
            {
                targetSpeed = 0;
            }
            motor.set(targetSpeed);
        }
    }

    /**
     * gets the default command for the {@link NFRArmMotorExtensionJoint} class
     * @param speedSupplier to control the extension
     * @return the command
     */
    public DefaultCommand getDefaultCommand(DoubleSupplier speedSupplier) {
        return new DefaultCommand(speedSupplier);
    }

    /**
     * the set length command for the {@link NFRArmMotorExtensionJoint} class
     */
    public class setLength extends CommandBase {
        protected final double targetDistance;
        /**
         * Creates a new set length.
         * @param targetDistance the distance past the retracted length
         */
        public setLength(double targetDistance)
        {
            addRequirements(NFRArmMotorExtensionJoint.this);
            this.targetDistance = targetDistance;
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
                pidController.get().setSetpoint(targetDistance);
            }
            else
            {
                if (config.useTrapezoidalPositioning)
                {
                    motor.setPositionTrapezoidal(config.pidSlot, targetDistance);
                }
                else
                {
                    motor.setPosition(config.pidSlot, targetDistance);
                }
            }
        }
        /**
         * If using pid controller, computes closed loop control
         */
        @Override
        public void execute()
        {
            if (pidController.isPresent()) {
                double targetSpeed = pidController.get().calculate(getLength());
                if (externalEncoder.isPresent() && getLength() <= config.negativeLimit &&
                    targetSpeed < 0) {
                    targetSpeed = 0;
                }
                else if (externalEncoder.isPresent() && getLength() >= config.positiveLimit &&
                    targetSpeed > 0) {
                    targetSpeed = 0;
                }
                motor.set(targetSpeed);
            }
        }
        /**
         * Returns whether within tolerance of the angle
         */
        @Override
        public boolean isFinished()
        {
            return Math.abs(getLength() - targetDistance) <= config.tolerance;
        }
    }

    /**
     * gets the set length command for the {@link NFRArmMotorExtensionJoint} class
     * @param targetDistance the distance to extend or retract to.
     * @return the command
     */
    public setLength getSetLengthCommand(double targetDistance) {
        return new setLength(targetDistance);
    }

    /**
     * the extend by speed command for the {@link NFRArmMotorExtensionJoint} class
     */
    public class ExtendBySpeed extends CommandBase {
        /**
         * the speed to extend at
         */
        protected double speed;

        /**
         * creates a new extend by speed command
         * @param speed the speed to extend at
         */
        public ExtendBySpeed(double speed) {
            this.speed = speed;
        }

        @Override
        public void initialize() {
            motor.set(speed);
        }

        @Override
        public boolean isFinished() {
            return config.retractedArmLength + getLength() < config.extendedArmLength;
        }
    }
    
    /**
     * gets the extend by speed command for the {@link NFRArmMotorExtensionJoint} class
     * @param speed the speed to extend at
     * @return the command
     */
    public ExtendBySpeed getExtendBySpeedCommand(double speed) {
        return new ExtendBySpeed(speed);
    }

    /**
     * the retract by speed class for the {@link NFRArmMotorExtensionJoint} class
     */
    public class RetractBySpeed extends CommandBase {
        protected double speed;

        /**
         * creates a new retract by speed command
         * @param speed the speed to retract at
         */
        public RetractBySpeed(double speed) {
            this.speed = speed;
        }

        @Override
        public void initialize() {
            motor.set(speed);
        }

        @Override
        public boolean isFinished() {
            return getLength() > 0;
        }
    }
    
    /**
     * gets the retract by speed command for the {@link NFRArmMotorExtensionJoint} class
     * @param speed the speed to retract at
     * @return the command
     */
    public RetractBySpeed getRetractBySpeedCommand(double speed) {
        return new RetractBySpeed(speed);
    }

    /**
     * gets the extend by PID command for the {@link NFRArmMotorExtensionJoint} class
     * @return the command
     */
    public setLength getExtendByPIDCommand() {
        return new setLength(config.extendedArmLength - config.retractedArmLength);
    }

    /**
     * gets the retract by PID command for the {@link NFRArmMotorExtensionJoint} class
     * @return the command
     */
    public setLength getRetractByPIDCommand() {
        return new setLength(0);
    }

    /**
     * the extend until boolean class for the {@link NFRArmMotorExtensionJoint} class
     */
    public class ExtendUntilBoolean extends ExtendBySpeed {
        protected BooleanSupplier stopCondition;

        /**
         * creates a new extend until boolean
         * @param speed the speed to extend at
         * @param stopCondition boolean supplier should return true when the command should stop
         */
        public ExtendUntilBoolean(double speed, BooleanSupplier stopCondition) {
            super(speed);
            this.stopCondition = stopCondition;
        }

        @Override
        public boolean isFinished() {
            return stopCondition.getAsBoolean();
        }
    }

    /**
     * the retract until boolean class for the {@link NFRArmMotorExtensionJoint} class
     */
    public class RetractUntilBoolean extends RetractBySpeed {
        protected BooleanSupplier stopCondition;

        /**
         * creates a new retract until boolean
         * @param speed the speed to retract at
         * @param stopCondition boolean supplier should return true when the command should stop
         */
        public RetractUntilBoolean(double speed, BooleanSupplier stopCondition) {
            super(speed);
            this.stopCondition = stopCondition;
        }

        @Override
        public boolean isFinished() {
            return stopCondition.getAsBoolean();
        }
    }

    /**
     * gets the length of the arm length in this case is the distance past the fully retracted point
     * @return returns the length as defined above
     */
    public double getLength() {
        double dLength;
        if (externalEncoder.isPresent()) dLength = externalEncoder.get().getPosition();
        else dLength = motor.getSelectedEncoder().getPosition();
        return dLength;
    }

    @Override
    public Transform3d getEndState() {
        return config.originOffset.plus(new Transform3d(new Translation3d(getLength(), 0, 0), new Rotation3d()));
    }
}
