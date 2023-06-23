package org.northernforce.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.northernforce.motors.MotorEncoderMismatchException;
import org.northernforce.motors.NFRMotorController;
import org.northernforce.gyros.NFRGyro;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This is a generic tank drive that supports closed loop and open loop control. Uses common NFR interfaces such as
 * NFRGyro and NFRMotorController. Inherits from NFRDrive.
 */
public class NFRTankDrive extends NFRDrive
{
    /**
     * This holds configurations for simulation and real driving of the robot.
     */
    public static class NFRTankDriveConfiguration extends NFRDriveConfiguration
    {
        protected final double trackWidth, gearRatio, wheelRadius, moi, mass, maxSpeed, maxThetaVelocity;
        protected final int velocityPidSlot, positionPidSlot;
        protected final boolean useClosedLoopControl, useClosedLoopBrake, useTrapezoidalPositioning;
        protected final DCMotor gearbox;
        /**
         * Creates a new NFRTankDriveConfiguration.
         * @param name the unique identifier for the subsystem, eg. "drive"
         * @param trackWidth the width of the chassis in meters
         * @param gearRatio the gear ratio between the motor and the wheel.
         * @param wheelRadius the radius of the wheel in meters
         * @param moi the moment of inertia of the drive train. Can be left as zero if simulation is not used.
         * @param mass the mass of the robot. Can be left as zero is simulation is not used.
         * @param maxSpeed the max speed of the robot. Best estimation is used for closed loop control. It is in
         * meters/s.
         * @param maxThetaVelocity the max angular velocity of the robot. Best estimation is used for closed loop
         * control. It is in radians/s.
         * @param velocityPidSlot the index of the pid slot that contains the velocity closed-loop configuration.
         * @param positionPidSlot the index of the pid slot that contains the position closed-loop configuration.
         * @param useClosedLoopControl whether or not to use closed loop control (going absolute speeds) for the
         * default command.
         * @param useClosedLoopBrake whether or not to use basic braking (just setting motor to zero) or using
         * closed-loop control to brake the robot for the stop command.
         * @param useTrapezoidalPositioning whether or not to use the more advanced trapezoidal positioning to get to
         * location. Must be tuned.
         * @param gearbox the gearbox that the controller is controlling. Only necessary if simulation is used.
         */
        public NFRTankDriveConfiguration(String name, double trackWidth, double gearRatio, double wheelRadius, double moi,
            double mass, double maxSpeed, double maxThetaVelocity, int velocityPidSlot, int positionPidSlot,
            boolean useClosedLoopControl, boolean useClosedLoopBrake, boolean useTrapezoidalPositioning, DCMotor gearbox)
        {
            super(name);
            this.trackWidth = trackWidth;
            this.gearRatio = gearRatio;
            this.wheelRadius = wheelRadius;
            this.gearbox = gearbox;
            this.moi = moi;
            this.mass = mass;
            this.maxSpeed = maxSpeed;
            this.velocityPidSlot = velocityPidSlot;
            this.positionPidSlot = positionPidSlot;
            this.useClosedLoopControl = useClosedLoopControl;
            this.useClosedLoopBrake = useClosedLoopBrake;
            this.maxThetaVelocity = maxThetaVelocity;
            this.useTrapezoidalPositioning = useTrapezoidalPositioning;
        }
    }
    protected final DifferentialDriveKinematics kinematics;
    protected final NFRMotorController leftSide, rightSide;
    protected final DifferentialDrive robotDrive;
    protected final DifferentialDrivePoseEstimator estimator;
    protected final NFRGyro gyro;
    protected final DifferentialDrivetrainSim simulator;
    protected final NFRTankDriveConfiguration config;
    /**
     * Creates a new NFR Tank Drive.
     * @param config the class containing the configuration parameters.
     * @param leftSide the NFRMotorController for the left side
     * @param rightSide the NFRMotorController for the right side
     * @param gyro the gyroscope
     */
    public NFRTankDrive(NFRTankDriveConfiguration config, NFRMotorController leftSide, NFRMotorController rightSide,
        NFRGyro gyro)
    {
        super(config);
        this.config = config;
        kinematics = new DifferentialDriveKinematics(config.trackWidth);
        this.leftSide = leftSide;
        this.rightSide = rightSide;
        robotDrive = new DifferentialDrive(leftSide, rightSide);
        try
        {
            leftSide.setSelectedEncoder(leftSide.getIntegratedEncoder());
            rightSide.setSelectedEncoder(rightSide.getIntegratedEncoder());
        }
        catch (MotorEncoderMismatchException e)
        {
            e.printStackTrace();
        }
        leftSide.getSelectedEncoder().setConversionFactor((Math.PI * config.wheelRadius * 2) / (config.gearRatio));
        rightSide.getSelectedEncoder().setConversionFactor((Math.PI * config.wheelRadius * 2) / (config.gearRatio));
        this.gyro = gyro;
        estimator = new DifferentialDrivePoseEstimator(kinematics, gyro.getGyroYaw(),
            leftSide.getSelectedEncoder().getPosition(), rightSide.getSelectedEncoder().getPosition(),
            new Pose2d());
        if (RobotBase.isSimulation())
        {
            simulator = new DifferentialDrivetrainSim(
                config.gearbox, config.gearRatio, config.moi, config.mass,
                config.wheelRadius, config.trackWidth,
                VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005)
            );
        }
        else
        {
            simulator = null;
        }
    }
    /**
     * Gets the estimated pose that is a combination of odometry and vision estimates
     * @return pose consisting of x, y, and theta. Relative to wpilib origin (blue corner).
     */
    @Override
    public Pose2d getEstimatedPose()
    {
        return estimator.getEstimatedPosition();
    }
    /**
     * Resets the pose to a new pose. This is meant for at the start of a match. Not meant for vision readings.
     * @param newPose pose consisting of x, y, and theta. Relative to wpilib origin (blue corner).
     */
    @Override
    public void resetPose(Pose2d newPose)
    {
        estimator.resetPosition(
            gyro.getGyroYaw(),
            leftSide.getSelectedEncoder().getPosition(),
            rightSide.getSelectedEncoder().getPosition(),
            newPose
        );
    }
    /**
     * Adds a vision estimate that is factored into the pose estimation.
     * @param timestamp timestamp of the pose. Seconds.
     * @param pose pose consisting of x, y, and theta. Relative to wpilib origin (blue corner).
     */
    @Override
    public void addVisionEstimate(double timestamp, Pose2d pose)
    {
        estimator.addVisionMeasurement(pose, timestamp);
    }
    /**
     * Gets the current chassis speeds as recored by the encoder
     * @return chassis speeds which holds vx, vy, and vtheta.
     */
    @Override
    public ChassisSpeeds getChassisSpeeds()
    {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
            leftSide.getSelectedEncoder().getVelocity(),
            rightSide.getSelectedEncoder().getVelocity()
        ));
    }
    /**
     * Sets the chassis to a specific speed of vx, vy, and vtheta.
     * @param speeds
     */
    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds)
    {
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        leftSide.setVelocity(config.velocityPidSlot, wheelSpeeds.leftMetersPerSecond);
        rightSide.setVelocity(config.velocityPidSlot, wheelSpeeds.rightMetersPerSecond);
        robotDrive.feed();
    }
    /**
     * Gets the default drive command that uses double suppliers (from controllers) to move.
     * @param suppliers the suppliers. Two for arcade, three for swerve.
     * @return Default drive command for subsystem
     */
    @Override
    public Command getDefaultDriveCommand(DoubleSupplier... suppliers)
    {
        if (config.useClosedLoopControl)
        {
            return Commands.run(() -> {
                setChassisSpeeds(new ChassisSpeeds(
                    suppliers[0].getAsDouble() * config.maxSpeed,
                    0,
                    -suppliers[1].getAsDouble() * config.maxThetaVelocity
                ));
            }, this);
        }
        else
        {
            return Commands.run(() -> {
                robotDrive.arcadeDrive(suppliers[0].getAsDouble(), suppliers[1].getAsDouble());
            }, this);
        }
    }
    /**
     * Gets a command that stops the robot. Finishes when stopped.
     * @return a stop command
     */
    @Override
    public Command getStopCommand() {
        if (config.useClosedLoopControl)
        {
            return Commands.deadline(Commands.waitUntil(() -> {
                return Math.abs(leftSide.getSelectedEncoder().getVelocity()) <= 0.1
                    || Math.abs(rightSide.getSelectedEncoder().getVelocity()) <= 0.1;
            }), Commands.run(() -> {
                setChassisSpeeds(new ChassisSpeeds());
            }, this));
        }
        else
        {
            return Commands.deadline(Commands.waitUntil(() -> {
                return Math.abs(leftSide.getSelectedEncoder().getVelocity()) <= 0.1
                    && Math.abs(rightSide.getSelectedEncoder().getVelocity()) <= 0.1;
            }), Commands.run(() -> {
                robotDrive.arcadeDrive(0, 0);
            }, this));
        }
    }
    /**
     * An internal class responsible for driving a tank drive forward a set number of meters.
     */
    protected class DriveMeters extends CommandBase
    {
        protected final double meters;
        protected double startingLeft, startingRight;
        public DriveMeters(double meters)
        {
            addRequirements(NFRTankDrive.this);
            this.meters = meters;
        }
        @Override
        public void initialize()
        {
            if (config.useTrapezoidalPositioning)
            {
                leftSide.setPositionTrapezoidal(config.positionPidSlot, leftSide.getSelectedEncoder().getPosition() + meters);
                rightSide.setPositionTrapezoidal(config.positionPidSlot, rightSide.getSelectedEncoder().getPosition() + meters);
            }
            else
            {
                leftSide.setPosition(config.positionPidSlot, leftSide.getSelectedEncoder().getPosition() + meters);
                rightSide.setPosition(config.positionPidSlot, rightSide.getSelectedEncoder().getPosition() + meters);
            }
        }
        @Override
        public void execute()
        {
            robotDrive.feed();
        }
        @Override
        public boolean isFinished()
        {
            return Math.abs(leftSide.getSelectedEncoder().getPosition() - meters) <= 0.1
                && Math.abs(rightSide.getSelectedEncoder().getPosition() - meters) <= 0.1;
        }
    }
    /**
     * Gets a command to drive the robot forward a set amount of meters at a set speed. Uses encoders.
     * @param meters the distance to drive
     * @return a command to drive forward
     */
    @Override
    public Command getDriveMetersCommand(double meters)
    {
        return new DriveMeters(meters);
    }
    /**
     * This is the periodic function. In it, the estimator is fed the current information from the encoders and
     * gyroscope.
     */
    @Override
    public void periodic()
    {
        estimator.update(
            gyro.getGyroYaw(),
            leftSide.getSelectedEncoder().getPosition(),
            rightSide.getSelectedEncoder().getPosition()
        );
    }
    /**
     * This is the simulation periodic function. The simulator is fed the inputs from the motors, and the simulation
     * returns new encoder values, and a new gyroscope heading.
     */
    @Override
    public void simulationPeriodic()
    {
        simulator.setInputs(
            leftSide.get() * RobotController.getBatteryVoltage(),
            rightSide.get() * RobotController.getBatteryVoltage()
        );
        simulator.update(0.02);
        leftSide.getSelectedEncoder().addSimulationPosition(simulator.getLeftVelocityMetersPerSecond() * 0.02);
        rightSide.getSelectedEncoder().addSimulationPosition(simulator.getRightVelocityMetersPerSecond() * 0.02);
        gyro.addSimulationYaw(simulator.getHeading());
    }
}