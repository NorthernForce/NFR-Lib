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
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This is a generic tank drive that supports closed loop and open loop control. Uses common NFR interfaces such as
 * NFRGyro
 */
public class NFRTankDrive extends NFRDrive
{
    public static class NFRTankDriveConfiguration extends NFRDriveConfiguration
    {
        protected final double trackWidth, gearRatio, wheelRadius, moi, mass, maxSpeed, maxThetaVelocity;
        protected final int velocityPidSlot, positionPidSlot;
        protected final boolean useClosedLoopControl, useClosedLoopBrake, useTrapezoidalPositioning;
        protected final DCMotor gearbox;
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
    @Override
    public Pose2d getEstimatedPose()
    {
        return estimator.getEstimatedPosition();
    }
    @Override
    public void resetPose(Pose2d pose)
    {
        estimator.resetPosition(
            gyro.getGyroYaw(),
            leftSide.getSelectedEncoder().getPosition(),
            rightSide.getSelectedEncoder().getPosition(),
            pose
        );
    }
    @Override
    public void addVisionEstimate(double timestamp, Pose2d pose)
    {
        estimator.addVisionMeasurement(pose, timestamp);
    }
    @Override
    public ChassisSpeeds getChassisSpeeds()
    {
        return kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
            leftSide.getSelectedEncoder().getVelocity(),
            rightSide.getSelectedEncoder().getVelocity()
        ));
    }
    @Override
    public void setChassisSpeeds(ChassisSpeeds speeds)
    {
        var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        leftSide.setVelocity(config.velocityPidSlot, wheelSpeeds.leftMetersPerSecond);
        rightSide.setVelocity(config.velocityPidSlot, wheelSpeeds.rightMetersPerSecond);
        robotDrive.feed();
    }
    @Override
    public Command getDefaultDriveCommand(DoubleSupplier... suppliers)
    {
        if (config.useClosedLoopControl)
        {
            return Commands.run(() -> {
                setChassisSpeeds(new ChassisSpeeds(
                    suppliers[0].getAsDouble() * config.maxSpeed,
                    0,
                    suppliers[1].getAsDouble() * config.maxThetaVelocity
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
    @Override
    public Command getStopCommand() {
        if (config.useClosedLoopControl)
        {
            return Commands.deadline(Commands.waitUntil(() -> {
                return Math.abs(leftSide.getSelectedEncoder().getVelocity()) <= 0.1
                    || Math.abs(rightSide.getSelectedEncoder().getVelocity()) <= 0.1;
            }), Commands.run(() -> {
                setChassisSpeeds(new ChassisSpeeds());
            }));
        }
        else
        {
            return Commands.deadline(Commands.waitUntil(() -> {
                return Math.abs(leftSide.getSelectedEncoder().getVelocity()) <= 0.1
                    && Math.abs(rightSide.getSelectedEncoder().getVelocity()) <= 0.1;
            }), Commands.run(() -> {
                robotDrive.arcadeDrive(0, 0);
            }));
        }
    }
    protected class DriveMeters extends CommandBase
    {
        protected final double meters, speed;
        protected double startingLeft, startingRight;
        public DriveMeters(double meters, double speed)
        {
            addRequirements(NFRTankDrive.this);
            this.meters = meters;
            this.speed = speed;
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
    @Override
    public Command getDriveMetersCommand(double meters, double speed)
    {
        return new DriveMeters(meters, speed);
    }
}