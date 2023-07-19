package org.northernforce.subsystems.drive;

import org.northernforce.gyros.NFRGyro;
import org.northernforce.subsystems.drive.swerve.NFRSwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class NFRSwerveDrive extends NFRDrive
{
    public static class NFRSwerveDriveConfiguration extends NFRDriveConfiguration
    {
        protected double maxSpeed;
        public NFRSwerveDriveConfiguration(String name)
        {
            super(name);
            maxSpeed = 0;
        }
        public NFRSwerveDriveConfiguration(String name, double maxSpeed)
        {
            super(name);
            this.maxSpeed = maxSpeed;
        }
        public NFRSwerveDriveConfiguration withMaxSpeed(double maxSpeed)
        {
            this.maxSpeed = maxSpeed;
            return this;
        }
    }
    protected final NFRSwerveDriveConfiguration config;
    protected final NFRSwerveModule[] modules;
    protected final NFRGyro gyro;
    protected final SwerveDriveKinematics kinematics;
    protected final SwerveDrivePoseEstimator poseEstimator;
    public NFRSwerveDrive(NFRSwerveDriveConfiguration config, NFRSwerveModule[] modules, Translation2d[] offsets,
        NFRGyro gyro)
    {
        super(config);
        this.config = config;
        this.modules = modules;
        this.gyro = gyro;
        kinematics = new SwerveDriveKinematics(offsets);
        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getGyroYaw(), getPositions(), new Pose2d());
    }
    public SwerveModulePosition[] getPositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++)
        {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
    @Override
    public Pose2d getEstimatedPose()
    {
        return poseEstimator.getEstimatedPosition();
    }
    @Override
    public void resetPose(Pose2d newPose)
    {
        poseEstimator.resetPosition(gyro.getGyroYaw(), getPositions(), newPose);
    }
    @Override
    public void addVisionEstimate(double timestamp, Pose2d pose)
    {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }
    public SwerveModuleState[] getStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++)
        {
            states[i] = modules[i].getState();
        }
        return states;
    }
    @Override
    public ChassisSpeeds getChassisSpeeds()
    {
        return kinematics.toChassisSpeeds(getStates());
    }
    public NFRSwerveModule[] getModules()
    {
        return modules;
    }
    public SwerveModuleState[] toModuleStates(ChassisSpeeds speeds)
    {
        return kinematics.toSwerveModuleStates(speeds);
    }
    public SwerveModuleState[] scaleSpeeds(SwerveModuleState[] states)
    {
        SwerveModuleState[] newStates = new SwerveModuleState[states.length];
        for (int i = 0; i < states.length; i++)
        {
            newStates[i] = new SwerveModuleState(states[i].speedMetersPerSecond * config.maxSpeed,
                states[i].angle);
        }
        return newStates;
    }
}
