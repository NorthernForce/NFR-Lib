package org.northernforce.subsystems.drive;

import org.northernforce.gyros.NFRGyro;
import org.northernforce.subsystems.drive.swerve.NFRSwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The subsystem used for a swerve drive.
 */
public class NFRSwerveDrive extends NFRDrive
{
    /**
     * The configuration of the swerve drive.
     */
    public static class NFRSwerveDriveConfiguration extends NFRDriveConfiguration
    {
        /**
         * Creates a new configuration.
         * @param name the name of the swerve drive. Typically "drive" or something similar.
         */
        public NFRSwerveDriveConfiguration(String name)
        {
            super(name);
        }
    }
    protected final NFRSwerveDriveConfiguration config;
    protected final NFRSwerveModule[] modules;
    protected final NFRGyro gyro;
    protected final SwerveDriveKinematics kinematics;
    protected final SwerveDrivePoseEstimator poseEstimator;
    /**
     * Creates a new NFRSwerveDrive.
     * @param config the configuration for the swerve drive.
     * @param modules the array of modules
     * @param offsets the array of offsets... this should be equal in length to that of the modules.
     * @param gyro the imu.
     */
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
    /**
     * Returns an array of all of the positions of the swerve modules
     * @return an array of SwerveModulePositions
     */
    public SwerveModulePosition[] getPositions()
    {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++)
        {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }
    /**
     * Gets the estimated pose that is a combination of odometry and vision estimates
     * @return pose consisting of x, y, and theta. Relative to wpilib origin (blue corner).
     */
    @Override
    public Pose2d getEstimatedPose()
    {
        return poseEstimator.getEstimatedPosition();
    }
    /**
     * Resets the pose to a new pose. This is meant for at the start of a match. Not meant for vision readings.
     * @param newPose pose consisting of x, y, and theta. Relative to wpilib origin (blue corner).
     */
    @Override
    public void resetPose(Pose2d newPose)
    {
        gyro.setYawOffset(newPose.getRotation().minus(getRotation()));
        poseEstimator.resetPosition(gyro.getGyroYaw(), getPositions(), newPose);
    }
    /**
     * Adds a vision estimate that is factored into the pose estimation.
     * @param timestamp timestamp of the pose. Seconds.
     * @param pose pose consisting of x, y, and theta. Relative to wpilib origin (blue corner).
     */
    @Override
    public void addVisionEstimate(double timestamp, Pose2d pose)
    {
        poseEstimator.addVisionMeasurement(pose, timestamp);
    }
    /**
     * Returns an array of all of the states of the swerve modules
     * @return an array of SwerveModuleStates
     */
    public SwerveModuleState[] getStates()
    {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++)
        {
            states[i] = modules[i].getState();
        }
        return states;
    }
    /**
     * Gets the current chassis speeds as recored by the encoder
     * @return chassis speeds which holds vx, vy, and vtheta.
     */
    @Override
    public ChassisSpeeds getChassisSpeeds()
    {
        return kinematics.toChassisSpeeds(getStates());
    }
    /**
     * Gets the swerve module instances
     * @return an array of the swerve modules
     */
    public NFRSwerveModule[] getModules()
    {
        return modules;
    }
    /**
     * Converts chassis speeds to target swerve module states
     * @param speeds the vector/twist of target chassis speed.
     * @return an array of swerve module states.
     */
    public SwerveModuleState[] toModuleStates(ChassisSpeeds speeds)
    {
        return kinematics.toSwerveModuleStates(speeds);
    }
    /**
     * Updates the pose estimations with the latest encoder readings.
     */
    @Override
    public void periodic()
    {
        poseEstimator.update(gyro.getGyroYaw(), getPositions());
    }
    /**
     * Gets the rotation of the swerve module as reported by the gyroscope.
     * @return
     */
    public Rotation2d getRotation()
    {
        return gyro.getGyroYaw();
    }
}
