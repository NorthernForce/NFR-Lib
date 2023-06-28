package org.northernforce.subsystems.drive;

import java.util.function.DoubleSupplier;

import org.northernforce.subsystems.NFRSubsystem;
import org.northernforce.util.NFRFeedbackProvider;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The NFRDrive is an interface for all subsystems and provides common functionality among them.
 */
public abstract class NFRDrive extends NFRSubsystem
{
    /**
     * NFR Drive configuration has just a name. It must be overrided in NFRDrives.
     */
    public static abstract class NFRDriveConfiguration extends NFRSubsystemConfiguration
    {
        /**
         * Creates a new nfr drive config.
         * @param name a unique subsystem name.
         */
        public NFRDriveConfiguration(String name)
        {
            super(name);
        }
    }
    /**
     * Constructs the NFR Drive from a configuration containing a unique name
     * @param config nfr drive config
     */
    public NFRDrive(NFRDriveConfiguration config)
    {
        super(config);
    }
    /**
     * Gets the estimated pose that is a combination of odometry and vision estimates
     * @return pose consisting of x, y, and theta. Relative to wpilib origin (blue corner).
     */
    public abstract Pose2d getEstimatedPose();
    /**
     * Resets the pose to a new pose. This is meant for at the start of a match. Not meant for vision readings.
     * @param newPose pose consisting of x, y, and theta. Relative to wpilib origin (blue corner).
     */
    public abstract void resetPose(Pose2d newPose);
    /**
     * Adds a vision estimate that is factored into the pose estimation.
     * @param timestamp timestamp of the pose. Seconds.
     * @param pose pose consisting of x, y, and theta. Relative to wpilib origin (blue corner).
     */
    public abstract void addVisionEstimate(double timestamp, Pose2d pose);
    /**
     * Gets the current chassis speeds as recored by the encoder
     * @return chassis speeds which holds vx, vy, and vtheta.
     */
    public abstract ChassisSpeeds getChassisSpeeds();
    /**
     * Sets the chassis to a specific speed of vx, vy, and vtheta.
     * @param speeds
     */
    public abstract void setChassisSpeeds(NFRFeedbackProvider leftFeedback, NFRFeedbackProvider rightFeedback,
        ChassisSpeeds speeds);
    /**
     * Gets the default drive command that uses double suppliers (from controllers) to move.
     * @param suppliers the suppliers. Two for arcade, three for swerve.
     * @return Default drive command for subsystem
     */
    public abstract Command getDefaultDriveCommand(NFRFeedbackProvider leftFeedback, NFRFeedbackProvider rightFeedback,
        DoubleSupplier... suppliers);
    /**
     * Gets a command that stops the robot. Finishes when stopped.
     * @return a stop command
     */
    public abstract Command getStopCommand(NFRFeedbackProvider leftFeedback, NFRFeedbackProvider rightFeedback);
    /**
     * Gets a command to drive the robot forward a set amount of meters at a set speed. Uses encoders.
     * @param meters the distance to drive
     * @return a command to drive forward
     */
    public abstract Command getDriveMetersCommand(double meters, double tolerance, NFRFeedbackProvider leftFeedback,
        NFRFeedbackProvider rightFeedback);
}