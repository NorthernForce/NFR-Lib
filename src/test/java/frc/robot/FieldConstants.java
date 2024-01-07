package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public class FieldConstants
{
    /** 
     * All of these locations are relative to the WPILIB origin of the field, the blue corner.
     * All of the arrays of coordinates begin with the side of the community zones and go to the side of the loading zone.
     * The cone locations are alternating between middle and high.
     * All of these values were gained through the CAD for the field.
     */
    public static final Translation3d[] BLUE_CONE_PLACEMENT_LOCATIONS = new Translation3d[] {
        new Translation3d(
            0.795,
            0.519,
            0.866
        ),
        new Translation3d(
            0.366,
            0.519,
            1.162
        ),
        new Translation3d(
            0.795,
            1.629,
            0.866
        ),
        new Translation3d(
            0.366,
            1.629,
            1.162
        ),
        new Translation3d(
            0.795,
            2.195,
            0.866
        ),
        new Translation3d(
            0.366,
            2.195,
            1.162
        ),
        new Translation3d(
            0.795,
            3.330,
            0.866
        ),
        new Translation3d(
            0.366,
            3.330,
            1.162
        ),
        new Translation3d(
            0.795,
            3.865,
            0.866
        ),
        new Translation3d(
            0.366,
            3.865,
            1.162
        ),
        new Translation3d(
            0.795,
            5.007,
            0.866
        ),
        new Translation3d(
            0.366,
            5.007,
            1.162
        )
    };
    public static final Translation3d[] RED_CONE_PLACEMENT_LOCATIONS = new Translation3d[] {
        new Translation3d(
            15.717,
            0.519,
            0.866
        ),
        new Translation3d(
            16.182,
            0.519,
            1.162
        ),
        new Translation3d(
            15.717,
            1.629,
            0.866
        ),
        new Translation3d(
            16.182,
            1.629,
            1.162
        ),
        new Translation3d(
            15.717,
            2.195,
            0.866
        ),
        new Translation3d(
            16.182,
            2.195,
            1.162
        ),
        new Translation3d(
            15.717,
            3.330,
            0.866
        ),
        new Translation3d(
            16.182,
            3.330,
            1.162
        ),
        new Translation3d(
            15.717,
            3.865,
            0.866
        ),
        new Translation3d(
            16.182,
            3.865,
            1.162
        ),
        new Translation3d(
            15.717,
            5.007,
            0.866
        ),
        new Translation3d(
            16.182,
            5.007,
            1.162
        )
    };
    public static final Translation3d[] BLUE_CUBE_PLACEMENT_LOCATIONS = new Translation3d[]
    {
        new Translation3d(
            0.7945,
            1.0715,
            0.523
        ),
        new Translation3d(
            0.3505,
            1.0715,
            0.826
        ),
        new Translation3d(
            0.7945,
            2.754,
            0.523
        ),
        new Translation3d(
            0.3505,
            2.754,
            0.826
        ),
        new Translation3d(
            0.7945,
            4.4305,
            0.523
        ),
        new Translation3d(
            0.3505,
            4.4305,
            0.826
        )
    };
    public static final Translation3d[] RED_CUBE_PLACEMENT_LOCATIONS = new Translation3d[]
    {
        new Translation3d(
            15.735,
            1.0715,
            0.523
        ),
        new Translation3d(
            16.449,
            1.0715,
            0.826
        ),
        new Translation3d(
            15.735,
            2.754,
            0.523
        ),
        new Translation3d(
            16.449,
            2.754,
            0.826
        ),
        new Translation3d(
            15.735,
            4.4305,
            0.523
        ),
        new Translation3d(
            16.449,
            4.4305,
            0.826
        )
    };
    public static final Translation3d[] BLUE_FLOOR_PLACEMENT_LOCATIONS = new Translation3d[]
    {
        new Translation3d(
            1.118,
            0.519,
            0
        ),
        new Translation3d(
            1.118,
            1.0715,
            0
        ),
        new Translation3d(
            1.118,
            1.629,
            0
        ),
        new Translation3d(
            1.118,
            2.195,
            0
        ),
        new Translation3d(
            1.118,
            2.754,
            0
        ),
        new Translation3d(
            1.118,
            3.330,
            0
        ),
        new Translation3d(
            1.118,
            3.865,
            0
        ),
        new Translation3d(
            1.118,
            4.4305,
            0
        ),
        new Translation3d(
            1.118,
            5.007,
            0
        )
    };
    public static final Translation3d[] RED_FLOOR_PLACEMENT_LOCATIONS = new Translation3d[]
    {
        new Translation3d(
            15.411,
            0.519,
            0
        ),
        new Translation3d(
            15.411,
            1.0715,
            0
        ),
        new Translation3d(
            15.411,
            1.629,
            0
        ),
        new Translation3d(
            15.411,
            2.195,
            0
        ),
        new Translation3d(
            15.411,
            2.754,
            0
        ),
        new Translation3d(
            15.411,
            3.330,
            0
        ),
        new Translation3d(
            15.411,
            3.865,
            0
        ),
        new Translation3d(
            15.411,
            4.4305,
            0
        ),
        new Translation3d(
            15.411,
            5.007,
            0
        )
    };
    public static final Translation3d[] RED_SHELF_LOCATIONS = new Translation3d[]
    {
        new Translation3d(
            1.605,
            5.575,
            0.946
        ),
        new Translation3d(
            1.605,
            7.61,
            0.946
        )
    };
    public static final Translation3d[] BLUE_SHELF_LOCATIONS = new Translation3d[]
    {
        new Translation3d(
            16.41,
            5.575,
            0.946
        ),
        new Translation3d(
            16.41,
            7.61,
            0.946
        )
    };
    public static final Translation3d[] BLUE_GAME_PIECE_AUTO_LOCATIONS = new Translation3d[]
    {
        new Translation3d(
            7.062,
            0.925,
            0
        ),
        new Translation3d(
            7.062,
            2.143,
            0
        ),
        new Translation3d(
            7.062,
            3.361,
            0
        ),
        new Translation3d(
            7.062,
            4.583,
            0
        )
    };
    public static final Translation3d[] RED_GAME_PIECE_AUTO_LOCATIONS = new Translation3d[]
    {
        new Translation3d(
            9.468,
            0.925,
            0
        ),
        new Translation3d(
            9.468,
            2.143,
            0
        ),
        new Translation3d(
            9.468,
            3.361,
            0
        ),
        new Translation3d(
            9.468,
            4.583,
            0
        )
    };
    public static AprilTagFieldLayout APRILTAG_LAYOUT;
    static {
        APRILTAG_LAYOUT = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
    }
    /** DRIVER ORIENTED */
    public static final Pose2d[] BLUE_POSES = new Pose2d[] {
        new Pose2d(1.693, 4.402, Rotation2d.fromDegrees(180)), /* DRIVER ORIENTED Blue left */
        new Pose2d(1.693, 2.761, Rotation2d.fromDegrees(180)), /* DRIVER ORIENTED Blue center */
        new Pose2d(1.693, 1.083, Rotation2d.fromDegrees(180)) /* DRIVER ORIENTED Blue right */
    };
    public static final Pose2d[] RED_POSES = new Pose2d[] {
        new Pose2d(14.895, 1.083, Rotation2d.fromDegrees(0)), /* DRIVER ORIENTED Red left */
        new Pose2d(14.895, 2.761, Rotation2d.fromDegrees(0)), /* DRIVER ORIENTED Red center */
        new Pose2d(14.895, 4.402, Rotation2d.fromDegrees(0)) /* DRIVER ORIENTED Red right */
    };
}