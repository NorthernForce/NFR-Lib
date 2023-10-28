package frc.robot.subsystems;

import org.northernforce.subsystems.drive.NFRDrive;
import org.northernforce.subsystems.ros.ROSCoprocessor;
import org.northernforce.subsystems.ros.geometry_msgs.PoseStamped;
import org.northernforce.subsystems.ros.geometry_msgs.PoseWithCovarianceStamped;
import org.northernforce.subsystems.ros.nav_msgs.Odometry;
import org.northernforce.subsystems.ros.primitives.Time;
import org.northernforce.subsystems.ros.std_msgs.Header;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.PoseWithCovariance;
import edu.wpi.rail.jrosbridge.messages.geometry.Twist;
import edu.wpi.rail.jrosbridge.messages.geometry.TwistWithCovariance;

public class Xavier extends ROSCoprocessor
{
    protected final NFRDrive drive;
    protected Twist targetVelocity = new Twist();
    public Xavier(NFRDrive drive)
    {
        super(new ROSCoprocessorConfiguration("xavier", "localhost", 5809));
        this.drive = drive;
        onConnect(() -> {
            subscribe("/realsense/pose_estimations",
                "geometry_msgs/PoseWithCovarianceStamped", this::recieveDetection);
            subscribe("/cmd_vel", "geometry_msgs/Twist", message -> {
                targetVelocity = Twist.fromMessage(message);
            });
        });
        onDisconnect(() -> {
            startConnecting();
        });
        startConnecting();
    }
    public void recieveDetection(Message message)
    {
        PoseWithCovarianceStamped poseStamped = PoseWithCovarianceStamped.fromMessage(message);
        Pose3d pose = new Pose3d(
            new Translation3d(
                poseStamped.pose.getPose().getPosition().getX(),
                poseStamped.pose.getPose().getPosition().getY(),
                poseStamped.pose.getPose().getPosition().getZ()
            ),
            new Rotation3d(
                new Quaternion(
                    poseStamped.pose.getPose().getOrientation().getW(),
                    poseStamped.pose.getPose().getOrientation().getX(),
                    poseStamped.pose.getPose().getOrientation().getY(),
                    poseStamped.pose.getPose().getOrientation().getZ()
                )
            )
        );
        drive.addVisionEstimate(poseStamped.header.getStamp().toSec(), pose.toPose2d());
    }
    public void publishOdometry(Pose2d odometryPose, ChassisSpeeds speeds)
    {
        Odometry odometry = new Odometry(new Header(Time.now(), "odom"), "base_link",
            new PoseWithCovariance(fromPose2d(odometryPose)),
            new TwistWithCovariance(fromChassisSpeeds(speeds))
        );
        publish("/odom", "nav_msgs/Odometry", odometry);
    }
    public void setGlobalPose(Pose2d globalPose)
    {
        PoseWithCovarianceStamped poseMessage = new PoseWithCovarianceStamped(new Header(Time.now(), "map"),
            new PoseWithCovariance(fromPose2d(globalPose)));
        publish("/global_set_pose", poseMessage.getMessageType(), poseMessage);
    }
    public void sendTargetPose(Pose2d targetPose)
    {
        PoseStamped poseMessage = new PoseStamped(new Header(Time.now(), "map"), fromPose2d(targetPose));
        publish("/target_pose", poseMessage.getMessageType(), poseMessage);
    }
    public ChassisSpeeds getTargetVelocity()
    {
        return toChassisSpeeds(targetVelocity);
    }
    @Override
    public void periodic()
    {
        super.periodic();
        if (isConnected())
        {
            publishOdometry(drive.getOdometryPose(), drive.getChassisSpeeds());
        }
    }
}