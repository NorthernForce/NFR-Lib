package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.function.Consumer;

import org.northernforce.subsystems.NFRSubsystem;
import org.northernforce.subsystems.drive.NFRDrive;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

public class Xavier extends NFRSubsystem
{
    protected final NetworkTable table, targetPoseTable, odometryTable, imuTable, globalSetPoseTable, cmdVelTable, poseTable;
    protected final DoublePublisher targetPoseX;
    protected final DoublePublisher targetPoseY;
    protected final DoublePublisher targetPoseTheta;
    protected final IntegerPublisher targetPoseStamp;
    protected final DoublePublisher odometryDeltaX;
    protected final DoublePublisher odometryDeltaY;
    protected final BooleanPublisher cancelTargetPose;
    protected final IntegerPublisher odometryPoseStamp;
    protected final DoubleSubscriber cmdVelX, cmdVelY, cmdVelTheta, poseX, poseY, poseTheta;
    protected final DoublePublisher globalSetPoseX, globalSetPoseY, globalSetPoseTheta;
    protected final IntegerPublisher globalSetPoseStamp;
    protected final DoublePublisher imuTheta;
    protected final IntegerPublisher imuStamp;
    protected final NetworkTable realsenseTable;
    protected final DoubleSubscriber realsenseX, realsenseY, realsenseZ, realsenseQX, realsenseQY, realsenseQZ, realsenseQW;
    protected final IntegerSubscriber realsenseStamp;
    protected final NFRDrive drive;
    protected final Consumer<Pair<Double, Pose2d>> realsenseEstimates;
    protected volatile boolean xavierIsConnected;
    public static class XavierConfiguration extends NFRSubsystemConfiguration
    {
        public XavierConfiguration()
        {
            super("xavier");
        }
    }
    public Xavier(NFRDrive drive, Consumer<Pair<Double, Pose2d>> realsenseEstimates)
    {
        super(new XavierConfiguration());
        table = NetworkTableInstance.getDefault().getTable("xavier");
        NetworkTableInstance.getDefault().startServer();
        targetPoseTable = table.getSubTable("target_pose");
        targetPoseX = targetPoseTable.getDoubleTopic("x").publish();
        targetPoseY = targetPoseTable.getDoubleTopic("y").publish();
        targetPoseTheta = targetPoseTable.getDoubleTopic("theta").publish();
        targetPoseStamp = targetPoseTable.getIntegerTopic("stamp").publish();
        cancelTargetPose = targetPoseTable.getBooleanTopic("cancel").publish();
        odometryTable = table.getSubTable("odometry");
        odometryDeltaX = odometryTable.getDoubleTopic("vx").publish();
        odometryDeltaY = odometryTable.getDoubleTopic("vy").publish();
        odometryPoseStamp = odometryTable.getIntegerTopic("stamp").publish();
        imuTable = table.getSubTable("imu");
        imuTheta = imuTable.getDoubleTopic("theta").publish();
        imuStamp = imuTable.getIntegerTopic("stamp").publish();
        globalSetPoseTable = table.getSubTable("global_set_pose");
        globalSetPoseX = globalSetPoseTable.getDoubleTopic("x").publish();
        globalSetPoseY = globalSetPoseTable.getDoubleTopic("y").publish();
        globalSetPoseTheta = globalSetPoseTable.getDoubleTopic("theta").publish();
        globalSetPoseStamp = globalSetPoseTable.getIntegerTopic("stamp").publish();
        cmdVelTable = table.getSubTable("cmd_vel");
        cmdVelX = cmdVelTable.getDoubleTopic("x").subscribe(0);
        cmdVelY = cmdVelTable.getDoubleTopic("y").subscribe(0);
        cmdVelTheta = cmdVelTable.getDoubleTopic("theta").subscribe(0);
        poseTable = table.getSubTable("pose");
        poseX = poseTable.getDoubleTopic("x").subscribe(0);
        poseY = poseTable.getDoubleTopic("y").subscribe(0);
        poseTheta = poseTable.getDoubleTopic("theta").subscribe(0);
        realsenseTable = table.getSubTable("realsense");
        realsenseX = realsenseTable.getDoubleTopic("x").subscribe(0.0);
        realsenseY = realsenseTable.getDoubleTopic("x").subscribe(0.0);
        realsenseZ = realsenseTable.getDoubleTopic("x").subscribe(0.0);
        realsenseQX = realsenseTable.getDoubleTopic("qx").subscribe(0.0);
        realsenseQY = realsenseTable.getDoubleTopic("qy").subscribe(0.0);
        realsenseQZ = realsenseTable.getDoubleTopic("qz").subscribe(0.0);
        realsenseQW = realsenseTable.getDoubleTopic("qw").subscribe(0.0);
        realsenseStamp = realsenseTable.getIntegerTopic("stamp").subscribe(0);
        this.realsenseEstimates = realsenseEstimates;
        NetworkTableInstance.getDefault().addListener(realsenseStamp, EnumSet.of(Kind.kValueAll), this::realsenseCallback);
        this.drive = drive;
        xavierIsConnected = false;
    }
    public void publishOdometry(ChassisSpeeds speeds, Rotation2d imuAngle)
    { 
        odometryDeltaX.set(speeds.vxMetersPerSecond);
        odometryDeltaY.set(speeds.vyMetersPerSecond);
        odometryPoseStamp.set((long)(Timer.getFPGATimestamp() * 1e9));
        imuTheta.set(imuAngle.getRadians());
        imuStamp.set((long)(Timer.getFPGATimestamp() * 1e9));
    }
    public void setGlobalPose(Pose2d globalPose)
    {
        globalSetPoseX.set(globalPose.getX());
        globalSetPoseY.set(globalPose.getY());
        globalSetPoseTheta.set(globalPose.getRotation().getRadians());
        globalSetPoseStamp.set((long)(Timer.getFPGATimestamp() * 1e9));
    }
    public void sendTargetPose(Pose2d targetPose)
    {
        targetPoseX.set(targetPose.getX());
        targetPoseY.set(targetPose.getY());
        targetPoseTheta.set(targetPose.getRotation().getRadians());
        targetPoseStamp.set((long)(Timer.getFPGATimestamp() * 1e9));
    }
    public void cancelTargetPose()
    {
        cancelTargetPose.set(true);
    }
    public ChassisSpeeds getTargetVelocity()
    {
        return new ChassisSpeeds(cmdVelX.get(), cmdVelY.get(), cmdVelTheta.get());
    }
    public Pose2d getPose()
    {
        return new Pose2d(poseX.get(), poseY.get(), Rotation2d.fromRadians(poseTheta.get()));
    }
    public void realsenseCallback(NetworkTableEvent event)
    {
        if (realsenseEstimates != null)
        {
            double timestamp = realsenseStamp.get();
            Pose3d pose = new Pose3d(realsenseX.get(), realsenseY.get(), realsenseZ.get(), new Rotation3d(new Quaternion(
                realsenseQX.get(), realsenseQY.get(), realsenseQZ.get(), realsenseQW.get()
            )));
            realsenseEstimates.accept(Pair.of(timestamp, pose.toPose2d()));
        }
    }
    public boolean isConnected()
    {
        boolean flag = false;
        for (var connection : NetworkTableInstance.getDefault().getConnections())
        {
            if (connection.remote_ip.equals("10.1.72.47") || connection.remote_id.indexOf("xavier") != -1)
            {
                flag = true;
                break;
            }
        }
        return flag;
    }
    @Override
    public void periodic()
    {
        super.periodic();
        if (isConnected())
        {
            publishOdometry(drive.getChassisSpeeds(), drive.getFieldRelativeRotation().minus(Rotation2d.fromDegrees(180)));
        }
    }
    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addBooleanProperty("isConnected", this::isConnected, null);
    }
}