package frc.robot.subsystems;

import org.northernforce.subsystems.NFRSubsystem;
import org.northernforce.subsystems.drive.NFRDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;

public class Xavier extends NFRSubsystem
{
    protected final NetworkTable table, targetPoseTable, odometryTable, globalSetPoseTable, cmdVelTable;
    protected final DoublePublisher targetPoseX;
    protected final DoublePublisher targetPoseY;
    protected final DoublePublisher targetPoseTheta;
    protected final IntegerPublisher targetPoseStamp;
    protected final DoublePublisher odometryDeltaX;
    protected final DoublePublisher odometryDeltaY;
    protected final DoublePublisher odometryDeltaTheta;
    protected final BooleanPublisher cancelTargetPose;
    protected final DoublePublisher odometryPoseX;
    protected final DoublePublisher odometryPoseY;
    protected final DoublePublisher odometryPoseTheta;
    protected final IntegerPublisher odometryPoseStamp;
    protected final DoubleSubscriber cmdVelX, cmdVelY, cmdVelTheta;
    protected final DoublePublisher globalSetPoseX, globalSetPoseY, globalSetPoseTheta;
    protected final IntegerPublisher globalSetPoseStamp;
    protected final NFRDrive drive;
    protected volatile boolean xavierIsConnected;
    public static class XavierConfiguration extends NFRSubsystemConfiguration
    {
        public XavierConfiguration()
        {
            super("xavier");
        }
    }
    public Xavier(NFRDrive drive)
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
        odometryPoseX = odometryTable.getDoubleTopic("x").publish();
        odometryPoseY = odometryTable.getDoubleTopic("y").publish();
        odometryPoseTheta = odometryTable.getDoubleTopic("theta").publish();
        odometryDeltaX = odometryTable.getDoubleTopic("dx").publish();
        odometryDeltaY = odometryTable.getDoubleTopic("dy").publish();
        odometryDeltaTheta = odometryTable.getDoubleTopic("dtheta").publish();
        odometryPoseStamp = odometryTable.getIntegerTopic("stamp").publish();
        globalSetPoseTable = table.getSubTable("global_set_pose");
        globalSetPoseX = globalSetPoseTable.getDoubleTopic("x").publish();
        globalSetPoseY = globalSetPoseTable.getDoubleTopic("y").publish();
        globalSetPoseTheta = globalSetPoseTable.getDoubleTopic("theta").publish();
        globalSetPoseStamp = globalSetPoseTable.getIntegerTopic("stamp").publish();
        cmdVelTable = table.getSubTable("cmd_vel");
        cmdVelX = cmdVelTable.getDoubleTopic("x").subscribe(0);
        cmdVelY = cmdVelTable.getDoubleTopic("y").subscribe(0);
        cmdVelTheta = cmdVelTable.getDoubleTopic("theta").subscribe(0);
        this.drive = drive;
        xavierIsConnected = false;
        NetworkTableInstance.getDefault().addConnectionListener(true, this::connectionCallback);
    }
    public void publishOdometry(Pose2d odometryPose, ChassisSpeeds speeds)
    { 
        odometryPoseX.set(odometryPose.getX());
        odometryPoseY.set(odometryPose.getY());
        odometryPoseTheta.set(odometryPose.getRotation().getRadians());
        odometryDeltaX.set(speeds.vxMetersPerSecond);
        odometryDeltaY.set(speeds.vyMetersPerSecond);
        odometryDeltaTheta.set(speeds.omegaRadiansPerSecond);
        odometryPoseStamp.set((long)(Timer.getFPGATimestamp() * 1e9));
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
        globalSetPoseStamp.set((long)(Timer.getFPGATimestamp() * 1e9));
    }
    public void cancelTargetPose()
    {
        cancelTargetPose.set(true);
    }
    public ChassisSpeeds getTargetVelocity()
    {
        return new ChassisSpeeds(cmdVelX.get(), cmdVelY.get(), cmdVelTheta.get());
    }
    public void connectionCallback(NetworkTableEvent event)
    {
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
            publishOdometry(drive.getOdometryPose(), drive.getChassisSpeeds());
        }
    }
    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addBooleanProperty("isConnected", this::isConnected, null);
    }
}