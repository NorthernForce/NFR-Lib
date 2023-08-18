package org.northernforce.subsystems.ros;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Consumer;

import org.northernforce.subsystems.NFRSubsystem;
import org.northernforce.subsystems.ros.geometry_msgs.PoseStamped;
import org.northernforce.subsystems.ros.geometry_msgs.PoseWithCovarianceStamped;
import org.northernforce.subsystems.ros.geometry_msgs.TransformStamped;
import org.northernforce.subsystems.ros.nav_msgs.Odometry;
import org.northernforce.subsystems.ros.nfr_tf_bridge_msgs.LookupTransform;
import org.northernforce.subsystems.ros.nfr_tf_bridge_msgs.RequestTransform;
import org.northernforce.subsystems.ros.primitives.Time;
import org.northernforce.subsystems.ros.rosgraph_msgs.Clock;
import org.northernforce.subsystems.ros.std_msgs.Header;
import org.northernforce.subsystems.ros.primitives.Time;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Service;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;
import edu.wpi.rail.jrosbridge.messages.geometry.PoseWithCovariance;
import edu.wpi.rail.jrosbridge.messages.geometry.Quaternion;
import edu.wpi.rail.jrosbridge.messages.geometry.Twist;
import edu.wpi.rail.jrosbridge.messages.geometry.TwistWithCovariance;
import edu.wpi.rail.jrosbridge.messages.geometry.Vector3;

/**
 * Team 172's implementation of the ROSCoprocessor subsystem which maintains a ROSBridge websocket between the
 * coprocessor and the subsystem.
 */
public class ROSCoprocessor extends NFRSubsystem
{
    /**
     * The configuration for the ROSCoprocessor.
     */
    public static class ROSCoprocessorConfiguration extends NFRSubsystemConfiguration
    {
        protected String hostname;
        protected int port;
        /**
         * Creates a new ROSCoprocessorConfiguration.
         * @param name the name of the subsystem
         */
        public ROSCoprocessorConfiguration(String name)
        {
            super(name);
            this.hostname = name;
            this.port = 5810;
        }
        /**
         * Creates a new ROSCoprocessorConfiguration
         * @param name the name of the subsystem.
         * @param hostname the hostname of the coprocessor
         * @param port the port to attach to.
         */
        public ROSCoprocessorConfiguration(String name, String hostname, int port)
        {
            super(name);
            this.hostname = hostname;
            this.port = port;
        }
        /**
         * With hostname
         * @param hostname the hostname of the coprocessor.
         * @return this
         */
        public ROSCoprocessorConfiguration withHostname(String hostname)
        {
            this.hostname = hostname;
            return this;
        }
        /**
         * With port
         * @param port the port to attach to.
         * @return this
         */
        public ROSCoprocessorConfiguration withPort(int port)
        {
            this.port = port;
            return this;
        }
    }
    protected final Ros ros;
    protected final HashMap<String, Topic> topics;
    protected final HashMap<String, Service> services;
    protected final ArrayList<Runnable> onConnects;
    protected final Notifier tryConnect;
    /**
     * Creates a new ROSCoprocessor.
     * @param config the configuration for the coprocessor
     */
    public ROSCoprocessor(ROSCoprocessorConfiguration config)
    {
        super(config);
        ros = new Ros(config.hostname, config.port);
        topics = new HashMap<>();
        services = new HashMap<>();
        onConnects = new ArrayList<>();
        tryConnect = new Notifier(this::connect);
    }
    /**
     * Starts a notifier to try to connect every 0.5 seconds
     */
    public void startConnecting()
    {
        tryConnect.startPeriodic(0.5);
    }
    /**
     * Adds a runnable to be executed on connect
     * @param runnable to be executed on connect
     */
    public void onConnect(Runnable runnable)
    {
        onConnects.add(runnable);
    }
    /**
     * Checks whether connected
     * @return if connected to the coprocessor via the ros bridge websocket.
     */
    public boolean isConnected()
    {
        return ros.isConnected();
    }
    /**
     * Subscribes to a topic
     * @param topicName the topic name/path
     * @param topicType the topic type
     * @param messageConsumer the message consumer for when a topic is recieved
     */
    public void subscribe(String topicName, String topicType, Consumer<Message> messageConsumer)
    {
        if (!topics.containsKey(topicName))
        {
            Topic topic = new Topic(ros, topicName, topicType);
            topics.put(topicName, topic);
        }
        topics.get(topicName).subscribe(new TopicCallback()
        {
            @Override
            public void handleMessage(Message message)
            {
                messageConsumer.accept(message);
            }
        });
    }
    /**
     * Publishes to a topic
     * @param topicName the topic name/path
     * @param topicType the topic type
     * @param message the message to publish
     */
    public void publish(String topicName, String topicType, Message message)
    {
        if (!topics.containsKey(topicName))
        {
            Topic topic = new Topic(ros, topicName, topicType);
            topic.advertise();
            topics.put(topicName, topic);
        }
        if (!topics.get(topicName).isAdvertised())
        {
            topics.get(topicName).advertise();
        }
        topics.get(topicName).publish(message);
    }
    /**
     * Gets the service reference for a given path
     * @param servicePath the path to the service
     * @param serviceType the type of the service
     * @return the service reference
     */
    public Service getService(String servicePath, String serviceType)
    {
        if (!services.containsKey(servicePath))
        {
            Service service = new Service(ros, servicePath, serviceType);
            services.put(servicePath, service);
        }
        return services.get(servicePath);
    }
    /**
     * Initializes the sendable data
     * @param builder the builder to add data to
     */
    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addBooleanProperty("Connection", ros::isConnected, null);
    }
    /**
     * Tries to connect to the ros coprocessor
     */
    public void connect()
    {
        if (ros.connect())
        {
            System.out.println("Connected");
            for (var onConnect : onConnects)
            {
                onConnect.run();
            }
            tryConnect.stop();
        }
        else
        {
            System.out.println("Could not connect to rosbridge");
        }
    }
    @Override
    public void periodic()
    {
        if (ros.isConnected())
        {
            Clock clock = new Clock(new Time((double)System.currentTimeMillis() / 1000));
            publish("/clock", "rosgraph_msgs/Clock", clock);
        }
    }
    public void publishOdometry(Pose2d odometryPose, ChassisSpeeds chassisSpeeds)
    {
        Header header = new Header(new Time((double)System.currentTimeMillis() / 1000), "odom");
        Rotation3d odometryOrientation = new Rotation3d(0, 0, odometryPose.getRotation().getRadians());
        Quaternion quaternion = new Quaternion(
            odometryOrientation.getQuaternion().getX(),
            odometryOrientation.getQuaternion().getY(),
            odometryOrientation.getQuaternion().getZ(),
            odometryOrientation.getQuaternion().getW()
        );
        PoseWithCovariance pose = new PoseWithCovariance(
            new Pose(new Point(odometryPose.getX(), odometryPose.getY(), 0), quaternion));
        TwistWithCovariance twist = new TwistWithCovariance(new Twist(
            new Vector3(
                chassisSpeeds.vxMetersPerSecond,
                chassisSpeeds.vyMetersPerSecond,
                0
            ),
            new Vector3(
                0,
                0,
                chassisSpeeds.omegaRadiansPerSecond
            )
        ));
        Odometry odometry = new Odometry(header, "base_link", pose, twist);
        publish("/odom", "nav_msgs/Odometry", odometry);
    }
    public void publishSetPose(Pose2d pose)
    {
        Header header = new Header(new Time((double)System.currentTimeMillis() / 1000), "map");
        Rotation3d rotation = new Rotation3d(0, 0, pose.getRotation().getRadians());
        PoseWithCovariance poseWithCovariance = new PoseWithCovariance(new Pose(
            new Point(pose.getX(), pose.getY(), 0),
            new Quaternion(
                rotation.getQuaternion().getX(),
                rotation.getQuaternion().getY(),
                rotation.getQuaternion().getZ(),
                rotation.getQuaternion().getW()
            )
        ));
        PoseWithCovarianceStamped poseStamped = new PoseWithCovarianceStamped(header, poseWithCovariance);
        publish("/set_pose", "geometry_msgs/PoseWithCovarianceStamped", poseStamped);
    }
    public void publishTargetPose(Pose2d targetPose)
    {
        Header header = new Header(new Time(Timer.getFPGATimestamp()), "map");
        Rotation3d rotation = new Rotation3d(0, 0, targetPose.getRotation().getRadians());
        Pose pose = new Pose(
            new Point(targetPose.getX(), targetPose.getY(), 0),
            new Quaternion(
                rotation.getQuaternion().getX(),
                rotation.getQuaternion().getY(),
                rotation.getQuaternion().getZ(),
                rotation.getQuaternion().getW()
            )
        );
        PoseStamped poseStamped = new PoseStamped(header, pose);
        publish("/target_pose", "geometry_msgs/PoseStamped", poseStamped);
    }
    public void subscribeCommandVelocity(Consumer<ChassisSpeeds> commmandVelocityConsumer)
    {
        subscribe("/cmd_vel", "geometry_msgs/Twist", message -> {
            Twist twist = Twist.fromMessage(message);
            ChassisSpeeds speeds = new ChassisSpeeds();
            speeds.vxMetersPerSecond = twist.getLinear().getX();
            speeds.vyMetersPerSecond = twist.getLinear().getY();
            speeds.omegaRadiansPerSecond = twist.getAngular().getZ();
            commmandVelocityConsumer.accept(speeds);
        });
    }
    public Optional<Transform3d> getTransform(String baseFrame, String childFrame)
    {
        var serviceResponse = getService("/lookup_transform", "nfr_tf_bridge_msgs/LookupTransform")
            .callServiceAndWait(new LookupTransform.Request(baseFrame, childFrame, new Time()));
        var response = LookupTransform.Response.fromServiceResponse(serviceResponse);
        if (response.success)
        {
            return Optional.of(new Transform3d(
                new Translation3d(
                    response.transform.transform.getTranslation().getX(),
                    response.transform.transform.getTranslation().getY(),
                    response.transform.transform.getTranslation().getZ()
                ),
                new Rotation3d(
                    new edu.wpi.first.math.geometry.Quaternion(
                        response.transform.transform.getRotation().getW(),
                        response.transform.transform.getRotation().getX(),
                        response.transform.transform.getRotation().getY(),
                        response.transform.transform.getRotation().getZ()
                    )
                )
            ));
        }
        else
        {
            return Optional.empty();
        }
    }
    public void subscribeTransform(String base_frame, String child_frame, String topic, double frequency,
        Consumer<Transform3d> transformConsumer)
    {
        publish("request_transform", "nfr_tf_bridge_msgs/RequestTransform",
            new RequestTransform(frequency, base_frame, child_frame, topic));
        subscribe(topic, "geometry_msgs/TransformStamped", message -> {
            TransformStamped transform = TransformStamped.fromMessage(message);
            transformConsumer.accept(new Transform3d(
                new Translation3d(
                    transform.transform.getTranslation().getX(),
                    transform.transform.getTranslation().getY(),
                    transform.transform.getTranslation().getZ()
                ),
                new Rotation3d(
                    new edu.wpi.first.math.geometry.Quaternion(
                        transform.transform.getRotation().getW(),
                        transform.transform.getRotation().getX(),
                        transform.transform.getRotation().getY(),
                        transform.transform.getRotation().getZ()
                    )
                )
            ));
        });
    }
}