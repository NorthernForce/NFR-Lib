package org.northernforce.subsystems.ros;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

import javax.websocket.Session;

import org.northernforce.subsystems.NFRSubsystem;
import org.northernforce.subsystems.ros.rosgraph_msgs.Clock;
import org.northernforce.subsystems.ros.primitives.Time;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Service;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.handler.RosHandler;
import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Point;
import edu.wpi.rail.jrosbridge.messages.geometry.Pose;
import edu.wpi.rail.jrosbridge.messages.geometry.Quaternion;
import edu.wpi.rail.jrosbridge.messages.geometry.Twist;
import edu.wpi.rail.jrosbridge.messages.geometry.Vector3;

/**
 * Team 172's implementation of the ROSCoprocessor subsystem which maintains a ROSBridge websocket between the
 * coprocessor and the subsystem.
 */
public class ROSCoprocessor extends NFRSubsystem implements RosHandler
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
    protected final ArrayList<Runnable> onConnects, onDisconnects;
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
        onDisconnects = new ArrayList<>();
        tryConnect = new Notifier(this::connect);
        ros.addRosHandler(this);
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
     * Adds a runnable to be executed on disconnect
     * @param runnable to be executed on disconnect
     */
    public void onDisconnect(Runnable runnable)
    {
        onDisconnects.add(runnable);
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
    public static Pose fromPose3d(Pose3d pose)
    {
        return new Pose(
            new Point(pose.getX(), pose.getY(), pose.getZ()),
            new Quaternion(
                pose.getRotation().getQuaternion().getX(),
                pose.getRotation().getQuaternion().getY(),
                pose.getRotation().getQuaternion().getZ(),
                pose.getRotation().getQuaternion().getW()
            )
        );
    }
    public static Pose fromPose2d(Pose2d pose)
    {
        return fromPose3d(new Pose3d(pose));
    }
    public static Twist fromChassisSpeeds(ChassisSpeeds speeds)
    {
        return new Twist(
            new Vector3(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0),
            new Vector3(0, 0, speeds.omegaRadiansPerSecond)
        );
    }
    public static ChassisSpeeds toChassisSpeeds(Twist twist)
    {
        return new ChassisSpeeds(twist.getLinear().getX(), twist.getLinear().getY(), twist.getAngular().getZ());
    }
    @Override
    public void handleConnection(Session session)
    {
    }
    @Override
    public void handleDisconnection(Session session)
    {
        DriverStation.reportWarning("Disconnected from ROS coprocessor", true);
        for (Runnable runnable : onDisconnects)
        {
            runnable.run();
        }
    }
    @Override
    public void handleError(Session session, Throwable t)
    {
        DriverStation.reportError(t.getMessage(), true);
    }
}