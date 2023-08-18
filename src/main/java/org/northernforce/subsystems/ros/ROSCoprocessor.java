package org.northernforce.subsystems.ros;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.Consumer;

import org.northernforce.subsystems.NFRSubsystem;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.rail.jrosbridge.Ros;
import edu.wpi.rail.jrosbridge.Service;
import edu.wpi.rail.jrosbridge.Topic;
import edu.wpi.rail.jrosbridge.callback.TopicCallback;
import edu.wpi.rail.jrosbridge.messages.Message;

public class ROSCoprocessor extends NFRSubsystem
{
    public static class ROSCoprocessorConfiguration extends NFRSubsystemConfiguration
    {
        protected String hostname;
        protected int port;
        public ROSCoprocessorConfiguration(String name)
        {
            super(name);
            this.hostname = name;
            this.port = 5810;
        }
        public ROSCoprocessorConfiguration(String name, String hostname, int port)
        {
            super(name);
            this.hostname = hostname;
            this.port = port;
        }
        public ROSCoprocessorConfiguration withHostname(String hostname)
        {
            this.hostname = hostname;
            return this;
        }
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
    public ROSCoprocessor(ROSCoprocessorConfiguration config)
    {
        super(config);
        ros = new Ros(config.hostname, config.port);
        topics = new HashMap<>();
        services = new HashMap<>();
        onConnects = new ArrayList<>();
        tryConnect = new Notifier(this::connect);
    }
    public void startConnecting()
    {
        tryConnect.startPeriodic(0.5);
    }
    public void onConnect(Runnable runnable)
    {
        onConnects.add(runnable);
    }
    public boolean isConnected()
    {
        return ros.isConnected();
    }
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
    public Service getService(String servicePath, String serviceType)
    {
        if (!services.containsKey(servicePath))
        {
            Service service = new Service(ros, servicePath, serviceType);
            services.put(servicePath, service);
        }
        return services.get(servicePath);
    }
    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addBooleanProperty("Connection", ros::isConnected, null);
    }
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
}