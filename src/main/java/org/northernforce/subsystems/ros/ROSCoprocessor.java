package org.northernforce.subsystems.ros;

import java.util.HashMap;
import java.util.function.Consumer;

import org.northernforce.subsystems.NFRSubsystem;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.rail.jrosbridge.Ros;
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
    public ROSCoprocessor(ROSCoprocessorConfiguration config)
    {
        super(config);
        ros = new Ros(config.hostname, config.port);
        ros.connect();
        topics = new HashMap<>();
    }
    public void subscribe(String topicName, String topicType, Consumer<Message> messageConsumer)
    {
        Topic topic = new Topic(ros, topicName, topicType);
        topic.subscribe(new TopicCallback()
        {
            @Override
            public void handleMessage(Message message)
            {
                messageConsumer.accept(message);
            }
        });
        topics.put(topic.getName(), topic);
    }
    public void publish(String topicName, String topicType, Message message)
    {
        if (!topics.containsKey(topicName))
        {
            Topic topic = new Topic(ros, topicName, topicType);
            topic.advertise();
            topics.put(topicName, topic);
        }
        topics.get(topicName).publish(message);
    }
    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addBooleanProperty("Connection", ros::isConnected, null);
    }
}