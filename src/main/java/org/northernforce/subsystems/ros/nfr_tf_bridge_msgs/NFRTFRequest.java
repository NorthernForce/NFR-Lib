package org.northernforce.subsystems.ros.nfr_tf_bridge_msgs;

import javax.json.Json;
import javax.json.JsonObject;

import edu.wpi.rail.jrosbridge.messages.Message;

public class NFRTFRequest extends Message
{
    public final String source_frame;
    public final String target_frame;
    public final String publish_topic;
    public final double update_rate;
    public NFRTFRequest()
    {
        this("", "", "", 0);
    }
    public NFRTFRequest(String source_frame, String target_frame, String publish_topic, double update_rate)
    {
        super(Json.createObjectBuilder()
            .add("source_frame", source_frame)
            .add("target_frame", target_frame)
            .add("publish_topic", publish_topic)
            .add("update_rate", update_rate)
            .build(), "nfr_tf_bridge_msgs/NFRTFRequest");
        this.source_frame = source_frame;
        this.target_frame = target_frame;
        this.publish_topic = publish_topic;
        this.update_rate = update_rate;
    }
    @Override
    public NFRTFRequest clone()
    {
        return new NFRTFRequest(source_frame, target_frame, publish_topic, update_rate);
    }
    public static NFRTFRequest fromJsonString(String json)
    {
        return fromMessage(new Message(json));
    }
    public static NFRTFRequest fromMessage(Message message)
    {
        return fromJsonObject(message.toJsonObject());
    }
    public static NFRTFRequest fromJsonObject(JsonObject object)
    {
        String source_frame = object.containsKey("source_frame") ? object.getString("source_frame") : "";
        String target_frame = object.containsKey("target_frame") ? object.getString("target_frame") : "";
        String publish_topic = object.containsKey("publish_topic") ? object.getString("publish_topic") : "";
        double update_rate = object.containsKey("update_rate") ? object.getJsonNumber("update_rate").doubleValue() : 0;
        return new NFRTFRequest(source_frame, target_frame, publish_topic, update_rate);
    }
}
