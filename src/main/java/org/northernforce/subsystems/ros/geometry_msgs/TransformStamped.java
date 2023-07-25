package org.northernforce.subsystems.ros.geometry_msgs;

import javax.json.Json;
import javax.json.JsonObject;

import org.northernforce.subsystems.ros.std_msgs.Header;

import edu.wpi.rail.jrosbridge.messages.Message;
import edu.wpi.rail.jrosbridge.messages.geometry.Transform;

public class TransformStamped extends Message
{
    public final Header header;
    public final Transform transform;
    public TransformStamped()
    {
        this(new Header(), new Transform());
    }
    public TransformStamped(Header header, Transform transform)
    {
        super(Json.createObjectBuilder()
            .add("header", header.toJsonObject())
            .add("transform", transform.toJsonObject())
            .build(), "geometry_msgs/TransformStamped");
        this.header = header;
        this.transform = transform;
    }
    @Override
    public TransformStamped clone()
    {
        return new TransformStamped(header, transform);
    }
    public static TransformStamped fromJsonString(String json)
    {
        return fromMessage(new Message(json));
    }
    public static TransformStamped fromMessage(Message message)
    {
        return fromJsonObject(message.toJsonObject());
    }
    public static TransformStamped fromJsonObject(JsonObject object)
    {
        Header header = object.containsKey("header") ? Header.fromJsonObject(object.getJsonObject("header"))
            : new Header();
        Transform transform = object.containsKey("transform") ?
            Transform.fromJsonObject(object.getJsonObject("transform")) : new Transform();
        return new TransformStamped(header, transform);
    }
}
