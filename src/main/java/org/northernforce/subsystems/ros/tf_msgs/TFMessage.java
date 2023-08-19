package org.northernforce.subsystems.ros.tf_msgs;

import java.io.StringReader;
import java.util.Arrays;

import javax.json.Json;
import javax.json.JsonArray;
import javax.json.JsonObject;

import org.northernforce.subsystems.ros.geometry_msgs.TransformStamped;

import edu.wpi.rail.jrosbridge.messages.Message;

public class TFMessage extends Message
{
    public final TransformStamped[] transforms;
    public TFMessage()
    {
        this(new TransformStamped[0]);
    }
    public TFMessage(TransformStamped[] transforms)
    {
        super(Json.createObjectBuilder()
            .add("transforms", Json.createReader(
                new StringReader(Arrays.deepToString(transforms)))
                .readArray())
            .build(), "tf_msgs/TFMessage");
        this.transforms = transforms;
    }
    public static TFMessage fromJsonString(String json)
    {
        return fromMessage(new Message(json));
    }
    public static TFMessage fromMessage(Message message)
    {
        return fromJsonObject(message.toJsonObject());
    }
    public static TFMessage fromJsonObject(JsonObject object)
    {
        TransformStamped[] transforms;
        if (object.containsKey("transforms"))
        {
            JsonArray array = object.getJsonArray("transforms");
            transforms = new TransformStamped[array.size()];
            for (int i = 0; i < array.size(); i++)
            {
                transforms[i] = TransformStamped.fromJsonObject(array.getJsonObject(i));
            }
        }
        else
        {
            transforms = new TransformStamped[0];
        }
        return new TFMessage(transforms);
    }
}