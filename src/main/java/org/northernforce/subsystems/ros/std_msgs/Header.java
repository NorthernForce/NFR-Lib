package org.northernforce.subsystems.ros.std_msgs;

import javax.json.Json;
import javax.json.JsonObject;

import edu.wpi.rail.jrosbridge.messages.Message;

/**
 * The std_msgs/Header message. Standard metadata for higher-level stamped data
 * types. This is generally used to communicate timestamped data in a particular
 * coordinate frame.
 * 
 * @author Russell Toris -- russell.toris@gmail.com
 * @version April 1, 2014
 */
public class Header extends Message {

	/**
	 * The name of the timestamp field for the message.
	 */
	public static final java.lang.String FIELD_STAMP = "stamp";

	/**
	 * The name of the frame ID field for the message.
	 */
	public static final java.lang.String FIELD_FRAME_ID = "frame_id";

	/**
	 * The message type.
	 */
	public static final java.lang.String TYPE = "std_msgs/Header";
	private final org.northernforce.subsystems.ros.primitives.Time stamp;
	private final java.lang.String frameID;

	/**
	 * Create a new Header with all empty values.
	 */
	public Header() {
		this(new org.northernforce.subsystems.ros.primitives.Time(), "");
	}

	/**
	 * Create a new Header with the given values.
	 * @param stamp
	 *            The timestamp.
	 * @param frameID
	 *            The frame ID.
	 */
	public Header(org.northernforce.subsystems.ros.primitives.Time stamp,
			java.lang.String frameID) {
		// build the JSON object
		super(Json.createObjectBuilder()
				.add(Header.FIELD_STAMP, stamp.toJsonObject())
				.add(Header.FIELD_FRAME_ID, frameID).build(), Header.TYPE);
		this.stamp = stamp;
		this.frameID = frameID;
	}

	/**
	 * Get the timestamp value of this header.
	 * 
	 * @return The timestamp value of this header.
	 */
	public org.northernforce.subsystems.ros.primitives.Time getStamp() {
		return this.stamp;
	}

	/**
	 * Get the frame ID value of this header.
	 * 
	 * @return The frame ID value of this header.
	 */
	public java.lang.String getFrameID() {
		return this.frameID;
	}

	/**
	 * Create a clone of this Header.
	 */
	@Override
	public Header clone() {
		// time primitives are mutable, create a clone
		return new Header(this.stamp.clone(), this.frameID);
	}

	/**
	 * Create a new Header based on the given JSON string. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonString
	 *            The JSON string to parse.
	 * @return A Header message based on the given JSON string.
	 */
	public static Header fromJsonString(java.lang.String jsonString) {
		// convert to a message
		return Header.fromMessage(new Message(jsonString));
	}

	/**
	 * Create a new Header based on the given Message. Any missing values will
	 * be set to their defaults.
	 * 
	 * @param m
	 *            The Message to parse.
	 * @return A Header message based on the given Message.
	 */
	public static Header fromMessage(Message m) {
		// get it from the JSON object
		return Header.fromJsonObject(m.toJsonObject());
	}

	/**
	 * Create a new Header based on the given JSON object. Any missing values
	 * will be set to their defaults.
	 * 
	 * @param jsonObject
	 *            The JSON object to parse.
	 * @return A Header message based on the given JSON object.
	 */
	public static Header fromJsonObject(JsonObject jsonObject) {
		org.northernforce.subsystems.ros.primitives.Time stamp = jsonObject
				.containsKey(Header.FIELD_STAMP) ? org.northernforce.subsystems.ros.primitives.Time
				.fromJsonObject(jsonObject.getJsonObject(Header.FIELD_STAMP))
				: new org.northernforce.subsystems.ros.primitives.Time();
		java.lang.String frameID = jsonObject
				.containsKey(Header.FIELD_FRAME_ID) ? jsonObject
				.getString(Header.FIELD_FRAME_ID) : "";
		return new Header(stamp, frameID);
	}
}