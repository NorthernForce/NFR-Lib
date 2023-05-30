package org.northernforce.encoders;

public interface SmartAbsoluteEncoder extends SmartEncoder {
    /**
     * Gets the current absolute position
     * @return position in rotations of encoder
     */
    public double getAbsoluteEncoderRotations();
    /**
     * Gets the absolute encoder offset
     * @return offset in rotations of encoder
     */
    public double getAbsoluteEncoderOffsetRotations();
    /**
     * Sets the absolute encoder offset
     * @param offset in rotations of encoder
     */
    public void setAbsoluteEncoderOffsetRotations(double offset);
    /**
     * Resets the absolute encoder encoder offset to a new position
     * @param newPos the new position in rotations of encoder
     */
    public default void resetAbsoluteEncoderOffset(double newPos)
    {
        setAbsoluteEncoderOffsetRotations(newPos - getAbsoluteEncoderRotations() + getAbsoluteEncoderOffsetRotations());
    }
}
