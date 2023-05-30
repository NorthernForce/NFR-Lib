package org.northernforce.encoders;

public interface SmartEncoder {
    /**
     * Gets the amount of encoder rotations recorded by the encoder.
     * @return encoder rotations
     */
    public double getEncoderRotations();
    /**
     * Gets the amount of encoder rotations per second if the encoder supports it, or else it returns 0.
     * @return encoder rotations per second
     */
    public double getEncoderRPS();
    /**
     * Sets the amount of rotations in the simulation mode.
     * @param rotations of the encoder.
     */
    public void setSimulationRotations(double rotations);
    /**
     * Sets the amount of rotations per second in the simulation mode.
     * @param rps of the encoder.
     */
    public void setSimulationRPS(double rps);
    /**
     * Adds to the amount of rotations in the simulation mode.
     * @param rotations of the encoder.
     */
    public void addSimulationRotations(double rotations);
    /**
     * Resets the encoder rotations to a set amount of rotations. Note: this does not involve simulation.
     * @param rotations of the encoder.
     */
    public void resetEncoderRotations(double rotations);
    /**
     * Returns whether the encoder is based on the roborio data ports, or is on the CAN bus somewhere.
     * @return a boolean of whether it is based on the roborio.
     */
    public boolean isOnRoborio();
}
