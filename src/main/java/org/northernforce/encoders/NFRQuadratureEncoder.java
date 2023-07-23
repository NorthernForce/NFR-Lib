package org.northernforce.encoders;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

/**
 * Hardware interface for the quadrature encoder.
 */
public class NFRQuadratureEncoder extends Encoder implements NFREncoder
{
    private double offset = 0;
    private EncoderSim sim;
    /**
     * Creates a new quadrature encoder.
     * @param sourceA the source for channel a.
     * @param sourceB the source for channel b.
     */
    public NFRQuadratureEncoder(DigitalSource sourceA, DigitalSource sourceB)
    {
        super(sourceA, sourceB);
        if (RobotBase.isSimulation())
        {
            sim = new EncoderSim(this);
        }
        else
        {
            sim = null;
        }
    }
    /**
     * Checks to see whether an encoder is present through various means. This may not be possible with all types of encoders
     * such as duty cycle encoders plugged into the rio, thus this may return true regardless. Please be aware of this.
     * @return true if the encoder is present, false if not present
     */
    @Override
    public boolean isPresent()
    {
        return true;
    }
    /**
     * Gets the position recorded by the sensor. This will return the relative position for any relative encoder,
     * and may return relative or absolute for an absolute encoder depending on how it is configured and implemented.
     * This value will be inverted as specified, and will be affected by the conversion factor.
     * @return Position affected by conversion factor, rotations by default.
     */
    @Override
    public double getPosition()
    {
        return getDistance() + offset;
    }
    /**
     * Sets the inversion of the encoder. This affects getVelocity and getPosition. Remember, however, this is all relative
     * to how the encoder is placed on the shaft.
     * @param inverted whether to invert the encoder readings
     */
    @Override
    public void setInverted(boolean inverted)
    {
        setReverseDirection(inverted);
    }
    /**
     * Gets the velocity recorded by the sensor. This will return the velocity measured by the sensor, either the relative part,
     * or absolute depending on configuration. The value will be inverted as specified, and will be affected by the conversion
     * factor.
     * @return Velocity affected by conversion factor, rotations per second by default.
     */
    @Override
    public double getVelocity()
    {
        return getRate();
    }
    /**
     * Sets the conversion factor that affects readings of the sensor. This is by default 1. 
     * @param factor the factor for measurements of velocity and position. 1 means 1 unit = 1 encoder rotation.
     */
    @Override
    public void setConversionFactor(double factor)
    {
        setDistancePerPulse(factor);
    }
    /**
     * Gets the conversion factor that affects readings of the sensor. This is by default 1. 
     * @return the factor for measurements of velocity and position. 1 means 1 unit = 1 encoder rotation.
     */
    @Override
    public double getConversionFactor()
    {
        return getDistancePerPulse();
    }
    /**
     * Resets the position of the encoder. Useful for relative applications.
     * @param position of the encoder which is affected by the conversion factor.
     */
    @Override
    public void setEncoderPosition(double position)
    {
        offset = position - getPosition();
    }
    /**
     * Sets the position using the simulation API. Use solely in simulation as it can lead to undefined behavior on
     * a robot.
     * @param position of the encoder which is affected by the conversion factor.
     */
    @Override
    public void setSimulationPosition(double position)
    {
        sim.setDistance(position);
    }
    /**
     * Adds to the position using the simulation API. Use solely in simulation as it can lead to undefined behavior on
     * a robot.
     * @param position of the encoder which is affected by the conversion factor.
     */
    @Override
    public void addSimulationPosition(double position)
    {
        sim.setDistance(position + sim.getDistance());
    }
    /**
     * Sets the velocity using the simulation API. Use solely in simulation as it can lead to undefined behavior on
     * a robot.
     * @param velocity of the encoder which is affected by the conversion factor.
     */
    @Override
    public void setSimulationVelocity(double velocity)
    {
        sim.setRate(velocity);
    }
}