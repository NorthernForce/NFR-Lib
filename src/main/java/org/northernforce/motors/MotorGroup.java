// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.northernforce.motors;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import org.northernforce.encoders.SmartEncoder;

/** 
 * Interface for motor controller that can be constructed with followers.
 * Also interfaces with encoders.
 */
public interface MotorGroup extends MotorController {
    public static class MotorEncoderMismatchException extends Exception
    {
        public MotorEncoderMismatchException(Class<? extends MotorGroup> motor, Class<? extends SmartEncoder> encoder)
        {
            super("Cannot link " + encoder.getName() + " to " + motor.getName());
        }
    }
    /**
     * Gives the rotations done by the encoder. Flips the value if the controller is inverted.
     * @return double value for full rotations of encoder
     */
    public double getEncoderRotations();
    /**
     * Gets the rotational velocity of the encoder. Flips the value if the controller is inverted.
     * @return double rotations per second
     */
    public double getEncoderRPS();
    /**
     * Sets a follower to have the opposite invert type as primary.
     * This must be called after followers are set.
     * @param index of the follower in parameter list.
     */
    public void setFollowerOppose(int i);
    /**
     * Resets the encoder rotations to (0,0)
     */
    public void resetEncoderRotations();
    /**
     * Resets the encoder position to a new position
     * @param rotations of the encoder in question
     */
    public void setEncoderRotations(double rotations);
    /**
     * Sets the velocity of a motor
     * @param velocity the velocity of the motor in rps
     */
    public void setVelocity(double velocity);
    /**
     * Sets the position of a motor using closed-loop control
     * @param position in encoder rotations
     */
    public void setPosition(double position);
    /**
     * Sets the position of a motor using advanced closed-loop control
     * @param position in encoder rotations
     */
    public void setSmartPosition(double position);
    /**
     * Configures the internal closed loop controller
     * @param slotIdx the index of the slot
     * @param kP proportion of PIDF
     * @param kI integral of PIDF
     * @param kD derivative of PIDF
     * @param kF documentation provides more information as to the use of kF
     * @param kMaxAcceleration Maximum acceleration
     * @param kMaxVelocity Maximum velocity
     * @param kAllowableError Maximum allowable error in encoder rotations
     */
    public void configureClosedLoop(int slotIdx, double kP, double kI, double kD, double kF, double kMaxAcceleration,
        double kMaxVelocity, double kAllowableError);
    /**
     * Gets the output voltage provided by the simulation instance of the motor. It will apply inversion as necessary.
     * Will not work if robot is not in simulation.
     * @return [-12.. 12] volts
     */
    public double getSimulationOutputVoltage();
    /**
     * Sets the position of the integrated encoder in simulation mode.
     * @param position number of rotations of the encoder
     */
    public void setSimulationPosition(double position);
    /**
     * Sets the velocity of the integrated encoder in simulation mode.
     * @param position number of rotations per second of the encoder
     */
    public void setSimulationVelocity(double velocity);
    /**
     * Adds to the current position in simulation mode.
     * @param deltaPosition difference of position in encoder rotations
     */
    public void addSimulationPosition(double deltaPosition);
    /**
     * Gets the simulation instance/constants for the type of motor.
     * @return the simulation instance for the type of motor
     */
    public DCMotor getGearbox();
    /**
     * Sets the forward soft limit.
     * @param rotations number of rotations to limit the forward soft limit.
     */
    public void setForwardSoftLimit(double rotations);
    /**
     * Sets the reverse soft limit.
     * @param rotations number of rotations to limit the reverse soft limit.
     */
    public void setReverseSoftLimit(double rotations);
    /**
     * Disables the forward soft limit.
     */
    public void disableForwardSoftLimit();
    /**
     * Disables the reverse soft limit
     */
    public void disableReverseSoftLimit();
    /**
     * Links this encoder as the default encoder if possible.
     * @param encoder the encoder
     */
    public void linkEncoder(SmartEncoder encoder) throws MotorEncoderMismatchException;
    /**
     * Returns a new encoder that uses the integrated position.
     * @return
     */
    public SmartEncoder getIntegratedEncoder();
    /**
     * Changes that status of continuous input. This helps when you link an absolute encoder to a motor.
     * @param enable whether to enable continuous input or not
     */
    public void enableContinuousInput(boolean enable);
}