package org.northernforce.motors;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.northernforce.encoders.NFRAbsoluteEncoder;
import org.northernforce.encoders.NFREncoder;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * This is a class designed for interfacing with a group of TalonFX's. This uses Phoenix 6 API.
 */
public class NFRTalonFX extends TalonFX implements NFRMotorController {
    /**
     * This class is the hardware interface for the integrated encoder. Uses Phoenix 6 API.
     */
    public class IntegratedEncoder implements NFREncoder
    {
        private final StatusSignal<Double> rotorPositionSignal, rotorVelocitySignal;
        private double conversionFactor;
        /**
         * Creates a new IntegratedEncoder. Should be called internally.
         */
        private IntegratedEncoder()
        {
            this.rotorPositionSignal = getRotorPosition();
            this.rotorVelocitySignal = getRotorVelocity();
            conversionFactor = 1;
        }
        /**
         * Checks to see if encoder is present by checking to see whether motor is present.
         */
        @Override
        public boolean isPresent()
        {
            return NFRTalonFX.this.isPresent();
        }
        /**
         * Gets the position of the integrated encoder. Factors in conversion factor and inversion.
         */
        @Override
        public double getPosition()
        {
            rotorVelocitySignal.refresh();
            return rotorPositionSignal.getValue() * conversionFactor;
        }
        /**
         * Sets the inversion of the sensor. Check to see whether the sensor is inverted
         * @param inverted whether or not the sensor is inverted.
         */
        @Override
        public void setInverted(boolean inverted)
        {
            conversionFactor = Math.abs(conversionFactor) * (inverted ? -1 : 1);
        }
        /**
         * Gets the velocity of the motor. Factors in conversion factor.
         * @return the velocit of the motor's integrated encoder
         */
        @Override
        public double getVelocity()
        {
            rotorVelocitySignal.refresh();
            return rotorVelocitySignal.getValue();
        }
        /**
         * Sets the conversion factor. 1 means 1 rotation of the motor.
         * @param factor 1 means 1 rotation of the motor.
         */
        @Override
        public void setConversionFactor(double factor)
        {
            conversionFactor = Math.abs(factor) * (conversionFactor > 0 ? 1 : -1);
        }
        /**
         * Resets the encoder position. Do not do this too often. Could interfere with odometry.
         * @param position the new position of the encoder. Implementation factors in conversionFactor
         */
        @Override
        public void setEncoderPosition(double position)
        {
            setRotorPosition(position / conversionFactor);
        }
        /**
         * Sets the position using the simulation API. Use solely in simulation as it can lead to undefined behavior on
         * a robot.
         * @param position of the encoder which is affected by the conversion factor.
         */
        @Override
        public void setSimulationPosition(double position)
        {
            simState.setRawRotorPosition(position / conversionFactor);
        }
        /**
         * Adds to the position using the simulation API. Use solely in simulation as it can lead to undefined behavior on
         * a robot.
         * @param position of the encoder which is affected by the conversion factor.
         */
        @Override
        public void addSimulationPosition(double position)
        {
            simState.addRotorPosition(position / conversionFactor);
        }
        /**
         * Sets the velocity using the simulation API. Use solely in simulation as it can lead to undefined behavior on
         * a robot.
         * @param velocity of the encoder which is affected by the conversion factor.
         */
        @Override
        public void setSimulationVelocity(double velocity)
        {
            simState.setRotorVelocity(velocity / conversionFactor);
        }
    }
    private final TalonFXSimState simState;
    private final ArrayList<TalonFX> followers;
    private final PositionVoltage positionVoltage = new PositionVoltage(0);
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0);
    private NFREncoder selectedEncoder;
    private final IntegratedEncoder integratedEncoder;
    /**
     * Creates a new NFRTalon FX instance that contains a list of followers.
     * @param config The TalonFXConfiguration
     * @param primaryID the canbus id of the primary motor
     * @param followerIDs the canbus ids of the follower motors
     */
    public NFRTalonFX(TalonFXConfiguration config, int primaryID, int... followerIDs)
    {
        super(primaryID); 
        followers = new ArrayList<>();
        for (int id : followerIDs)
        {
            followers.add(new TalonFX(id));
        }
        getConfigurator().apply(config);
        for (var motor : followers)
        {
            motor.getConfigurator().apply(config);
        }
        selectedEncoder = integratedEncoder = new IntegratedEncoder();
        if (RobotBase.isSimulation())
        {
            simState = getSimState();
        }
        else
        {
            simState = null;
        }
    }
    /**
     * Creates a new NFRTalon FX instance that contains a list of followers.
     * @param config The TalonFXConfiguration
     * @param primaryID the canbus id of the primary motor
     */
    public NFRTalonFX(TalonFXConfiguration config, int primaryID)
    {
        this(config, primaryID, new int[]{});
    }
    /**
     * Checks to see whether an motor controller is present by comparing the version to -1.
     * @return true if the motor controller is present, false if not present
     */
    @Override
    public boolean isPresent()
    {
        return getVersion().getValue() != -1;
    }
    /**
     * Gets the number of motors in the NFRMotorController interface.
     * @return number of motor controllers
     */
    @Override
    public int getNumberOfMotors()
    {
        return followers.size() + 1;
    }
    /**
     * Returns a list of the WPILib interface for individual motor controllers.
     * @return a list of all of the motor controllers including the primary and the followers
     */
    @Override
    public List<MotorController> getMotorControllers()
    {
        ArrayList<MotorController> controllers = new ArrayList<>();
        controllers.add(this);
        controllers.addAll(followers);
        return controllers;
    }
    /**
     * Sets the selected encoder to a specific encoder that inherits from NFREncoder. This should be used only with
     * encoders that are compatible with the motor controller.
     * @param encoder to link to the motor controller
     * @throws MotorEncoderMismatchException an exception if the encoder is not compatible to be directly linked to 
     * the motor
     */
    @Override
    public void setSelectedEncoder(NFREncoder encoder) throws MotorEncoderMismatchException 
    {
        if (IntegratedEncoder.class.isAssignableFrom(encoder.getClass()))
        {
            selectedEncoder = encoder;
            TalonFXConfiguration config = new TalonFXConfiguration();
            getConfigurator().refresh(config);
            config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        }
        else
        {
            throw new MotorEncoderMismatchException(getClass(), encoder.getClass());
        }
    }
    /**
     * Returns the selected encoder that is linked to the motor controller. By default, the integrated encoder is chosen.
     * @return the selected encoder
     */
    @Override
    public NFREncoder getSelectedEncoder()
    {
        return selectedEncoder;
    }
    /**
     * Returns the external quadrature encoder plugged into the motor. May not be able to tell if the external quadrature
     * is present however.
     * @return optionally, the external quadrature encdoer
     */
    @Override
    public NFREncoder getIntegratedEncoder()
    {
        return integratedEncoder;
    }
    /**
     * Returns the external quadrature encoder plugged into the motor. May not be able to tell if the external quadrature
     * is present however.
     * @return optionally, the external quadrature encdoer
     */
    @Override
    public Optional<NFREncoder> getExternalQuadratureEncoder()
    {
        return Optional.empty();
    }
    /**
     * Returns the external absolute encoder plugged into the motor. May not be able to tell if the absolute
     * is present however.
     * @return optionally, the external absolute encoder
     */
    @Override
    public Optional<NFRAbsoluteEncoder> getAbsoluteEncoder()
    {
        return Optional.empty();
    }
    /**
     * Sets the velocity of the motor using closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param velocity the target velocity in terms of the selected sensor's units.
     */
    @Override
    public void setVelocity(int pidSlot, double velocity)
    {
        setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot));
    }
    /**
     * Sets the velocity of the motor using closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param velocity the target velocity in terms of the selected sensor's units.
     * @param arbitraryFeedforward the arbitrary percentage value applied. Often to counteract gravity.
     * Added to the output calculated by the closed-loop control.
     */
    @Override
    public void setVelocity(int pidSlot, double velocity, double arbitraryFeedforward)
    {
       setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot).withFeedForward(arbitraryFeedforward));
    }
    /**
     * Sets the position of the motor using closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param position the target position in terms of the selected sensor's units.
     */
    @Override
    public void setPosition(int pidSlot, double position)
    {
        setControl(positionVoltage.withPosition(position).withSlot(pidSlot));
    }
    /**
     * Sets the position of the motor using closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param position the target position in terms of the selected sensor's units.
     * @param arbitraryFeedforward the arbitrary percentage value applied. Often to counteract gravity.
     * Added to the output calculated by the closed-loop control.
     */
    @Override
    public void setPosition(int pidSlot, double position, double arbitraryFeedforward)
    {
        setControl(positionVoltage.withPosition(position).withSlot(pidSlot).withFeedForward(arbitraryFeedforward));
    }
    /**
     * Sets the position of the motor using trapezoidal closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param position the target position in terms of the selected sensor's units.
     */
    @Override
    public void setPositionTrapezoidal(int pidSlot, double position)
    {
        setControl(motionMagicVoltage.withSlot(pidSlot).withPosition(position));
    }
    /**
     * Sets the position of the motor using trapezoidal closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param position the target position in terms of the selected sensor's units.
     * @param arbitraryFeedforward the arbitrary percentage value applied. Often to counteract gravity.
     * Added to the output calculated by the closed-loop control.
     */
    @Override
    public void setPositionTrapezoidal(int pidSlot, double position, double arbitraryFeedforward)
    {
        setControl(motionMagicVoltage.withSlot(pidSlot).withPosition(position));
    }
    /**
     * Gets the simulation output voltage that is calculated by inputs to the motor. This should not be used
     * when not in simulation, as it can lead to undefined behavior.
     * @return the simulation output voltage [-12v..12v]
     */
    @Override
    public double getSimulationOutputVoltage()
    {
        return simState.getMotorVoltage();
    }
    /**
     * Sets the opposition of a follower so it opposes the inversion of the primary motor.
     * @param idx index of the motor in the opposition
     */
    @Override
    public void setFollowerOppose(int idx)
    {
        followers.get(idx).setControl(new Follower(getDeviceID(), true));
    }
}