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

public class NFRTalonFX extends TalonFX implements NFRMotorController {
    public class IntegratedEncoder implements NFREncoder
    {
        private final StatusSignal<Double> rotorPositionSignal, rotorVelocitySignal;
        private double conversionFactor;
        public IntegratedEncoder()
        {
            this.rotorPositionSignal = getRotorPosition();
            this.rotorVelocitySignal = getRotorVelocity();
            conversionFactor = 1;
        }
        @Override
        public boolean isPresent()
        {
            return NFRTalonFX.this.isPresent();
        }
        @Override
        public double getPosition()
        {
            rotorVelocitySignal.refresh();
            return rotorPositionSignal.getValue() * conversionFactor;
        }
        @Override
        public void setInverted(boolean inverted)
        {
            conversionFactor = Math.abs(conversionFactor) * (inverted ? -1 : 1);
        }
        @Override
        public double getVelocity()
        {
            rotorVelocitySignal.refresh();
            return rotorVelocitySignal.getValue();
        }
        @Override
        public void setConversionFactor(double factor)
        {
            conversionFactor = Math.abs(factor) * (conversionFactor > 0 ? 1 : -1);
        }
        @Override
        public void setEncoderPosition(double position)
        {
            setRotorPosition(position);
        }
        @Override
        public void setSimulationPosition(double position)
        {
            simState.setRawRotorPosition(position / conversionFactor);
        }
        @Override
        public void addSimulationPosition(double position)
        {
            simState.addRotorPosition(position / conversionFactor);
        }
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
    public NFRTalonFX(TalonFXConfiguration config, int primaryID, int ...followerIDs)
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
    public NFRTalonFX(TalonFXConfiguration config, int primaryID)
    {
        this(config, primaryID, new int[]{});
    }
    @Override
    public boolean isPresent()
    {
        return getVersion().getValue() != -1;
    }
    @Override
    public int getNumberOfMotors()
    {
        return followers.size() + 1;
    }
    @Override
    public List<MotorController> getMotorControllers()
    {
        ArrayList<MotorController> controllers = new ArrayList<>();
        controllers.add(this);
        controllers.addAll(followers);
        return controllers;
    }
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
    }
    @Override
    public NFREncoder getSelectedEncoder()
    {
        return selectedEncoder;
    }
    @Override
    public NFREncoder getIntegratedEncoder()
    {
        return integratedEncoder;
    }
    @Override
    public Optional<NFREncoder> getExternalQuadratureEncoder()
    {
        return Optional.empty();
    }
    @Override
    public Optional<NFRAbsoluteEncoder> getAbsoluteEncoder()
    {
        return Optional.empty();
    }
    @Override
    public void setVelocity(int pidSlot, double velocity)
    {
        setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot));
    }
    @Override
    public void setVelocity(int pidSlot, double velocity, double arbitraryFeedforward)
    {
       setControl(velocityVoltage.withVelocity(velocity).withSlot(pidSlot).withFeedForward(arbitraryFeedforward));
    }
    @Override
    public void setPosition(int pidSlot, double position)
    {
        setControl(positionVoltage.withPosition(position).withSlot(pidSlot));
    }
    @Override
    public void setPosition(int pidSlot, double position, double arbitraryFeedforward)
    {
        setControl(positionVoltage.withPosition(position).withSlot(pidSlot).withFeedForward(arbitraryFeedforward));
    }
    @Override
    public void setPositionTrapezoidal(int pidSlot, double position)
    {
        setControl(motionMagicVoltage.withSlot(pidSlot).withPosition(position));
    }
    @Override
    public void setPositionTrapezoidal(int pidSlot, double position, double arbitraryFeedforward)
    {
        setControl(motionMagicVoltage.withSlot(pidSlot).withPosition(position));
    }
    @Override
    public double getSimulationOutputVoltage()
    {
        return simState.getMotorVoltage();
    }
    @Override
    public void setFollowerOppose(int idx)
    {
        followers.get(idx).setControl(new Follower(getDeviceID(), true));
    }
}
