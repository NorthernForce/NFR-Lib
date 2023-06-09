package org.northernforce.motors;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.northernforce.encoders.NFRAbsoluteEncoder;
import org.northernforce.encoders.NFREncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class NFRSparkMax extends CANSparkMax implements NFRMotorController
{
    public class IntegratedEncoder implements NFREncoder
    {
        @Override
        public boolean isPresent()
        {
            return NFRSparkMax.this.isPresent();
        }
        @Override
        public double getPosition()
        {
            return getEncoder().getPosition();
        }
        @Override
        public void setInverted(boolean inverted)
        {
            getEncoder().setInverted(inverted);
        }
        @Override
        public double getVelocity() {
            return getEncoder().getVelocity();
        }
        @Override
        public void setConversionFactor(double factor)
        {
            getEncoder().setPositionConversionFactor(factor);
            getEncoder().setVelocityConversionFactor(factor);
        }
        @Override
        public void setEncoderPosition(double position)
        {
            getEncoder().setPosition(position);
        }
        @Override
        public void setSimulationPosition(double position)
        {
        }
        @Override
        public void addSimulationPosition(double position)
        {
        }
        @Override
        public void setSimulationVelocity(double velocity)
        {
        }
    }
    private final ArrayList<CANSparkMax> followers;
    private final IntegratedEncoder integratedEncoder;
    private NFREncoder selectedEncoder;
    public NFRSparkMax(MotorType type, int primaryId, int... followerIds)
    {
        super(primaryId, type);
        followers = new ArrayList<>();
        for (int id : followerIds)
        {
            followers.add(new CANSparkMax(id, type));
        }
        for (var follower : followers)
        {
            follower.follow(this);
        }
        integratedEncoder = new IntegratedEncoder();
    }
    @Override
    public boolean isPresent()
    {
        return getFirmwareVersion() == 0;
    }
    @Override
    public int getNumberOfMotors()
    {
        return 1 + followers.size();
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
            getPIDController().setFeedbackDevice(getEncoder());
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
    public class AbsoluteEncoder implements NFRAbsoluteEncoder
    {
        @Override
        public boolean isPresent()
        {
            return NFRSparkMax.this.isPresent();
        }
        @Override
        public double getPosition()
        {
            return getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        }
        @Override
        public void setInverted(boolean inverted)
        {
            getAbsoluteEncoder(Type.kDutyCycle).setInverted(inverted);
        }
        @Override
        public double getVelocity()
        {
            return getAbsoluteEncoder(Type.kDutyCycle).getVelocity();
        }
        @Override
        public void setConversionFactor(double factor)
        {
            getAbsoluteEncoder(Type.kDutyCycle).setPositionConversionFactor(factor);
            getAbsoluteEncoder(Type.kDutyCycle).setVelocityConversionFactor(factor);
        }
        @Override
        public void setEncoderPosition(double position)
        {
            getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(
                position - getAbsoluteEncoder(Type.kDutyCycle).getPosition() + getAbsoluteEncoder(Type.kDutyCycle).getZeroOffset()
            );
        } 
        @Override
        public void setSimulationPosition(double position)
        {
        }
        @Override
        public void addSimulationPosition(double position)
        {
        }
        @Override
        public void setSimulationVelocity(double velocity) {
        }
        @Override
        public double getAbsolutePosition()
        {
            return getAbsoluteEncoder(Type.kDutyCycle).getPosition();
        }
        @Override
        public double getAbsoluteVelocity() {
            return getAbsoluteEncoder(Type.kDutyCycle).getVelocity();
        }
        @Override
        public void setAbsoluteInverted(boolean inverted) {
            getAbsoluteEncoder(Type.kDutyCycle).setInverted(inverted);
        }
        @Override
        public void setAbsoluteOffset(double offset) {
            getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(offset);
        }
        @Override
        public void setAbsoluteConversionFactor(double factor) {
            setConversionFactor(factor);
        }
        @Override
        public void setUseAbsoluteMeasurements(boolean useAbsoluteMeasurements) {
        }
        @Override
        public void setAbsoluteSimulationPosition(double position) {
        }
        @Override
        public void addAbsoluteSimulationPosition(double position) {
        }
        @Override
        public void setAbsoluteSimulationVelocity(double velocity) {
        }
    }
    @Override
    public Optional<NFRAbsoluteEncoder> getAbsoluteEncoder()
    {
        return Optional.of(new AbsoluteEncoder());
    }
    @Override
    public void setVelocity(int pidSlot, double velocity) {
        getPIDController().setReference(velocity, ControlType.kVelocity, pidSlot);
    }
    @Override
    public void setVelocity(int pidSlot, double velocity, double arbitraryFeedforward) {
        getPIDController().setReference(velocity, ControlType.kVelocity, pidSlot, arbitraryFeedforward);
    }
    @Override
    public void setPosition(int pidSlot, double position) {
        getPIDController().setReference(position, ControlType.kPosition, pidSlot);
    }
    @Override
    public void setPosition(int pidSlot, double position, double arbitraryFeedforward) {
        getPIDController().setReference(position, ControlType.kPosition, pidSlot);
    }
    @Override
    public void setPositionTrapezoidal(int pidSlot, double position) {
        getPIDController().setReference(position, ControlType.kSmartMotion, pidSlot);
    }
    @Override
    public void setPositionTrapezoidal(int pidSlot, double position, double arbitraryFeedforward) {
        getPIDController().setReference(position, ControlType.kSmartMotion, pidSlot, arbitraryFeedforward);
    }
    @Override
    public double getSimulationOutputVoltage() {
        return get() * RobotController.getBatteryVoltage();
    }
    @Override
    public void setFollowerOppose(int idx) {
        followers.get(idx).follow(this, true);
    }    
}
