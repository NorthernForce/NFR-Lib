package org.northernforce.encoders;

import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.wpilibj.RobotBase;

public class SmartSparkAbsoluteEncoder implements SmartAbsoluteEncoder {
    private final AbsoluteEncoder encoder;
    private double simulationPosition;
    private double simulationRate;
    public SmartSparkAbsoluteEncoder(AbsoluteEncoder encoder)
    {
        this.encoder = encoder;
        simulationPosition = 0;
        simulationRate = 0;
    }
    @Override
    public double getEncoderRotations() {
        return RobotBase.isReal() ? encoder.getPosition() : simulationPosition;
    }
    @Override
    public double getEncoderRPS() {
        return RobotBase.isReal() ? encoder.getVelocity() : simulationRate;
    }
    @Override
    public void setSimulationRotations(double rotations) {
        simulationPosition = rotations;
    }
    @Override
    public void setSimulationRPS(double rps) {
        simulationRate = rps;
    }
    @Override
    public void addSimulationRotations(double rotations) {
        simulationPosition += rotations;
    }
    @Override
    public void resetEncoderRotations(double rotations) {
        simulationPosition = rotations;
    }
    @Override
    public boolean isOnRoborio() {
        return false;
    }
    @Override
    public double getAbsoluteEncoderRotations() {
        return getEncoderRotations();
    }
    @Override
    public double getAbsoluteEncoderOffsetRotations() {
        return encoder.getZeroOffset();
    }
    @Override
    public void setAbsoluteEncoderOffsetRotations(double offset) {
        encoder.setZeroOffset(offset);
    }
    public AbsoluteEncoder getEncoder()
    {
        return encoder;
    }
}
