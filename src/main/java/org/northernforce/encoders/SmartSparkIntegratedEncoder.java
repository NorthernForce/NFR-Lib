package org.northernforce.encoders;

import com.revrobotics.RelativeEncoder;

public class SmartSparkIntegratedEncoder implements SmartEncoder {
    private final RelativeEncoder encoder;
    public SmartSparkIntegratedEncoder(RelativeEncoder encoder)
    {
        this.encoder = encoder;
    }
    @Override
    public double getEncoderRotations() {
        return encoder.getPosition();
    }
    @Override
    public double getEncoderRPS() {
        return encoder.getVelocity() / 60;
    }
    @Override
    public void setSimulationRotations(double rotations) {
        encoder.setPosition(rotations);
    }
    @Override
    public void setSimulationRPS(double rps) {
    }
    @Override
    public void addSimulationRotations(double rotations) {
        encoder.setPosition(encoder.getPosition() + rotations);
    }
    @Override
    public void resetEncoderRotations(double rotations) {
        encoder.setPosition(rotations);
    }
    @Override
    public boolean isOnRoborio() {
        return false;
    }
    public RelativeEncoder getEncoder()
    {
        return encoder;
    }
}
