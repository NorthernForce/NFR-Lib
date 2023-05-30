package org.northernforce.encoders;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;

public class SmartRioQuadrature extends Encoder implements SmartEncoder {
    private final double countsPerRevolution;
    private double resetOffset;
    private final EncoderSim simEncoder;
    public SmartRioQuadrature(DigitalSource sourceA, DigitalSource sourceB, double countsPerRevolution) {
        super(sourceA, sourceB);
        this.countsPerRevolution = countsPerRevolution;
        this.resetOffset = 0;
        if (RobotBase.isSimulation())
        {
            simEncoder = new EncoderSim(this);
        }
        else
        {
            simEncoder = null;
        }
    }
    @Override
    public double getEncoderRotations() {
        return super.get() / countsPerRevolution + resetOffset;
    }
    @Override
    public double getEncoderRPS() {
        return super.getRate() / countsPerRevolution;
    }
    @Override
    public void setSimulationRotations(double rotations) {
        simEncoder.setCount((int)(rotations * countsPerRevolution));
    }

    @Override
    public void setSimulationRPS(double rps) {
        simEncoder.setRate(rps * countsPerRevolution);
    }
    @Override
    public void addSimulationRotations(double rotations) {
        simEncoder.setCount((int)(simEncoder.getCount() + rotations / countsPerRevolution));
    }
    @Override
    public boolean isOnRoborio() {
        return true;
    }
    @Override
    public void resetEncoderRotations(double rotations) {
        resetOffset = rotations - getEncoderRotations();
    }
    
}
