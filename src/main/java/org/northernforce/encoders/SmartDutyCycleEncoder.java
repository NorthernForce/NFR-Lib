package org.northernforce.encoders;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;

public class SmartDutyCycleEncoder extends DutyCycleEncoder implements SmartAbsoluteEncoder {
    private final DutyCycleEncoderSim simCollection;
    public SmartDutyCycleEncoder(DigitalSource source) {
        super(source);
        if (RobotBase.isSimulation())
        {
            simCollection = new DutyCycleEncoderSim(this);
        }
        else
        {
            simCollection = null;
        }
    }
    @Override
    public double getEncoderRotations() {
        return super.get();
    }
    @Override
    public double getEncoderRPS() {
        return 0;
    }
    @Override
    public void setSimulationRotations(double rotations) {
        simCollection.set(rotations);
    }
    @Override
    public void setSimulationRPS(double rps) {
    }
    @Override
    public void addSimulationRotations(double rotations) {
        simCollection.set(super.get() + rotations);
    }
    @Override
    public double getAbsoluteEncoderRotations() {
        return super.getAbsolutePosition();
    }
    @Override
    public double getAbsoluteEncoderOffsetRotations() {
        return super.getPositionOffset();
    }
    @Override
    public void setAbsoluteEncoderOffsetRotations(double offset) {
        super.setPositionOffset(offset);
    }
    @Override
    public boolean isOnRoborio()
    {
        return true;
    }
    @Override
    public void resetEncoderRotations(double rotations) {
        resetAbsoluteEncoderOffset(rotations);
    }
}
