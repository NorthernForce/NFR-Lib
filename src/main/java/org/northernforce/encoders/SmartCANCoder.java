package org.northernforce.encoders;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;

import edu.wpi.first.wpilibj.RobotBase;

public class SmartCANCoder extends CANCoder implements SmartAbsoluteEncoder {
    private final CANCoderSimCollection simCollection;
    private int invertCoefficient = 1;
    public SmartCANCoder(int deviceNumber) {
        super(deviceNumber);
        if (RobotBase.isSimulation())
        {
            simCollection = getSimCollection();
            setSimulationRotations(0);
        }
        else
        {
            simCollection = null;
        }
    }
    @Override
    public ErrorCode configSensorDirection(boolean inverted)
    {
        invertCoefficient = inverted ? -1 : 1;
        return super.configSensorDirection(inverted);
    }
    @Override
    public double getEncoderRotations() {
        return getPosition() / 360;
    }
    @Override
    public double getEncoderRPS() {
        return getVelocity() / 360;
    }
    @Override
    public void setSimulationRotations(double rotations) {
        simCollection.setRawPosition((int)(4096 * rotations * invertCoefficient));
    }
    @Override
    public void setSimulationRPS(double rps) {
        simCollection.setVelocity((int)(4096 * rps * invertCoefficient / 10));
    }
    @Override
    public void addSimulationRotations(double rotations) {
        simCollection.addPosition((int)(4096 * rotations * invertCoefficient));
    }
    @Override
    public double getAbsoluteEncoderRotations() {
        return getAbsolutePosition() / 360;
    }
    @Override
    public double getAbsoluteEncoderOffsetRotations() {
        return configGetMagnetOffset() / 360;
    }
    @Override
    public void setAbsoluteEncoderOffsetRotations(double offset) {
        configMagnetOffset(offset * 360);
    }
    @Override
    public boolean isOnRoborio()
    {
        return false;
    }
    @Override
    public void resetEncoderRotations(double rotations) {
        resetAbsoluteEncoderOffset(rotations);
    }
}
