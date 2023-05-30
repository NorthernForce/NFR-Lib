package org.northernforce.encoders;

import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;

import org.northernforce.motors.MotorGroupTalonFX;

public class SmartTalonFXIntegratedEncoder implements SmartEncoder {
    private final TalonFXSensorCollection sensor;
    private final MotorGroupTalonFX group;
    private final TalonFXSimCollection simCollection;
    public SmartTalonFXIntegratedEncoder(TalonFXSensorCollection sensor, MotorGroupTalonFX group,
        TalonFXSimCollection simCollection)
    {
        this.sensor = sensor;
        this.group = group;
        this.simCollection = simCollection;
    }
    @Override
    public double getEncoderRotations() {
        return sensor.getIntegratedSensorPosition() / 2048 * (group.getInverted() ? -1 : 1);
    }
    @Override
    public double getEncoderRPS() {
        return sensor.getIntegratedSensorVelocity() / 204.8 * (group.getInverted() ? -1 : 1);
    }
    @Override
    public void setSimulationRotations(double rotations) {
        simCollection.setIntegratedSensorRawPosition((int)((group.getInverted() ? -1 : 1) * rotations * 2048));
    }
    @Override
    public void setSimulationRPS(double rps) {
        simCollection.setIntegratedSensorRawPosition((int)((group.getInverted() ? -1 : 1) * rps * 204.8));
    }
    @Override
    public void addSimulationRotations(double rotations) {
        simCollection.addIntegratedSensorPosition((int)((group.getInverted() ? -1 : 1) * rotations * 2048));
    }
    @Override
    public void resetEncoderRotations(double rotations) {
        sensor.setIntegratedSensorPosition((group.getInverted() ? -1 : 1) * rotations * 2048, 0);
    }
    @Override
    public boolean isOnRoborio() {
        return false;
    }
}
