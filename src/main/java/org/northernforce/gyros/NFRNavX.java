package org.northernforce.gyros;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.I2C.Port;

public class NFRNavX extends AHRS implements NFRGyro
{
    private final int simulationHandle;
    private final SimDouble headingSimulation;
    public NFRNavX()
    {
        if (RobotBase.isSimulation())
        {
            simulationHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            headingSimulation = new SimDouble(SimDeviceDataJNI.getSimValueHandle(simulationHandle, "Yaw"));
        }
        else
        {
            simulationHandle = -1;
            headingSimulation = null;
        }
    }
    public NFRNavX(Port port)
    {
        super(port);
        if (RobotBase.isSimulation())
        {
            simulationHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
            headingSimulation = new SimDouble(SimDeviceDataJNI.getSimValueHandle(simulationHandle, "Yaw"));
        }
        else
        {
            simulationHandle = -1;
            headingSimulation = null;
        }
    }
    @Override
    public Rotation2d getGyroYaw()
    {
        return Rotation2d.fromDegrees(getAngle());
    }
    @Override
    public Rotation3d getGyroAngle()
    {
        return new Rotation3d(
            getRoll(),
            getPitch(),
            getYaw()
        );
    }
    @Override
    public Translation3d getGyroVelocity()
    {
        return new Translation3d(
            getVelocityX(),
            getVelocityY(),
            getVelocityZ()
        );
    }
    @Override
    public Rotation2d getYawOffset()
    {
        return Rotation2d.fromDegrees(getAngleAdjustment());
    }
    @Override
    public void setYawOffset(Rotation2d offset)
    {
        setAngleAdjustment(offset.getDegrees());
    }
    @Override
    public void addSimulationYaw(Rotation2d deltaYaw)
    {
        headingSimulation.set(
            -MathUtil.inputModulus(getGyroYaw().plus(deltaYaw).getDegrees(), -180, 180) - getAngleAdjustment()
        );
    }
}