package org.northernforce.subsystems.drive.swerve;

import java.util.Optional;

import org.northernforce.encoders.NFRAbsoluteEncoder;
import org.northernforce.motors.NFRMotorController;
import org.northernforce.subsystems.NFRSubsystem;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class NFRSwerveModule extends NFRSubsystem
{
    public static class NFRSwerveModuleConfiguration extends NFRSubsystemConfiguration
    {
        protected DCMotor driveGearbox, turnGearbox;
        protected double driveGearRatio, turnGearRatio, driveMOI, turnMOI;
        public NFRSwerveModuleConfiguration(String name)
        {
            super(name);
            driveGearbox = null;
            turnGearbox = null;
            driveGearRatio = 0;
            turnGearRatio = 0;
            driveMOI = 0;
            turnMOI = 0;
        }
        public NFRSwerveModuleConfiguration(String name, DCMotor driveGearbox, DCMotor turnGearbox,
            double driveGearRatio, double turnGearRatio, double driveMOI, double turnMOI)
        {
            super(name);
            this.driveGearbox = driveGearbox;
            this.turnGearbox = turnGearbox;
            this.driveGearRatio = driveGearRatio;
            this.turnGearRatio = turnGearRatio;
            this.driveMOI = driveMOI;
            this.turnMOI = turnMOI;
        }
        public NFRSwerveModuleConfiguration withGearboxes(DCMotor driveGearbox, DCMotor turnGearbox)
        {
            this.driveGearbox = driveGearbox;
            this.turnGearbox = turnGearbox;
            return this;
        }
        public NFRSwerveModuleConfiguration withGearRatios(double driveGearRatio, double turnGearRatio)
        {
            this.driveGearRatio = driveGearRatio;
            this.turnGearRatio = turnGearRatio;
            return this;
        }
        public NFRSwerveModuleConfiguration withMOIs(double driveMOI, double turnMOI)
        {
            this.driveMOI = driveMOI;
            this.turnMOI = turnMOI;
            return this;
        }
    }
    protected final NFRSwerveModuleConfiguration config;
    protected final NFRMotorController driveController, turnController;
    protected final Optional<NFRAbsoluteEncoder> externalEncoder;
    protected final FlywheelSim driveSim, turnSim;
    public NFRSwerveModule(NFRSwerveModuleConfiguration config, NFRMotorController driveController,
        NFRMotorController turnController, Optional<NFRAbsoluteEncoder> externalEncoder)
    {
        super(config);
        this.config = config;
        this.driveController = driveController;
        this.turnController = turnController;
        this.externalEncoder = externalEncoder;
        if (RobotBase.isSimulation())
        {
            driveSim = new FlywheelSim(config.driveGearbox, config.driveGearRatio, config.driveMOI);
            turnSim = new FlywheelSim(config.turnGearbox, config.turnGearRatio, config.turnMOI);
        }
        else
        {
            driveSim = null;
            turnSim = null;
        }
    }
    public Rotation2d getRotation()
    {
        return externalEncoder.isPresent() ? Rotation2d.fromRotations(externalEncoder.get().getAbsolutePosition())
            : Rotation2d.fromRotations(turnController.getSelectedEncoder().getPosition());
    }
    public double getVelocity()
    {
        return driveController.getSelectedEncoder().getVelocity();
    }
    public SwerveModuleState getState()
    {
        return new SwerveModuleState(getVelocity(), getRotation());
    }
    public double getDistance()
    {
        return driveController.getSelectedEncoder().getPosition();
    }
    public SwerveModulePosition getPosition()
    {
        return new SwerveModulePosition(getDistance(), getRotation());
    }
    public void setDriveSpeed(double velocity)
    {
        driveController.set(velocity);
    }
    public void setDriveSpeed(double velocity, int pidSlot)
    {
        driveController.setVelocity(pidSlot, velocity);
    }
    public void setTurnSpeed(double velocity)
    {
        turnController.set(velocity);
    }
    public void setTurnPosition(Rotation2d position, int pidSlot, boolean useTrapezoidalPositioning)
    {
        if (useTrapezoidalPositioning)
        {
            turnController.setPositionTrapezoidal(pidSlot, position.getRotations());
        }
        else
        {
            turnController.setPosition(pidSlot, position.getRotations());
        }
    }
    @Override
    public void simulationPeriodic()
    {
        driveSim.setInputVoltage(driveController.getSimulationOutputVoltage());
        turnSim.setInputVoltage(turnController.getSimulationOutputVoltage());
        driveSim.update(0.02);
        turnSim.update(0.02);
        driveController.getSelectedEncoder().addSimulationPosition(turnSim.getAngularVelocityRPM() / 60 * 0.02);
        driveController.getSelectedEncoder().setSimulationVelocity(turnSim.getAngularVelocityRPM() / 60);
        if (externalEncoder.isPresent())
        {
            externalEncoder.get().addAbsoluteSimulationPosition(turnSim.getAngularVelocityRPM() / 60 * 0.02);
            externalEncoder.get().setAbsoluteSimulationVelocity(turnSim.getAngularVelocityRPM() / 60);
        }
        else
        {
            turnController.getSelectedEncoder().addSimulationPosition(turnSim.getAngularVelocityRPM() / 60 * 0.02);
            turnController.getSelectedEncoder().setSimulationVelocity(turnSim.getAngularVelocityRPM() / 60);
        }
    }
}
