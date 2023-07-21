package org.northernforce.subsystems.drive.swerve;

import java.util.Optional;

import org.northernforce.encoders.NFRAbsoluteEncoder;
import org.northernforce.encoders.NFRCANCoder;
import org.northernforce.motors.MotorEncoderMismatchException;
import org.northernforce.motors.NFRMotorController;
import org.northernforce.motors.NFRTalonFX;
import org.northernforce.subsystems.NFRSubsystem;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class NFRSwerveModule extends NFRSubsystem
{
    public static class NFRSwerveModuleConfiguration extends NFRSubsystemConfiguration
    {
        protected DCMotor driveGearbox, turnGearbox;
        protected double driveGearRatio, turnGearRatio, driveMOI, turnMOI, maxSpeed;
        public NFRSwerveModuleConfiguration(String name)
        {
            super(name);
            driveGearbox = null;
            turnGearbox = null;
            driveGearRatio = 0;
            turnGearRatio = 0;
            driveMOI = 0;
            turnMOI = 0;
            maxSpeed = 0;
        }
        public NFRSwerveModuleConfiguration(String name, DCMotor driveGearbox, DCMotor turnGearbox,
            double driveGearRatio, double turnGearRatio, double driveMOI, double turnMOI, double maxSpeed)
        {
            super(name);
            this.driveGearbox = driveGearbox;
            this.turnGearbox = turnGearbox;
            this.driveGearRatio = driveGearRatio;
            this.turnGearRatio = turnGearRatio;
            this.driveMOI = driveMOI;
            this.turnMOI = turnMOI;
            this.maxSpeed = maxSpeed;
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
        public NFRSwerveModuleConfiguration withMaxSpeed(double maxSpeed)
        {
            this.maxSpeed = maxSpeed;
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
            turnController.setPositionTrapezoidal(pidSlot,
                MathUtil.inputModulus(position.getRotations(), 0, 1));
        }
        else
        {
            turnController.setPosition(pidSlot, position.getRotations(),
                MathUtil.inputModulus(position.getRotations(), 0, 1));
        }
    }
    @Override
    public void simulationPeriodic()
    {
        driveSim.setInputVoltage(driveController.getSimulationOutputVoltage());
        turnSim.setInputVoltage(turnController.getSimulationOutputVoltage());
        driveSim.update(0.02);
        turnSim.update(0.02);
        driveController.getSelectedEncoder().addSimulationPosition(driveSim.getAngularVelocityRPM() / 60 * 0.02);
        driveController.getSelectedEncoder().setSimulationVelocity(driveSim.getAngularVelocityRPM() / 60);
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
    public SwerveModuleState scaleSpeed(SwerveModuleState state)
    {
        return new SwerveModuleState(state.speedMetersPerSecond * config.maxSpeed, state.angle);
    }
    @Override
    public void initSendable(SendableBuilder builder)
    {
        builder.addDoubleProperty("Velocity", () -> getVelocity(), null);
        builder.addDoubleProperty("Angle", () -> getRotation().getDegrees(), null);
        builder.addDoubleProperty("Target Angle", () -> turnController.getTargetPosition(), null);
        builder.addDoubleProperty("Angle Rotations", () -> turnController.getSelectedEncoder().getPosition(), null);
        if (RobotBase.isSimulation())
        {
            builder.addDoubleProperty("Drive - Simulation Voltage", driveController::getSimulationOutputVoltage, null);
            builder.addDoubleProperty("Turn - Simulation Voltage", turnController::getSimulationOutputVoltage, null);
            builder.addDoubleProperty("Drive - Estimated Speed", driveSim::getAngularVelocityRPM, null);
            builder.addDoubleProperty("Turn - Estimated Speed", turnSim::getAngularVelocityRPM, null);
        }
    }
    public static final class Mk3SwerveConstants
    {
        public static final double kDriveGearRatioSlow = 8.16;
        public static final double kDriveGearRatioFast = 6.86;
        public static final double kTurnGearRatio = 12.8;
        public static final double kWheelRadius = Units.inchesToMeters(2);
        public static final double kWheelCircumference = kWheelRadius * 2 * Math.PI;
        public static final double kDriveMaxSpeed = Units.feetToMeters(13);
        public static final double kDriveV = 12 / (kDriveMaxSpeed * kDriveGearRatioSlow / kWheelCircumference);
        public static final double kDriveP = 0.5;
        public static final double kTurnMaxSpeed = 1;
        public static final double kTurnD = 0.2;
        public static final double kTurnP = 30;
    }
    public static NFRSwerveModule createMk3Slow(String name, int driveID, int turnID, int cancoderID)
    {
        NFRSwerveModuleConfiguration config = new NFRSwerveModuleConfiguration("Front Left Module")
            .withGearRatios(Mk3SwerveConstants.kDriveGearRatioSlow, Mk3SwerveConstants.kTurnGearRatio)
            .withGearboxes(DCMotor.getFalcon500(1), DCMotor.getFalcon500(1))
            .withMOIs(1.2, 1.2)
            .withMaxSpeed(Mk3SwerveConstants.kDriveMaxSpeed);
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = 40;
        driveConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        driveConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.Slot0.kP = Mk3SwerveConstants.kDriveP;
        driveConfig.Slot0.kV = Mk3SwerveConstants.kDriveV;
        NFRTalonFX driveMotor = new NFRTalonFX(driveConfig, driveID);
        driveMotor.getSelectedEncoder().setConversionFactor(Mk3SwerveConstants.kWheelCircumference /
            Mk3SwerveConstants.kDriveGearRatioSlow);
        TalonFXConfiguration turnConfig = new TalonFXConfiguration();
        turnConfig.CurrentLimits.SupplyCurrentLimit = 40;
        turnConfig.CurrentLimits.SupplyCurrentThreshold = 60;
        turnConfig.CurrentLimits.SupplyTimeThreshold = 0.5;
        turnConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        turnConfig.MotionMagic.MotionMagicCruiseVelocity = Mk3SwerveConstants.kTurnGearRatio * 8;
        turnConfig.MotionMagic.MotionMagicAcceleration = turnConfig.MotionMagic.MotionMagicCruiseVelocity * 4;
        turnConfig.Slot0.kP = Mk3SwerveConstants.kTurnP;
        turnConfig.Slot0.kD = Mk3SwerveConstants.kTurnD;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        NFRTalonFX turnMotor = new NFRTalonFX(turnConfig, turnID);
        NFRCANCoder cancoder = new NFRCANCoder(cancoderID);
        try
        {
            turnMotor.setSelectedEncoder(cancoder);
        }
        catch (MotorEncoderMismatchException e)
        {
            e.printStackTrace();
        }
        return new NFRSwerveModule(config, driveMotor, turnMotor, Optional.empty());
    }
}
