// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.northernforce.motors;

import java.util.*;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import org.northernforce.encoders.SmartEncoder;



/** 
 * Group of Talons to be used like MotorController Class
*/
public class MotorGroupTalonSRX extends WPI_TalonSRX implements MotorGroup {
    private List<WPI_TalonSRX> followers = new ArrayList<WPI_TalonSRX>();
    private int COUNTS_PER_REVOLUTION = 2048;
    private int invertCoefficient = 1;
    private TalonFXSimCollection motorSim;
    private Supplier<DCMotor> gearbox;
    /**
     * Creates a new motor controlled by a talon
     * @param primaryID id for the Talon being created
     */
    public MotorGroupTalonSRX(Supplier<DCMotor> gearbox, int primaryID) {
        this(gearbox, primaryID, new int[]{});
    }
    /**
     * Creates a new motor with optional followers controlled by talons.
     * Default inverted state is false.
     * @param primaryID id for the Talon being created
     * @param followerIDs ids in integer array for the followers
     */
    public MotorGroupTalonSRX(Supplier<DCMotor> gearbox, int primaryID, int[] followerIDs) {
        super(primaryID);
        for (int followerID: followerIDs) {
            this.followers.add(new WPI_TalonSRX(followerID));
        }
        setFollowers();
        setInverted(false);
        configureAllControllers();
        if (RobotBase.isSimulation())
        {
            motorSim = getTalonFXSimCollection();
        }
        this.gearbox = gearbox;
    }
    public double getEncoderRotations() {
        return getSelectedSensorPosition() / COUNTS_PER_REVOLUTION;
    }
    public double getEncoderRPS() {
        //10 represents the amount of 100ms periods in a single second.
        return getSelectedSensorVelocity() / COUNTS_PER_REVOLUTION * 10;
    }
    public void setFollowerOppose(int i) {
        followers.get(i).setInverted(InvertType.OpposeMaster);
    }
    public void setInverted(boolean isInverted) {
        super.setInverted(isInverted);
        invertCoefficient = isInverted ? -1 : 1;
    }
    public void resetEncoderRotations() {
        setSelectedSensorPosition(0);
    }
    private void configureAllControllers() {
        configureController(this, false);
        for (WPI_TalonSRX wpi_TalonFX : followers) {
            configureController(wpi_TalonFX, true);
        }
    }
    /**
     * Links CANCoder to be used
     * @param coder CANCoder reference
     */
    public void linkAndUseCANCoder(CANCoder coder)
    {
        configRemoteFeedbackFilter(coder, 0);
        configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
        setCountsPerRevolution(4096);
    }
    /**
     * Configures the internal closed loop to be used.
     */
    public void configSelectedSlot(int slotIdx, int pidIdx)
    {
        selectProfileSlot(slotIdx, pidIdx);
    }
    /**
     * Sets the velocity of the motor
     * Uses integrated PID.
     * @param velocity does not factor in gear ratio
     * @param feedforward value of feedforward
     */
    public void setVelocity(double velocity, double feedforward)
    {
        set(ControlMode.Velocity, velocity * COUNTS_PER_REVOLUTION / 10, DemandType.ArbitraryFeedForward, feedforward);
    }
    /**
     * Sets the position using motion magic
     * @param position position in rotations... does not factor in gear ratio
     * @param feedforward the feedforward value to be added
     */
    public void setMotionMagic(double position, double feedforward)
    {
        set(ControlMode.MotionMagic, position * COUNTS_PER_REVOLUTION, DemandType.ArbitraryFeedForward, feedforward);
    }
    public void setCountsPerRevolution(int countsPerRevolution)
    {
        COUNTS_PER_REVOLUTION = countsPerRevolution;
    }
    /**
     * Sets the position of Falcon motor using integrated PIDControl
     * @param rotations Number of rotations. Does not factor in gear ratio.
     */
    public void setPosition(double rotations)
    {
        set(ControlMode.Position, rotations * COUNTS_PER_REVOLUTION);
    }
    /**
     * Sets the position of Falcon motor using integrated PIDControl
     * @param rotations Number of rotations. Does not factor in gear ratio.
     */
    public void setPosition(double rotations, double feedforward)
    {
        set(ControlMode.Position, rotations * COUNTS_PER_REVOLUTION, DemandType.ArbitraryFeedForward, feedforward);
    }
    /**
     * Configures a closed loop
     * @param slotIdx the index of the closed loop to configure. Thus you can have multiple
     * @param allowableError allowableError in sensor units per 100ms.
     * @param kF velocity feedforward gain
     * @param kP proportion
     * @param kI integral
     * @param kD derivative
     */
    public void configClosedLoop(int slotIdx, double allowableError, double kF, double kP, double kI, double kD)
    {
        configAllowableClosedloopError(slotIdx, allowableError, 0);
        config_kF(slotIdx, kF, 0);
		config_kP(slotIdx, kP, 0);
		config_kI(slotIdx, kI, 0);
		config_kD(slotIdx, kD, 0);
    }
    /**
     * Sets the current encoder rotations
     * @param rotations in rotations... does not factor in gear ratio
     */
    public void setEncoderRotations(double rotations)
    {
        setSelectedSensorPosition(COUNTS_PER_REVOLUTION * rotations);
    }
    private void configureController(WPI_TalonSRX controller, Boolean isFollower) {
        /** These 3 values are used to prevent breakers from tripping*/
        final double currentLimit = 40; //Holding current in amps to limit when feature is activated
        final double limitThreshold = 90; //Current must excede this threshold (amps) before limiting occurs
        final double triggerThreshTimeInSec = 1; //How long the current must excede thrreshold (seconds) before limiting occurs
        controller.configFactoryDefault();
        controller.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, currentLimit, limitThreshold, triggerThreshTimeInSec));
        if (!isFollower) {
          controller.configClosedloopRamp(0.2);
          controller.configOpenloopRamp(0.2);
        }
        controller.setNeutralMode(NeutralMode.Brake);
        TalonSRXConfiguration configs = new TalonSRXConfiguration();
        configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
        controller.configAllSettings(configs);
    }
    private void setFollowers() {
        for (WPI_TalonSRX wpi_TalonFX : followers) {
            wpi_TalonFX.follow(this);
            wpi_TalonFX.setInverted(InvertType.FollowMaster);
        }
    }
    /**
     * Sets the soft limits of a TalonSRX Controller
     * @param positive the forward angle maximum
     * @param negative the backward angle maximum
     */
    public void setLimits(Rotation2d positive, Rotation2d negative) {
        configForwardSoftLimitThreshold(positive.getRotations() * COUNTS_PER_REVOLUTION, 0);
        configReverseSoftLimitThreshold(negative.getRotations() * COUNTS_PER_REVOLUTION, 0);
        configForwardSoftLimitEnable(true, 0);
        configReverseSoftLimitEnable(true, 0);
    }
    @Override
    public void setVelocity(double velocity) {
        set(ControlMode.Velocity, velocity);
    }
    @Override
    public double getSimulationOutputVoltage() {
        return motorSim.getMotorOutputLeadVoltage() * invertCoefficient;
    }
    @Override
    public void setSimulationPosition(double position) {
        motorSim.setIntegratedSensorRawPosition((int)(invertCoefficient * COUNTS_PER_REVOLUTION * position));
    }
    @Override
    public void setSimulationVelocity(double velocity) {
        motorSim.setIntegratedSensorVelocity((int)(invertCoefficient * COUNTS_PER_REVOLUTION * velocity / 10));
    }
    @Override
    public void addSimulationPosition(double deltaPosition) {
        motorSim.addIntegratedSensorPosition((int)(invertCoefficient * deltaPosition * COUNTS_PER_REVOLUTION));
    }
    @Override
    public DCMotor getGearbox()
    {
        return gearbox.get();
    }
    @Override
    public void configureClosedLoop(int slotIdx, double kP, double kI, double kD, double kF, double kMaxAcceleration,
        double kMaxVelocity, double kAllowableError) {
        config_kP(slotIdx, kP);
        config_kI(slotIdx, kI);
        config_kD(slotIdx, kD);
        config_kF(slotIdx, kF);
        configAllowableClosedloopError(slotIdx, kAllowableError * COUNTS_PER_REVOLUTION / 10);
        configMotionAcceleration(kMaxAcceleration);
        configMotionCruiseVelocity(kMaxVelocity);
    }
    @Override
    public void setSmartPosition(double position) {
        setMotionMagic(position, 0);
    }
    @Override
    public void setForwardSoftLimit(double rotations)
    {
        configForwardSoftLimitThreshold(rotations * COUNTS_PER_REVOLUTION);
        configForwardSoftLimitEnable(true);
    }
    @Override
    public void setReverseSoftLimit(double rotations)
    {
        configReverseSoftLimitThreshold(rotations * COUNTS_PER_REVOLUTION);
        configReverseSoftLimitEnable(true);
    }
    @Override
    public void disableForwardSoftLimit()
    {
        configForwardSoftLimitEnable(false);
    }
    @Override
    public void disableReverseSoftLimit()
    {
        configReverseSoftLimitEnable(false);
    }
    @Override
    public void linkEncoder(SmartEncoder encoder) throws MotorEncoderMismatchException
    {
        if (CANCoder.class.isAssignableFrom(encoder.getClass()))
        {
            linkAndUseCANCoder((CANCoder)encoder);
        }
        else
        {
            throw new MotorEncoderMismatchException(getClass(), encoder.getClass());
        }
    }
    @Override
    public SmartEncoder getIntegratedEncoder() {
        return null;
    }
    @Override
    public void enableContinuousInput(boolean enable) {
        configFeedbackNotContinuous(!enable, 0);
    }
}
