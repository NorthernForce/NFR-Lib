// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.northernforce.motors;

import java.util.*;
import java.util.function.Supplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.MotorFeedbackSensor;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import org.northernforce.encoders.SmartEncoder;
import org.northernforce.encoders.SmartSparkAbsoluteEncoder;
import org.northernforce.encoders.SmartSparkIntegratedEncoder;
/** 
 * Group of Sparks to be used like MotorController Class
*/
public class MotorGroupSpark extends CANSparkMax implements MotorGroup {
    private List<CANSparkMax> followers = new ArrayList<CANSparkMax>();
    private int invertCoefficient = 1;
    private Supplier<DCMotor> gearbox;
    /**
     * Creates a new motor controlled by a talon
     * @param primaryID id for the Talon being created
     */
    public MotorGroupSpark(MotorType type, Supplier<DCMotor> gearbox, int primaryID) {
        this(type, gearbox, primaryID, new int[]{});
    }
    /**
     * Creates a new motor with optional followers controlled by talons. 
     * Default inverted state is false.
     * @param primaryID id for the Talon being created
     * @param followerIDs ids in integer array for the followers
     */
    public MotorGroupSpark(MotorType type, Supplier<DCMotor> gearbox, int primaryID, int... followerIDs) {
        super(primaryID, type);
        for (int followerID: followerIDs) {
            this.followers.add(new CANSparkMax(followerID, type));
        }
        setFollowers();
        setInverted(false);
        configureAllControllers();
        if (type == MotorType.kBrushless)
            getEncoder().setVelocityConversionFactor(1 / 60);
        this.gearbox = gearbox;
        if (RobotBase.isSimulation())
        {
            REVPhysicsSim.getInstance().addSparkMax(this, getGearbox());
        }
    }
    public double getEncoderRotations() {
        return invertCoefficient * getEncoder().getPosition();
    }
    public double getEncoderRPS() {
        return invertCoefficient * getEncoder().getVelocity();
    }
    public void setFollowerOppose(int i) {
        followers.get(i).follow(this, true);
    }
    public void setInverted(boolean isInverted) {
        super.setInverted(isInverted);
        invertCoefficient = (isInverted ? -1 : 1);
    }
    public void resetEncoderRotations() {
        getEncoder().setPosition(0);
    }
    private void configureAllControllers() {
        configureController(this, false);
        for (CANSparkMax canSparkMax : followers) {
            configureController(canSparkMax, true);
        }
    }
    private void configureController(CANSparkMax controller, Boolean isFollower) {
        controller.setIdleMode(IdleMode.kBrake);
    }
    private void setFollowers() {
        for (CANSparkMax canSparkMax : followers) {
            canSparkMax.follow(this);
        }
    }
    /**
     * Sets up soft limits for a spark max.
     * @param forward
     * @param backward
     */
    public void setLimits(Rotation2d forward, Rotation2d backward) {
        setSoftLimit(SoftLimitDirection.kForward, (float)backward.getDegrees());
        setSoftLimit(SoftLimitDirection.kReverse, (float)forward.getDegrees());
        enableSoftLimit(SoftLimitDirection.kForward, true);
        enableSoftLimit(SoftLimitDirection.kReverse, true);
    }
    @Override
    public void configureClosedLoop(int slot, double kP, double kI, double kD, double kF, double kMaxAcceleration, double kMaxVelocity, double kAllowableError){
        getPIDController().setP(kP, slot);
        getPIDController().setI(kI, slot);
        getPIDController().setD(kD, slot);
        getPIDController().setSmartMotionAllowedClosedLoopError(kAllowableError, slot);
        getPIDController().setSmartMotionMaxAccel(kMaxAcceleration, slot);
        getPIDController().setSmartMotionMaxVelocity(kMaxVelocity, slot);
    }
    public void setUsingSmartMotion(double position, int slot){
        getPIDController().setReference(position, ControlType.kPosition, slot);
    }
    public double getAbsolute(){
        return getAbsoluteEncoder(Type.kDutyCycle).getPosition();
    }
    public void resetAbsolute(double position){
        getAbsoluteEncoder(Type.kDutyCycle).setZeroOffset(position - getAbsolute());
    }
    public double getAbsoluteRPS() {
        return getAbsoluteEncoder(Type.kDutyCycle).getVelocity();
    }
    public void setFeedbackSensor(MotorFeedbackSensor sensor)
    {
        getPIDController().setFeedbackDevice(sensor);
    }
    public AbsoluteEncoder getAbsoluteEncoder()
    {
        return getAbsoluteEncoder(Type.kDutyCycle);
    }
    @Override
    public void setVelocity(double velocity) {
        getPIDController().setReference(velocity, ControlType.kVelocity);
    }
    @Override
    public double getSimulationOutputVoltage() {
        return 0;
    }
    @Override
    public void setSimulationPosition(double position) {
    }
    @Override
    public void setSimulationVelocity(double velocity) {
    }
    @Override
    public void addSimulationPosition(double deltaPosition) {
    }
    @Override
    public void setPosition(double position) {
        getPIDController().setReference(position, ControlType.kPosition);
    }
    public DCMotor getGearbox()
    {
        return gearbox.get();
    }
    @Override
    public void setEncoderRotations(double rotations) {
        getEncoder().setPosition(rotations);
    }
    @Override
    public void setSmartPosition(double position) {
        getPIDController().setReference(position, ControlType.kSmartMotion);
    }
    @Override
    public void setForwardSoftLimit(double rotations)
    {
        setSoftLimit(SoftLimitDirection.kForward, (float)rotations);
        enableSoftLimit(SoftLimitDirection.kForward, true);
    }
    @Override
    public void setReverseSoftLimit(double rotations)
    {
        setSoftLimit(SoftLimitDirection.kReverse, (float)rotations);
        enableSoftLimit(SoftLimitDirection.kReverse, true);
    }
    @Override
    public void disableForwardSoftLimit()
    {
        enableSoftLimit(SoftLimitDirection.kForward, false);
    }
    @Override
    public void disableReverseSoftLimit()
    {
        enableSoftLimit(SoftLimitDirection.kReverse, false);
    }
    @Override
    public void linkEncoder(SmartEncoder encoder) throws MotorEncoderMismatchException
    {
        if (SmartSparkIntegratedEncoder.class.isAssignableFrom(encoder.getClass()))
        {
            setFeedbackSensor(((SmartSparkIntegratedEncoder)encoder).getEncoder());
        }
        if (MotorFeedbackSensor.class.isAssignableFrom(encoder.getClass()))
        {
            setFeedbackSensor((MotorFeedbackSensor)encoder);
        }
        if (SmartSparkAbsoluteEncoder.class.isAssignableFrom(encoder.getClass()))
        {
            setFeedbackSensor(((SmartSparkAbsoluteEncoder)encoder).getEncoder());
        }
        else
        {
            throw new MotorEncoderMismatchException(getClass(), encoder.getClass());
        }
    }
    @Override
    public SmartEncoder getIntegratedEncoder() {
        return new SmartSparkIntegratedEncoder(getEncoder());
    }
    @Override
    public void enableContinuousInput(boolean enable) {
        getPIDController().setPositionPIDWrappingEnabled(enable);
        getPIDController().setPositionPIDWrappingMinInput(0);
        getPIDController().setPositionPIDWrappingMaxInput(1);
    }
}
