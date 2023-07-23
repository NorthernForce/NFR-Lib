package org.northernforce.motors;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.northernforce.encoders.NFRAbsoluteEncoder;
import org.northernforce.encoders.NFREncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * The class responsible for controlling a basic spark over PWM (ex. the motor controller that the Romi uses).
 */
public class NFRSpark extends Spark implements NFRMotorController
{
    /**
     * The command responsible for getting the motor controller to an exact speed via feedback.
     */
    private class VelocityCommand extends CommandBase
    {
        private PIDController controller = new PIDController(0, 0, 0);
        private double kF = 0, arbitraryFeedforward = 0;
        /**
         * Creates a new empty/unconfigured VelocityCommand.
         */
        public VelocityCommand()
        {
        }
        /**
         * Configures the velocity command with pid constants.
         * @param controller the controller containing the kP, kI, kD, etc.
         * @param kF the kF value to be multiplied by the setpoint.
         */
        public void setPIDController(PIDController controller, double kF)
        {
            this.controller = controller;
            this.kF = kF;
        }
        /**
         * Sets the target velocity of the motor controller.
         * @param velocity the velocity in units native to the encoder
         */
        public void setTarget(double velocity)
        {
            controller.setSetpoint(velocity);
            arbitraryFeedforward = 0;
        }
        /**
         * Sets the target velocity of the motor controller. With an arbitrary feedforward.
         * @param velocity the velocity in units native to the encoder
         * @param arbitraryFeedforward to be added to the PID Controller's computation.
         */
        public void setTarget(double velocity, double arbitraryFeedforward)
        {
            controller.setSetpoint(velocity);
            this.arbitraryFeedforward = arbitraryFeedforward;
        }
        /**
         * Initializes the command (Resets the controller).
         */
        @Override
        public void initialize()
        {
            controller.reset();
        }
        /**
         * Sets the speed of the motor based on feedback.
         */
        @Override
        public void execute()
        {
            internalSet(kF * controller.getSetpoint() + controller.calculate(selectedEncoder.getVelocity()) + arbitraryFeedforward);
        }
    }
    /**
     * The command responsible for getting the motor controller to a position via feedback.
     */
    private class PositionCommand extends CommandBase
    {
        private PIDController controller = new PIDController(0, 0, 0);
        private double arbitraryFeedforward = 0;
        /**
         * Creates a new empty/unconfigured PositionCommand.
         */
        public PositionCommand()
        {
        }
        /**
         * Configures the position command with pid constants.
         * @param controller the controller containing the kP, kI, kD, etc.
         */
        public void setPIDController(PIDController controller)
        {
            this.controller = controller;
        }
        /**
         * Sets the target position of the motor controller.
         * @param position the position in units native to the encoder
         */
        public void setTarget(double position)
        {
            controller.setSetpoint(position);
            arbitraryFeedforward = 0;
        }
        /**
         * Sets the target position of the motor controller. With an arbitrary feedforward.
         * @param position the position in units native to the encoder
         * @param arbitraryFeedforward to be added to the PID Controller's computation.
         */
        public void setTarget(double position, double arbitraryFeedforward)
        {
            controller.setSetpoint(position);
            this.arbitraryFeedforward = arbitraryFeedforward;
        }
        /**
         * Initializes the command (Resets the controller).
         */
        @Override
        public void initialize()
        {
            controller.reset();
        }
        /**
         * Sets the speed of the motor based on feedback.
         */
        @Override
        public void execute()
        {
            internalSet(controller.calculate(selectedEncoder.getPosition()) + arbitraryFeedforward);
        }
    }
    /**
     * The command responsible for getting the motor controller to a position via trapezoidal feedback.
     */
    private class PositionTrapezoidalCommand extends CommandBase
    {
        private ProfiledPIDController controller = new ProfiledPIDController(0, 0, 0,
            new Constraints(0, 0));
        private double arbitraryFeedforward = 0;
        /**
         * Creates a new empty/unconfigured PositionTrapezoidalCommand.
         */
        public PositionTrapezoidalCommand()
        {
        }
        /**
         * Configures the position command with pid constants.
         * @param controller the controller containing the kP, kI, kD, etc.
         */
        public void setPIDController(ProfiledPIDController controller)
        {
            this.controller = controller;
        }
        /**
         * Sets the target position of the motor controller.
         * @param position the position in units native to the encoder
         */
        public void setTarget(double position)
        {
            controller.setGoal(position);
            arbitraryFeedforward = 0;
        }
        /**
         * Sets the target position of the motor controller. With an arbitrary feedforward.
         * @param position the position in units native to the encoder
         * @param arbitraryFeedforward to be added to the PID Controller's computation.
         */
        public void setTarget(double position, double arbitraryFeedforward)
        {
            controller.setGoal(position);
            this.arbitraryFeedforward = arbitraryFeedforward;
        }
        /**
         * Initializes the command (Resets the controller).
         */
        @Override
        public void initialize()
        {
            controller.reset(selectedEncoder.getPosition());
        }
        /**
         * Sets the speed of the motor based on feedback.
         */
        @Override
        public void execute()
        {
            internalSet(controller.calculate(selectedEncoder.getPosition()) + arbitraryFeedforward);
        }
    }
    private final ArrayList<Spark> followers;
    private NFREncoder selectedEncoder = null;
    private Command runningCommand = null;
    private final VelocityCommand velocityCommand;
    private final PositionCommand positionCommand;
    private final PositionTrapezoidalCommand positionTrapezoidalCommand;
    private double positiveLimit, negativeLimit;
    private boolean positiveLimitEnabled = false, negativeLimitEnabled = false;
    /**
     * Creates a new NFRSpark.
     * @param primaryChannel the primary channel for pwm control.
     * @param channels the secondary channels for the subsequent motors.
     */
    public NFRSpark(int primaryChannel, int... channels)
    {
        super(primaryChannel);
        followers = new ArrayList<>();
        for (int channel : channels)
        {
            followers.add(new Spark(channel));
        }
        velocityCommand = new VelocityCommand();
        positionCommand = new PositionCommand();
        positionTrapezoidalCommand = new PositionTrapezoidalCommand();
    }
    /**
     * Creates a new NFRSpark.
     * @param primaryChannel the primary channel for pwm control.
     */
    public NFRSpark(int primaryChannel)
    {
        this(primaryChannel, new int[]{});
    }
    /**
     * Checks to see whether an motor controller is present through various means.
     * @return true if the motor controller is present, false if not present
     */
    @Override
    public boolean isPresent()
    {
        return true;
    }
    /**
     * Gets the number of motors in the NFRMotorController interface.
     * @return number of motor controllers
     */
    @Override
    public int getNumberOfMotors()
    {
        return 1 + followers.size();
    }
    /**
     * Sets the velocity PID constants.
     * @param controller the velocity PID controller.
     * @param kF the velocity kF value.
     */
    public void setVelocityPID(PIDController controller, double kF)
    {
        velocityCommand.setPIDController(controller, kF);
    }
    /**
     * Sets the position PID constants.
     * @param controller the position PID controller.
     */
    public void setPositionPID(PIDController controller)
    {
        positionCommand.setPIDController(controller);
    }
    /**
     * Sets the trapezoidal position PID constants.
     * @param controller the trapezoidal position PID controller.
     */
    public void setPositionTrapezoidalPID(ProfiledPIDController controller)
    {
        positionTrapezoidalCommand.setPIDController(controller);
    }
    /**
     * Returns a list of the WPILib interface for individual motor controllers.
     * @return a list of all of the motor controllers including the primary and the followers
     */
    @Override
    public List<MotorController> getMotorControllers()
    {
        ArrayList<MotorController> controllers = new ArrayList<>();
        controllers.add(this);
        controllers.addAll(followers);
        return controllers;
    }
    /**
     * Sets the inversion of the motor. Affects the integrated sensor, the external quadratures and pulse width absolutes
     * that are directly plugged into the motor.
     * @param inverted whether to invert the motor controller
     */
    @Override
    public void setInverted(boolean inverted)
    {
        super.setInverted(inverted);
        for (var follower : followers)
        {
            follower.setInverted(inverted);
        }
    }
    /**
     * Sets the selected encoder to a specific encoder that inherits from NFREncoder. This should be used only with
     * encoders that are compatible with the motor controller.
     * @param encoder to link to the motor controller
     * @throws MotorEncoderMismatchException an exception if the encoder is not compatible to be directly linked to 
     * the motor
     */
    @Override
    public void setSelectedEncoder(NFREncoder encoder) throws MotorEncoderMismatchException
    {
        selectedEncoder = encoder;
    }
    /**
     * Returns the selected encoder that is linked to the motor controller. By default, the integrated encoder is chosen.
     * @return the selected encoder
     */
    @Override
    public NFREncoder getSelectedEncoder()
    {
        return selectedEncoder;
    }
    /**
     * Returns the integrated encoder built into the motor it is controller. This is different from an alternate quadrature
     * that is separately pluggen into the motor controller.
     * @return the integrated encoder
     */
    @Override
    public NFREncoder getIntegratedEncoder()
    {
        return null;
    }
    /**
     * Returns the external quadrature encoder plugged into the motor. May not be able to tell if the external quadrature
     * is present however.
     * @return optionally, the external quadrature encdoer
     */
    @Override
    public Optional<NFREncoder> getExternalQuadratureEncoder()
    {
        return Optional.empty();
    }
    /**
     * Returns the external absolute encoder plugged into the motor. May not be able to tell if the absolute
     * is present however.
     * @return optionally, the external absolute encoder
     */
    @Override
    public Optional<NFRAbsoluteEncoder> getAbsoluteEncoder()
    {
        return Optional.empty();
    }
    /**
     * Sets the speed of the motor controllers.
     * @param speed the speed between -1 and 1.
     */
    @Override
    public void set(double speed)
    {
        if (runningCommand != null)
        {
            runningCommand.cancel();
        }
        runningCommand = null;
        internalSet(speed);
    }
    /**
     * Sets the speed of the motor controllers. This does not stop the running command, thus it is internal.
     * @param speed the speed between -1 and 1.
     */
    private void internalSet(double speed)
    {
        if (getSelectedEncoder() != null)
        {
            if (speed > 0 && positiveLimitEnabled)
            {
                if (getSelectedEncoder().getPosition() >= positiveLimit)
                {
                    speed = 0;
                }
            }
            else if (speed < 0 && negativeLimitEnabled)
            {
                if (getSelectedEncoder().getPosition() <= negativeLimit)
                {
                    speed = 0;
                }
            }
        }
        super.set(speed);
        for (var follower : followers)
        {
            follower.set(speed);
        }
    }
    /**
     * Sets the velocity of the motor using closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param velocity the target velocity in terms of the selected sensor's units.
     */
    @Override
    public void setVelocity(int pidSlot, double velocity)
    {
        if (runningCommand != null)
        {
            runningCommand.cancel();
        }
        runningCommand = velocityCommand;
        velocityCommand.setTarget(velocity);
        velocityCommand.schedule();
    }
    /**
     * Sets the velocity of the motor using closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param velocity the target velocity in terms of the selected sensor's units.
     * @param arbitraryFeedforward the arbitrary percentage value applied. Often to counteract gravity.
     * Added to the output calculated by the closed-loop control.
     */
    @Override
    public void setVelocity(int pidSlot, double velocity, double arbitraryFeedforward)
    {
        if (runningCommand != null)
        {
            runningCommand.cancel();
        }
        runningCommand = velocityCommand;
        velocityCommand.setTarget(velocity, arbitraryFeedforward);
        velocityCommand.schedule();
    }
    /**
     * Sets the position of the motor using closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param position the target position in terms of the selected sensor's units.
     */
    @Override
    public void setPosition(int pidSlot, double position)
    {
        if (runningCommand != null)
        {
            runningCommand.cancel();
        }
        runningCommand = positionCommand;
        positionCommand.setTarget(position);
        positionCommand.schedule();
    }
    /**
     * Sets the position of the motor using closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param position the target position in terms of the selected sensor's units.
     * @param arbitraryFeedforward the arbitrary percentage value applied. Often to counteract gravity.
     * Added to the output calculated by the closed-loop control.
     */
    @Override
    public void setPosition(int pidSlot, double position, double arbitraryFeedforward)
    {
        if (runningCommand != null)
        {
            runningCommand.cancel();
        }
        runningCommand = positionCommand;
        positionCommand.setTarget(position, arbitraryFeedforward);
        positionCommand.schedule();
    }
    /**
     * Sets the position of the motor using trapezoidal closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param position the target position in terms of the selected sensor's units.
     */
    @Override
    public void setPositionTrapezoidal(int pidSlot, double position)
    {
        if (runningCommand != null)
        {
            runningCommand.cancel();
        }
        runningCommand = positionTrapezoidalCommand;
        positionTrapezoidalCommand.setTarget(position);
        positionTrapezoidalCommand.schedule();
    }
    /**
     * Sets the position of the motor using trapezoidal closed-loop feedback. Must use a configured pid slot.
     * @param pidSlot the pid slot of the closed-loop configuration.
     * @param position the target position in terms of the selected sensor's units.
     * @param arbitraryFeedforward the arbitrary percentage value applied. Often to counteract gravity.
     * Added to the output calculated by the closed-loop control.
     */
    @Override
    public void setPositionTrapezoidal(int pidSlot, double position, double arbitraryFeedforward)
    {
        if (runningCommand != null)
        {
            runningCommand.cancel();
        }
        runningCommand = positionTrapezoidalCommand;
        positionTrapezoidalCommand.setTarget(position, arbitraryFeedforward);
        positionTrapezoidalCommand.schedule();
    }
    /**
     * Sets the voltage of the motor.
     * @param voltage of the motor. Between -12 volts and 12 volts.
     */
    @Override
    public void setVoltage(double voltage)
    {
        set(voltage / 12);
    }
    /**
     * Gets the simulation output voltage that is calculated by inputs to the motor. This should not be used
     * when not in simulation, as it can lead to undefined behavior.
     * @return the simulation output voltage [-12v..12v]
     */
    @Override
    public double getSimulationOutputVoltage()
    {
        return get() * 12;
    }
    /**
     * Sets the opposition of a follower so it opposes the inversion of the primary motor.
     * @param idx index of the motor in the opposition
     */
    @Override
    public void setFollowerOppose(int idx)
    {
        followers.get(idx).setInverted(!getInverted());
    }
    /**
     * Sets up limits that prevent the motor from moving in all modes when past these limits.
     * @param positiveLimit in selected sensor units
     * @param negativeLimit in selected sensor units
     */
    @Override
    public void setupLimits(double positiveLimit, double negativeLimit)
    {
        this.positiveLimit = positiveLimit;
        this.positiveLimitEnabled = true;
        this.negativeLimit = negativeLimit;
        this.negativeLimitEnabled = true;
    }
}