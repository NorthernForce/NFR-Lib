package org.northernforce.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.northernforce.subsystems.drive.NFRSwerveDrive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NFRSwerveDriveWithJoystickAbsolute extends CommandBase
{
    protected final NFRSwerveDrive drive;
    protected final NFRSwerveModuleSetState[] setStateCommands;
    protected final DoubleSupplier xSupplier, ySupplier;
    protected final Supplier<Optional<Rotation2d>> thetaSupplier;
    protected final boolean optimize, fieldRelative;
    protected final ProfiledPIDController thetaController;
    /**
     * Creates a new NFRSwerveDriveWithJoystick
     * @param drive the swerve drive instance
     * @param setStateCommands the list of commands that will be used to set the state of the swerve drive.
     * @param xSupplier the x supplier for the ChassisSpeeds.
     * @param ySupplier the y supplier for the ChassisSpeeds.
     * @param thetaSupplier the theta supplier for the ChassisSpeeds.
     * @param optimize whether to optimize swerve module positions.
     * @param fieldRelative use field relative control.
     * @param thetaController the pid controller to control the theta velocity
     */
    public NFRSwerveDriveWithJoystickAbsolute(NFRSwerveDrive drive, NFRSwerveModuleSetState[] setStateCommands,
        DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Optional<Rotation2d>> thetaSupplier, boolean optimize,
        boolean fieldRelative, ProfiledPIDController thetaController)
    {
        addRequirements(drive);
        this.drive = drive;
        this.setStateCommands = setStateCommands;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.thetaSupplier = thetaSupplier;
        this.optimize = optimize;
        this.fieldRelative = fieldRelative;
        this.thetaController = thetaController;
        thetaController.enableContinuousInput(0, 360);
    }
    /**
     * Initializes the command and schedules the swerve module set state commands.
     */
    @Override
    public void initialize()
    {
        for (NFRSwerveModuleSetState command : setStateCommands)
        {
            command.schedule();
        }
    }
    /**
     * Calculates inverse kinematics based on the supplied joystick inputs and feeds it to the setState commands.
     */
    @Override
    public void execute()
    {
        var targetTheta = thetaSupplier.get();
        double thetaSpeed;
        if (targetTheta.isPresent())
        {
            thetaSpeed = thetaController.calculate(
                MathUtil.inputModulus(drive.getRotation().getDegrees(), 0, 360),
                MathUtil.inputModulus(targetTheta.get().getDegrees(), 0, 360));
        }
        else
        {
            thetaSpeed = 0;
        }
        SmartDashboard.putNumber("Theta Speed", thetaSpeed);
        thetaController.setPID(SmartDashboard.getNumber("Theta P", 0), SmartDashboard.getNumber("Theta I", 0), SmartDashboard.getNumber("Theta D", 0));
        ChassisSpeeds speeds = new ChassisSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), thetaSpeed);
        if (fieldRelative)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drive.getRotation());
        SwerveModuleState[] states = drive.toModuleStates(speeds);
        for (int i = 0; i < states.length; i++)
        {
            setStateCommands[i].setTargetState(optimize ? SwerveModuleState.optimize(states[i],
                drive.getModules()[i].getRotation()) : states[i]);
        }
    }
    /**
     * Stops all of the setState commands.
     */
    @Override
    public void end(boolean interrupted)
    {
        for (NFRSwerveModuleSetState command : setStateCommands)
        {
            command.cancel();
        }
    }
}
