package frc.robot.commands;

import java.util.function.Supplier;

import org.northernforce.commands.NFRSwerveModuleSetState;
import org.northernforce.subsystems.drive.NFRSwerveDrive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Xavier;

public class FollowXavierPath extends CommandBase
{
    protected final NFRSwerveDrive drive;
    protected final NFRSwerveModuleSetState[] setStateCommands;
    protected final Xavier xavier;
    protected final Supplier<Pose2d> poseSupplier;
    public FollowXavierPath(NFRSwerveDrive drive, NFRSwerveModuleSetState[] setStateCommands, Xavier xavier, Supplier<Pose2d> poseSupplier)
    {
        addRequirements(drive, xavier);
        this.drive = drive;
        this.setStateCommands = setStateCommands;
        this.xavier = xavier;
        this.poseSupplier = poseSupplier;
    }
    @Override
    public void initialize()
    {
        for (NFRSwerveModuleSetState command : setStateCommands)
        {
            command.schedule();
        }
        xavier.sendTargetPose(poseSupplier.get());
    }
    @Override
    public void execute()
    {
        var targetVelocity = xavier.getTargetVelocity();
        targetVelocity.vxMetersPerSecond = Math.min(Math.max(targetVelocity.vxMetersPerSecond, -0.5), 0.5);
        targetVelocity.vyMetersPerSecond = Math.min(Math.max(targetVelocity.vyMetersPerSecond, -0.5), 0.5);
        targetVelocity.omegaRadiansPerSecond = Math.min(Math.max(targetVelocity.omegaRadiansPerSecond, -0.5), 0.5);
        SwerveModuleState[] targetStates = drive.toModuleStates(targetVelocity);
        for (int i = 0; i < targetStates.length; i++)
        {
            setStateCommands[i].setTargetState(targetStates[i]);
        }
    }
    @Override
    public void end(boolean interrupted)
    {
        for (NFRSwerveModuleSetState command : setStateCommands)
        {
            command.cancel();
        }
    }
}
