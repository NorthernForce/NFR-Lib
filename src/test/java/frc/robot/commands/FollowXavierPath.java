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
        SwerveModuleState[] targetStates = drive.toModuleStates(xavier.getTargetVelocity());
        for (int i = 0; i < targetStates.length; i++)
        {
            setStateCommands[i].setTargetState(targetStates[i], 1 / 3.3);
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
