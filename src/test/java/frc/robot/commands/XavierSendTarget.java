package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Xavier;

public class XavierSendTarget extends CommandBase
{
    protected final Xavier xavier;
    protected final Pose2d targetPose;
    public XavierSendTarget(Xavier xavier, Pose2d targetPose)
    {
        addRequirements(xavier);
        this.xavier = xavier;
        this.targetPose = targetPose;
    }
    @Override
    public void initialize()
    {
        xavier.sendTargetPose(targetPose);
    }
    @Override
    public boolean isFinished()
    {
        return true;
    }
}