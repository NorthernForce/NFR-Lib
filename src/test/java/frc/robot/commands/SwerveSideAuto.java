package frc.robot.commands;

import org.northernforce.commands.NFRSwerveDriveStop;
import org.northernforce.commands.NFRSwerveModuleSetState;
import org.northernforce.commands.NFRSwerveMoveAtAngle;
import org.northernforce.subsystems.arm.NFRRollerIntake;
import org.northernforce.subsystems.arm.NFRRotatingArmJoint;
import org.northernforce.subsystems.drive.NFRSwerveDrive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class SwerveSideAuto extends SequentialCommandGroup
{
    public SwerveSideAuto(NFRSwerveDrive drive, NFRSwerveModuleSetState[] setStateCommands, NFRRotatingArmJoint joint, NFRRollerIntake intake)
    {
        addCommands(
            new NFRSwerveMoveAtAngle(drive, Rotation2d.fromDegrees(0), 0.75, setStateCommands, true).withTimeout(1.75),
            new NFRSwerveDriveStop(drive, setStateCommands, true)
        );
    }
}
