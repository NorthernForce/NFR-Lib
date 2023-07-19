package frc.robot.robots;

import java.util.Map;

import org.northernforce.commands.NFRSwerveDriveWithJoystick;
import org.northernforce.commands.NFRSwerveModuleSetState;
import org.northernforce.gyros.NFRNavX;
import org.northernforce.subsystems.drive.NFRSwerveDrive;
import org.northernforce.subsystems.drive.NFRSwerveDrive.NFRSwerveDriveConfiguration;
import org.northernforce.subsystems.drive.swerve.NFRSwerveModule;
import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SwervyContainer implements NFRRobotContainer
{
    private final NFRSwerveDrive drive;
    private final Field2d field;
    public SwervyContainer()
    {
        NFRSwerveModule[] modules = new NFRSwerveModule[] {
            NFRSwerveModule.createMk3Slow("Front Left", 1, 5, 9),
            NFRSwerveModule.createMk3Slow("Front Right", 2, 6, 10),
            NFRSwerveModule.createMk3Slow("Back Left", 3, 7, 11),
            NFRSwerveModule.createMk3Slow("Back Right", 4, 8, 12)
        };
        NFRSwerveDriveConfiguration driveConfig = new NFRSwerveDriveConfiguration(
            "Swerve Drive",
            Units.feetToMeters(12)
        );
        Translation2d[] offsets = new Translation2d[] {
            new Translation2d(0.581025, 0.581025),
            new Translation2d(0.581025, -0.581025),
            new Translation2d(-0.581025, 0.581025),
            new Translation2d(-0.581025, -0.581025)
        };
        drive = new NFRSwerveDrive(driveConfig, modules, offsets, new NFRNavX());
        field = new Field2d();
        Shuffleboard.getTab("Swerve").add("Field", field);
    }
    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID)
    {
        if (driverHID instanceof XboxController && manipulatorHID instanceof XboxController)
        {
            XboxController driverController = (XboxController)driverHID;
            NFRSwerveModuleSetState[] commands = new NFRSwerveModuleSetState[] {
                new NFRSwerveModuleSetState(drive.getModules()[0], 0, 0,
                    false),
                new NFRSwerveModuleSetState(drive.getModules()[1], 0, 0,
                    false),
                new NFRSwerveModuleSetState(drive.getModules()[2], 0, 0,
                    false),
                new NFRSwerveModuleSetState(drive.getModules()[3], 0, 0,
                    false)
            };
            drive.setDefaultCommand(new NFRSwerveDriveWithJoystick(drive, commands, () -> -driverController.getLeftY(),
                () -> -driverController.getLeftX(), () -> -driverController.getRightX(), true));
        }
    }
    @Override
    public Map<String, Command> getAutonomousOptions()
    {
        return Map.of("Do nothing", new InstantCommand());
    }
    @Override
    public Map<String, Pose2d> getStartingLocations()
    {
        return Map.of("Unnecessary", new Pose2d());
    }
    @Override
    public void periodic()
    {
        field.setRobotPose(drive.getEstimatedPose());
    }
}
