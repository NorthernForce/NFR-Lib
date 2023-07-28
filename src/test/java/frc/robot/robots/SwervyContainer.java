package frc.robot.robots;

import java.util.Map;

import org.northernforce.commands.NFRSwerveDriveWithJoystick;
import org.northernforce.commands.NFRSwerveModuleSetState;
import org.northernforce.gyros.NFRNavX;
import org.northernforce.subsystems.drive.NFRSwerveDrive;
import org.northernforce.subsystems.drive.NFRSwerveDrive.NFRSwerveDriveConfiguration;
import org.northernforce.subsystems.drive.swerve.NFRSwerveModule;
import org.northernforce.util.NFRRobotContainer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
            NFRSwerveModule.createMk3Slow("Front Left", 1, 5, 9, false),
            NFRSwerveModule.createMk3Slow("Front Right", 2, 6, 10, true),
            NFRSwerveModule.createMk3Slow("Back Left", 3, 7, 11, false),
            NFRSwerveModule.createMk3Slow("Back Right", 4, 8, 12, true)
        };
        NFRSwerveDriveConfiguration driveConfig = new NFRSwerveDriveConfiguration("Swerve Drive");
        Translation2d[] offsets = new Translation2d[] {
            new Translation2d(0.581025, 0.581025),
            new Translation2d(0.581025, -0.581025),
            new Translation2d(-0.581025, 0.581025),
            new Translation2d(-0.581025, -0.581025)
        };
        NFRNavX gyro = new NFRNavX();
        Shuffleboard.getTab("Swerve").addNumber("Gyro", () -> gyro.getRotation2d().getDegrees())
            .withWidget(BuiltInWidgets.kGyro);
        gyro.reset();
        drive = new NFRSwerveDrive(driveConfig, modules, offsets, gyro);
        field = new Field2d();
        Shuffleboard.getTab("Swerve").add("Field", field);
        Shuffleboard.getTab("Swerve").add("Front Left", modules[0]);
        Shuffleboard.getTab("Swerve").add("Front Right", modules[1]);
        Shuffleboard.getTab("Swerve").add("Back Left", modules[2]);
        Shuffleboard.getTab("Swerve").add("Back Right", modules[3]);
        Shuffleboard.getTab("Swerve").addDouble("Front Left Angle", () -> drive.getModules()[0].getRotation().getDegrees())
            .withWidget(BuiltInWidgets.kGyro);
        Shuffleboard.getTab("Swerve").addDouble("Front Right Angle", () -> drive.getModules()[1].getRotation().getDegrees())
            .withWidget(BuiltInWidgets.kGyro);
        Shuffleboard.getTab("Swerve").addDouble("Back Left Angle", () -> drive.getModules()[2].getRotation().getDegrees())
            .withWidget(BuiltInWidgets.kGyro);
        Shuffleboard.getTab("Swerve").addDouble("Back Right Angle", () -> drive.getModules()[3].getRotation().getDegrees())
            .withWidget(BuiltInWidgets.kGyro);
    }
    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID)
    {
        NFRSwerveModuleSetState[] commands = new NFRSwerveModuleSetState[] {
            new NFRSwerveModuleSetState(drive.getModules()[0], 0,
                false),
            new NFRSwerveModuleSetState(drive.getModules()[1], 0,
                false),
            new NFRSwerveModuleSetState(drive.getModules()[2], 0,
                false),
            new NFRSwerveModuleSetState(drive.getModules()[3], 0,
                false)
        };
        if (driverHID instanceof XboxController && manipulatorHID instanceof XboxController)
        {
            XboxController driverController = (XboxController)driverHID;
            drive.setDefaultCommand(new NFRSwerveDriveWithJoystick(drive, commands,
                () -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                () -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                () -> -MathUtil.applyDeadband(driverController.getRightX(), 0.1),
                true, true));
        }
        else
        {
            drive.setDefaultCommand(new NFRSwerveDriveWithJoystick(drive, commands,
                () -> MathUtil.applyDeadband(driverHID.getRawAxis(1), 0.1),
                () -> MathUtil.applyDeadband(driverHID.getRawAxis(0), 0.1),
                () -> -MathUtil.applyDeadband(driverHID.getRawAxis(4), 0.1),
                true, true));
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
