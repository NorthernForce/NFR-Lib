package frc.robot.robots;

import java.util.Map;
import java.util.Optional;

import org.northernforce.commands.NFRSwerveDriveCalibrate;
import org.northernforce.commands.NFRRunRollerIntake;
import org.northernforce.commands.NFRRotatingArmJointWithJoystick;
import org.northernforce.commands.NFRSwerveDriveStop;
import org.northernforce.commands.NFRSwerveDriveWithJoystick;
import org.northernforce.commands.NFRSwerveModuleSetState;
import org.northernforce.encoders.NFRCANCoder;
import org.northernforce.gyros.NFRNavX;
import org.northernforce.motors.NFRTalonFX;
import org.northernforce.subsystems.arm.NFRRollerIntake;
import org.northernforce.subsystems.arm.NFRRollerIntake.NFRRollerIntakeConfiguration;
import org.northernforce.motors.MotorEncoderMismatchException;
import org.northernforce.subsystems.arm.NFRRotatingArmJoint;
import org.northernforce.subsystems.arm.NFRRotatingArmJoint.NFRRotatingArmJointConfiguration;
import org.northernforce.subsystems.drive.NFRSwerveDrive;
import org.northernforce.subsystems.drive.NFRSwerveDrive.NFRSwerveDriveConfiguration;
import org.northernforce.subsystems.drive.swerve.NFRSwerveModule;
import org.northernforce.subsystems.ros.ROSCoprocessor;
import org.northernforce.subsystems.ros.ROSCoprocessor.ROSCoprocessorConfiguration;
import org.northernforce.util.NFRRobotContainer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.FieldConstants;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DropCubeAuto;
import frc.robot.commands.SwerveSideAuto;

public class SwervyContainer implements NFRRobotContainer
{
    private final NFRSwerveDrive drive;
    private final Field2d field;
    private final ROSCoprocessor coprocessor;
    private final NFRRollerIntake intake;
    private final NFRRotatingArmJoint rotatingJoint;
    private final NFRSwerveModuleSetState[] setStateCommands;
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
        NFRTalonFX intakeMotor = new NFRTalonFX(new TalonFXConfiguration(), 47);
        NFRNavX gyro = new NFRNavX();
        Shuffleboard.getTab("Swerve").addNumber("Gyro", () -> gyro.getRotation2d().getDegrees())
            .withWidget(BuiltInWidgets.kGyro);
        gyro.reset();
        drive = new NFRSwerveDrive(driveConfig, modules, offsets, gyro);
        field = new Field2d();
        intake = new NFRRollerIntake(new NFRRollerIntakeConfiguration("Roller Intake", -1), intakeMotor); //TODO set 1 to -1 if positve speed intakes lower absolute value if slower speed is required
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
        ROSCoprocessorConfiguration coprocessorConfig = new ROSCoprocessorConfiguration("xavier")
            .withHostname("northernforce-desktop")
            .withPort(5809);
        coprocessor = new ROSCoprocessor(coprocessorConfig);
        coprocessor.startConnecting();
        Shuffleboard.getTab("Main").add("Xavier", coprocessor);
        Shuffleboard.getTab("Swerve").add("Calibrate", new NFRSwerveDriveCalibrate(drive).ignoringDisable(true));
        NFRRotatingArmJointConfiguration rotatingJointConfiguration = new NFRRotatingArmJointConfiguration("rotatingJoint")
            .withUseLimits(false)
            .withUseIntegratedLimits(true)
            .withGearbox(DCMotor.getFalcon500(1))
            .withLimits(new Rotation2d(), new Rotation2d()) // TODO
            .withOriginOffset(new Transform3d(new Translation3d(10.463, 0, 10.777), new Rotation3d()));
        TalonFXConfiguration rotatingJointMotorConfiguration = new TalonFXConfiguration();
        rotatingJointMotorConfiguration.MotionMagic.MotionMagicAcceleration = 0; // TODO
        rotatingJointMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0; // TODO
        rotatingJointMotorConfiguration.Slot0.kP = 0.5; // TODO
        rotatingJointMotorConfiguration.Slot0.kI = 0; // TODO
        rotatingJointMotorConfiguration.Slot0.kD = 0; // TODO
        rotatingJointMotorConfiguration.Slot0.kV = 0; // TODO
        rotatingJointMotorConfiguration.Slot0.kS = 24; // TODO
        rotatingJointMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rotatingJointMotorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rotatingJointMotorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 0.125;
        rotatingJointMotorConfiguration.MotionMagic.MotionMagicAcceleration = 1;
        NFRTalonFX rotatingJointMotor = new NFRTalonFX(rotatingJointMotorConfiguration, 13);
        NFRCANCoder rotatingJointCANCoder = new NFRCANCoder(14);
        rotatingJointCANCoder.setRange(true);
        try
        {
            rotatingJointMotor.setSelectedEncoder(rotatingJointCANCoder);
        }
        catch (MotorEncoderMismatchException e)
        {
            e.printStackTrace();
        }
        rotatingJoint = new NFRRotatingArmJoint(rotatingJointConfiguration, rotatingJointMotor, Optional.empty());
        Shuffleboard.getTab("General").addDouble("Arm Angle", () -> rotatingJoint.getRotation().getDegrees());
        Shuffleboard.getTab("General").add("Reset CANCoder",
            Commands.runOnce(
                () -> rotatingJointCANCoder.setAbsoluteOffset(rotatingJointCANCoder.getAbsoluteOffset()
                    - rotatingJointCANCoder.getPosition())));
        setStateCommands = new NFRSwerveModuleSetState[] {
            new NFRSwerveModuleSetState(drive.getModules()[0], 0,
                false),
            new NFRSwerveModuleSetState(drive.getModules()[1], 0,
                false),
            new NFRSwerveModuleSetState(drive.getModules()[2], 0,
                false),
            new NFRSwerveModuleSetState(drive.getModules()[3], 0,
                false)
        };
    }
    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID)
    {
        XboxController manipulatorController = (XboxController)manipulatorHID;
        if (driverHID instanceof XboxController && manipulatorHID instanceof XboxController)
        {
            XboxController driverController = (XboxController)driverHID;
            drive.setDefaultCommand(new NFRSwerveDriveWithJoystick(drive, setStateCommands,
                () -> -MathUtil.applyDeadband(driverController.getLeftY(), 0.1),
                () -> -MathUtil.applyDeadband(driverController.getLeftX(), 0.1),
                () -> -MathUtil.applyDeadband(manipulatorController.getRightX(), 0.1),
                true, true));
            new JoystickButton(driverController, XboxController.Button.kB.value)
                .onTrue(Commands.runOnce(drive::clearRotation));
            new JoystickButton(driverController, XboxController.Button.kY.value)
                .onTrue(new NFRSwerveDriveStop(drive, setStateCommands, true));
        }
        else
        {
            drive.setDefaultCommand(new NFRSwerveDriveWithJoystick(drive, setStateCommands,
                () -> MathUtil.applyDeadband(driverHID.getRawAxis(1), 0.1),
                () -> MathUtil.applyDeadband(driverHID.getRawAxis(0), 0.1),
                () -> -MathUtil.applyDeadband(driverHID.getRawAxis(4), 0.1),
                true, true));
            new JoystickButton(driverHID, 5)
                .onTrue(Commands.runOnce(drive::clearRotation));
            new JoystickButton(driverHID, 1)
                .onTrue(new NFRSwerveDriveStop(drive, setStateCommands, true));
        }
        //outtake
        new Trigger(() -> Math.abs(manipulatorController.getLeftTriggerAxis()) >= 0.3)
            .whileTrue(new NFRRunRollerIntake(intake, 1, true));
            //intake
        new Trigger(() ->  Math.abs(manipulatorController.getRightTriggerAxis()) >= 0.3)
            .whileTrue(new NFRRunRollerIntake(intake, -1, true));
        rotatingJoint.setDefaultCommand(new NFRRotatingArmJointWithJoystick(rotatingJoint,
            () -> -MathUtil.applyDeadband(manipulatorController.getLeftY(), 0.1, 1)));
    }
    @Override
    public void setInitialPose(Pose2d pose)
    {
        drive.resetPose(pose);
    }
    @Override
    public Map<String, Command> getAutonomousOptions()
    {
        return Map.of("Haha you got no autonomous", new DropCubeAuto(rotatingJoint, intake),
            "Auto??", new SwerveSideAuto(drive, setStateCommands, rotatingJoint, intake));
    }
    @Override
    public Map<String, Pose2d> getStartingLocations()
    {
        return Map.of(
            "Blue Left", FieldConstants.BLUE_POSES[0],
            "Blue Center", FieldConstants.BLUE_POSES[1],
            "Blue Right", FieldConstants.BLUE_POSES[2],
            "Red Left", FieldConstants.RED_POSES[0],
            "Red Center", FieldConstants.RED_POSES[1],
            "Red Right", FieldConstants.RED_POSES[2]
        );
    }
    @Override
    public Pair<String, Command> getDefaultAutonomous()
    {
        return Pair.of("Do nothing", new InstantCommand());
    }
    @Override
    public void periodic()
    {
        field.setRobotPose(drive.getEstimatedPose());
    }
}
