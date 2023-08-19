package frc.robot.robots;

import java.util.Map;

import org.northernforce.commands.NFRTankDriveStop;
import org.northernforce.commands.NFRTankDriveWithJoystick;
import org.northernforce.gyros.NFRNavX;
import org.northernforce.motors.NFRSparkMax;
import org.northernforce.subsystems.drive.NFRTankDrive;
import org.northernforce.subsystems.drive.NFRTankDrive.NFRTankDriveConfiguration;
import org.northernforce.util.NFRRobotContainer;

import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Xavier;

public class SquishyContainer implements NFRRobotContainer {
    private final NFRTankDrive drive;
    private final Field2d field;
    private final Xavier coprocessor;
    public SquishyContainer()
    {
        NFRTankDriveConfiguration config = new NFRTankDriveConfiguration(
            "drive",
            Units.inchesToMeters(24),
            11.74,
            Units.inchesToMeters(3),
            0.6,
            Units.lbsToKilograms(40),
            Units.feetToMeters(12),
            DCMotor.getNeo550(2)
        );
        NFRSparkMax leftSide = new NFRSparkMax(MotorType.kBrushless, 1, 3);
        NFRSparkMax rightSide = new NFRSparkMax(MotorType.kBrushless, 2, 4);
        leftSide.getPIDController().setP(0.2);
        rightSide.getPIDController().setP(0.2);
        leftSide.getPIDController().setI(0.001);
        rightSide.getPIDController().setI(0.001);
        rightSide.setInverted(false);
        leftSide.setInverted(true);
        NFRNavX navx = new NFRNavX();
        drive = new NFRTankDrive(config, leftSide, rightSide, navx);
        field = new Field2d();
        coprocessor = new Xavier(drive);
        Shuffleboard.getTab("Main").add("Xavier", coprocessor);
    }
    @Override
    public void bindOI(GenericHID driverHID, GenericHID manipulatorHID)
    {
        if (XboxController.class.isAssignableFrom(driverHID.getClass()) &&
            XboxController.class.isAssignableFrom(manipulatorHID.getClass()))
        {
            XboxController driverController = (XboxController)driverHID;
            drive.setDefaultCommand(new NFRTankDriveWithJoystick(
                drive,
                () -> {
                    return MathUtil.applyDeadband(-driverController.getLeftY(), 0.07);
                },
                () -> {
                    return MathUtil.applyDeadband(-driverController.getRightX(), 0.07);
                },
                0
            ));
            new JoystickButton(driverController, XboxController.Button.kB.value)
                .whileTrue(new NFRTankDriveStop(drive, 0.1));
        }
    }
    @Override
    public Map<String, Command> getAutonomousOptions()
    {
        return Map.of();
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
    @Override
    public Pair<String, Command> getDefaultAutonomous()
    {
        return Pair.of("Do Nothing", new InstantCommand());
    }
}
