package frc.robot.commands;

import org.northernforce.commands.NFRSwerveModuleSetState;
import org.northernforce.subsystems.drive.NFRSwerveDrive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutoBalance extends CommandBase {
    private final NFRSwerveDrive drive;
    private final NFRSwerveModuleSetState[] setStateCommands;
    private final boolean optimize;
    private PIDController pid;

    public AutoBalance(NFRSwerveDrive drive, NFRSwerveModuleSetState[] setStateCommands, boolean optimize) {
        addRequirements(drive);
        this.drive = drive;
        this.setStateCommands = setStateCommands;
        this.optimize = optimize;
    }

    @Override
    public void initialize() {
        PIDController pid = new PIDController(0.1, 0, 0);
        pid.setSetpoint(0);
        pid.setTolerance(Math.toRadians(5));
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(pid.calculate(drive.getPitch()), 0, 0, drive.getRotation()); 
		SwerveModuleState[] states = drive.toModuleStates(speeds);
		for (int i = 0; i < states.length; i++){
			setStateCommands[i].setTargetState(optimize ? SwerveModuleState.optimize(states[i],
				drive.getModules()[i].getRotation()) : states[i]);
		}
    }

    @Override
    public void end(boolean interrupted) {
        pid.close();
    }

    @Override
    public boolean isFinished() {
        return pid.atSetpoint();
    }
}
