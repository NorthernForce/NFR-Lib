// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.northernforce.commands.NFRRotatingArmJointSetAngle;
import org.northernforce.commands.NFRRunRollerIntake;
import org.northernforce.subsystems.arm.NFRRollerIntake;
import org.northernforce.subsystems.arm.NFRRotatingArmJoint;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropCubeAuto extends SequentialCommandGroup {
  /** Creates a new DropCubeAuto. */
  public DropCubeAuto(NFRRotatingArmJoint arm, NFRRollerIntake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new NFRRotatingArmJointSetAngle(arm, Rotation2d.fromDegrees(-50), Rotation2d.fromDegrees(5), 0, true),
      new NFRRunRollerIntake(intake, -0.75, true).withTimeout(0.5)
    );
  }
}
