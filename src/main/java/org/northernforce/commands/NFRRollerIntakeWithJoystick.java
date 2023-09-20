// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.northernforce.commands;

import java.util.function.DoubleSupplier;

import org.northernforce.subsystems.arm.NFRRollerIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Add your docs here. */
public class NFRRollerIntakeWithJoystick extends CommandBase {
    protected final NFRRollerIntake intake;
    protected final DoubleSupplier speedSupplier;
    protected final boolean stopOnCommandEnd;

    /**
     * sets speed of roller intake (command will not stop motor)
     * @param intake the roller intake to set speed of 
     * @param speed the speed to set
    */
    public NFRRollerIntakeWithJoystick(NFRRollerIntake intake, DoubleSupplier speedSupplier) {
        this.intake = intake;
        this.speedSupplier = speedSupplier;
        this.stopOnCommandEnd = false;
    }

    /**
     * sets speed of roller intake
     * @param intake the roller intake to set speed of 
     * @param speed the speed to set
     * @param stopOnCommandEnd if true motor will stop on command end
    */
    public NFRRollerIntakeWithJoystick(NFRRollerIntake intake, DoubleSupplier speedSupplier, boolean stopOnCommandEnd) {
        this.intake = intake;
        this.speedSupplier = speedSupplier;
        this.stopOnCommandEnd = stopOnCommandEnd;
    }

    @Override
    public void execute() {
        intake.setSpeed(speedSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        if(stopOnCommandEnd) {
            intake.setSpeed(0);
        }
    }
}
