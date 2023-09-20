// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.northernforce.commands;

import org.northernforce.subsystems.arm.NFRRollerIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** Add your docs here. */
public class NFRRunRollerIntake extends CommandBase {
    protected final NFRRollerIntake intake;
    protected final double speed;
    protected final boolean stopOnCommandEnd;

    /**
     * sets speed of roller intake (command will not stop motor)
     * @param intake the roller intake to set speed of 
     * @param speed the speed to set
    */
    public NFRRunRollerIntake(NFRRollerIntake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        this.stopOnCommandEnd = false;
    }

    /**
     * sets speed of roller intake
     * @param intake the roller intake to set speed of 
     * @param speed the speed to set
     * @param stopOnCommandEnd if true motor will stop on command end
    */
    public NFRRunRollerIntake(NFRRollerIntake intake, double speed, boolean stopOnCommandEnd) {
        this.intake = intake;
        this.speed = speed;
        this.stopOnCommandEnd = stopOnCommandEnd;
    }

    @Override
    public void initialize() {
        intake.setSpeed(speed);
    }

    @Override
    public void end(boolean interrupted) {
        if(stopOnCommandEnd) {
            intake.setSpeed(0);
        }
    }
    
}
