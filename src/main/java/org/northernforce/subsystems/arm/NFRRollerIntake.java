// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.northernforce.subsystems.arm;

import org.northernforce.motors.NFRMotorController;

import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class NFRRollerIntake extends NFRArmJoint {
    public static class NFRRollerIntakeConfiguration extends NFRArmJointConfiguration {
        protected double speedCoefficient = 1;

        public NFRRollerIntakeConfiguration(String name, double speedCoefficient) {
            super(name);
            this.speedCoefficient = speedCoefficient;
        }

        public NFRRollerIntakeConfiguration(String name) {
            super(name);
        }

        public NFRRollerIntakeConfiguration withSpeedCoefficient(double speedCoefficient) {
            this.speedCoefficient = speedCoefficient;
            return this;
        }
    }
    
    protected final NFRMotorController controller;
    protected final NFRRollerIntakeConfiguration config;

    public NFRRollerIntake(NFRRollerIntakeConfiguration config, NFRMotorController controller) {
        super(config);
        this.config = config;
        this.controller = controller;
    }

    public void setSpeed(double speed) {
        controller.set(speed * config.speedCoefficient);
    }

    public Transform3d getEndState() {
        return new Transform3d();
    }
}
