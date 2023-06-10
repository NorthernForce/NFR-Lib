package org.northernforce.subsystems.arm;

import org.northernforce.motors.NFRMotorController;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class NFRArmMotorExtensionJoint extends NFRArmJoint{

    private final NFRArmMotorExtensionJointConfiguration config;
    private Transform3d currentComponentEndPosition;

    public NFRArmMotorExtensionJoint(NFRArmMotorExtensionJointConfiguration config) {
        super(config);
        this.config = config;
    }

    public static class NFRArmMotorExtensionJointConfiguration extends NFRArmJointConfiguration {
        /**
         * The motor (or motor group) that is responsible for extending the arm
         */
        protected NFRMotorController motor;
        /**
         * The distance from the end point of the retracted component of the arm to the end point of the extended component
         */
        protected double armLength;

        public NFRArmMotorExtensionJointConfiguration(String name) {
            super(name);
        }

        /**
         * Creates the configuration for an arm motor extension
         * @param name the name of the subsystem to create
         * @param staticOffset the offset from the end of the last component to this joint 
         * more explanation in the {@link NFRArmJointConfiguration} super class
         * @param motor the motor (or motor group) that will be extending or retracting the arm
         * @param armLength The distance from the end point of the retracted component of the arm 
         * to the end point of the extended component
         */
        public NFRArmMotorExtensionJointConfiguration(String name, Transform3d staticOffset, NFRMotorController motor, 
            double armLength) {
            super(name);
            super.staticOffset = staticOffset;
            this.motor = motor;
            this.armLength = armLength;
        }

        /**
         * set the staticOffset
         * @param staticOffset the offset from the end of the last component to this joint 
         * more explanation in the {@link NFRArmJointConfiguration} super class
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration setStartPosition(Transform3d staticOffset) {
            super.staticOffset = staticOffset;
            return this;
        }

        /**
         * set the motor
         * @param motor the motor (or motor group) that will be extending or retracting the arm
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration setMotor(NFRMotorController motor) {
            this.motor = motor;
            return this;
        }

        /**
         * set the arm length
         * @param armLength The distance from the end point of the retracted component of the arm 
         * to the end point of the extended component
         * @return returns this for chaining
         */
        public NFRArmMotorExtensionJointConfiguration setArmLength(double armLength) {
            this.armLength = armLength;
            return this;
        }
    }

    @Override
    public void periodic() {
        
    }

    @Override
    public Transform3d getEndState() {
        return config.staticOffset.plus(currentComponentEndPosition);
    }
    
}
