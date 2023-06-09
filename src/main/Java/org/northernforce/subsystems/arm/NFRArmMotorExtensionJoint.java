package org.northernforce.subsystems.arm;

import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public class NFRArmMotorExtensionJoint extends NFRArmJoint{

    public NFRArmMotorExtensionJoint(NFRArmMotorExtensionJointConfiguration config) {
        super(config);
    }

    public static class NFRArmMotorExtensionJointConfiguration extends NFRArmJointConfiguration {
        protected Transform3d startPosition;

        public NFRArmMotorExtensionJointConfiguration(String name) {
            super(name);
        }

        public NFRArmMotorExtensionJointConfiguration(String name, Transform3d startPosition) {
            super(name);
            this.startPosition = startPosition;
        }

        public void setStartPosition(Transform3d startPosition) {
            this.startPosition = startPosition;
        }
    }

    @Override
    public Transform3d getEndState() {
        return null;
    }
    
}
