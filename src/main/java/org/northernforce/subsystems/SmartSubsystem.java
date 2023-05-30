package org.northernforce.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class SmartSubsystem extends SubsystemBase {
    protected SmartSubsystem()
    {
        super();
    }
    protected SmartSubsystem(String name)
    {
        super();
        setName(name);
    }
    public abstract Command getDefaultCommand();
}
