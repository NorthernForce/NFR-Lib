package org.northernforce.util;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.Map;
import java.util.Scanner;
import java.util.function.Supplier;

/**
 * A robot chooser is a class that is meant for reading a file to select the robot.
 */
public class RobotChooser
{
    protected final Supplier<RobotContainer> defaultRobot;
    protected final Map<String, Supplier<RobotContainer>> otherRobots;
    protected final String robotNamePath;
    /**
     * Creates a new robot chooser.
     * @param defaultRobot default robot container should the file be nonexistant. Should be competition bot for
     * purposes of fallbacks should anything happen to the roboRio.
     * @param otherRobots the map of other robots.
     * @param robotNamePath the path of where to find the file.
     */
    public RobotChooser(Supplier<RobotContainer> defaultRobot, Map<String, Supplier<RobotContainer>> otherRobots,
        String robotNamePath)
    {
        this.defaultRobot = defaultRobot;
        this.otherRobots = otherRobots;
        this.robotNamePath = robotNamePath;
    }
    /**
     * Creates a new robot chooser. Looks to '/home/admin/robot_settings.txt' for robot name.
     * @param defaultRobot default robot container should the file be nonexistant. Should be competition bot for
     * purposes of fallbacks should anything happen to the roboRio.
     * @param otherRobots the map of other robots.
     */
    public RobotChooser(Supplier<RobotContainer> defaultRobot, Map<String, Supplier<RobotContainer>> otherRobots)
    {
        this(defaultRobot, otherRobots, "/home/admin/robot_settings.txt");
    }
    /**
     * Gets the current robot container by reading the name from the specified file.
     * @return the robot container using one of the suppliers.
     */
    public RobotContainer getRobotContainer()
    {
        try
        {
            File file = new File(robotNamePath);
            Scanner scanner = new Scanner(file);
            String robotName = scanner.next();
            scanner.close();
            if (!otherRobots.containsKey(robotName)) return defaultRobot.get();
            return otherRobots.get(robotName).get();
        }
        catch (FileNotFoundException e)
        {
            return defaultRobot.get();
        }
    }
}