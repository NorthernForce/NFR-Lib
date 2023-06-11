package org.northernforce.cameras;

import java.util.ArrayList;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;

public interface NFRCamera
{
    public interface Pipeline
    {
        public enum Type
        {
            kDriverCamera,
            kApriltag,
            kReflectiveTape,
            kColoredShape
        }
        public int getIndex();
        public String getName();
        public Type getType();
    }
    public interface ApriltagPipeline extends Pipeline
    {
        @Override
        public default Type getType()
        {
            return Type.kApriltag;
        }
        public void setFieldLayout(AprilTagFieldLayout layout);
        public ArrayList<Pair<Double, Pose2d>> getEstimation();
    }
    public ArrayList<Pipeline> getPipelines();
    public void setPipeline(Pipeline pipeline);
    public Pipeline getCurrentPipeline(Pipeline pipeline);
}