package org.northernforce.cameras;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A NFRIsaacCamera is an interface meant for dealing with the isaac_ros_apriltag package.
 */
public class NFRIsaacCamera implements NFRCamera {
    /**
     * The default pipeline for the isaac ros camera. This is the only supported pipeline.
     */
    protected class DefaultPipeline implements ApriltagPipeline
    {
        protected AprilTagFieldLayout fieldLayout;
        /**
         * Gets the index of the pipeline.
         * @return the index of the pipeline (0 as this is the only pipeline)
         */
        @Override
        public int getIndex()
        {
            return 0;
        }
        /**
         * Gets the name of the pipeline.
         * @return the name of the pipeline (default as this is the only pipeline)
         */
        @Override
        public String getName()
        {
            return "default";
        }
        /**
         * Sets the current field layout for pose estimations
         * @param layout wpi AprilTagFieldLayout of apriltags
         */
        @Override
        public void setFieldLayout(AprilTagFieldLayout layout)
        {
            this.fieldLayout = layout;
        }
        /**
         * Gets the list of current estimations with timestamps
         * @return list of current estimations
         */
        @Override
        public ArrayList<Pair<Double, Pose2d>> getEstimations()
        {
            ArrayList<Pair<Double, Pose2d>> estimations = new ArrayList<>();
            double[] ids = table.getEntry("ids").getDoubleArray(new double[]{});
            double[] data = table.getEntry("data").getDoubleArray(new double[]{});
            for (int i = 0; i < ids.length; i++)
            {
                int id = (int)Math.round(ids[i]);
                double timestamp = data[i * 8];
                Transform3d transform = new Transform3d(
                    new Translation3d(
                        data[i * 8 + 1],
                        data[i * 8 + 2],
                        data[i * 8 + 3]
                    ),
                    new Rotation3d(
                        new Quaternion(
                            data[i * 8 + 4],
                            data[i * 8 + 5],
                            data[i * 8 + 6],
                            data[i * 8 + 7]
                        )
                    )
                );
                var tagPose = fieldLayout.getTagPose(id);
                if (tagPose.isPresent())
                {
                    var estimation = tagPose.get().transformBy(transform.plus(cameraOffset).inverse());
                    estimations.add(Pair.of(timestamp, estimation.toPose2d()));
                }
            }
            return estimations;
        }
    }
    protected final DefaultPipeline pipeline;
    protected final NetworkTable table;
    protected final Transform3d cameraOffset;
    /**
     * Creates a new NFRIsaacCamera
     * @param name the name that the camera is identified on networktables as
     * @param layout the layout of the apriltags
     * @param cameraOffset the camera offset from the center of the robot
     */
    public NFRIsaacCamera(String name, AprilTagFieldLayout layout, Transform3d cameraOffset)
    {
        pipeline = new DefaultPipeline();
        pipeline.setFieldLayout(layout);
        table = NetworkTableInstance.getDefault().getTable("ros/" + name);
        this.cameraOffset = cameraOffset;
    }
    /**
     * Returns a list of the avaliable pipelines.
     * @return a list of one apriltag pipeline
     */
    @Override
    public ArrayList<Pipeline> getPipelines()
    {
        return new ArrayList<>(List.of(pipeline));
    }
    /**
     * Sets the pipeline. Ignored as there is only one pipeline.
     * @param pipeline the pipeline
     */
    @Override
    public void setPipeline(Pipeline pipeline)
    {
    }
    /**
     * Gets the current pipeline.
     * @return the current apriltag pipeline
     */
    @Override
    public Pipeline getCurrentPipeline()
    {
        return pipeline;
    }
}
