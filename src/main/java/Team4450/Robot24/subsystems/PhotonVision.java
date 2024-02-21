package Team4450.Robot24.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A class that wraps the PhotonVision system running on a coprocessor
 * and adds utility functions to support using vision.
 * Note: Communication between this class and the PV on the coprocessor
 * is handled through the Network Tables and the tables are wrapped by
 * by PhotonLib.
 */
public class PhotonVision extends SubsystemBase
{
    private PhotonCamera            camera;
    private PhotonPipelineResult    latestResult;

    private VisionLEDMode           ledMode = VisionLEDMode.kOff;

    private VisionSystemSim         visionSim;

    private Field2d                 field = new Field2d();

    private final AprilTagFields    fields = AprilTagFields.k2024Crescendo;
    private AprilTagFieldLayout     fieldLayout;
    private PhotonPoseEstimator     poseEstimator;

    private Transform3d             robotToCam;
    private PipelineType            pipelineType;
    private boolean defaultHasID()
    {
        return pipelineType == PipelineType.APRILTAG_TRACKING;
    }

    public static enum PipelineType {APRILTAG_TRACKING, OBJECT_TRACKING};

    /**
     * Create an instance of PhotonVision class for a camera.
     * @param cameraName The name of the camera.
     */
    public PhotonVision(String cameraName, PipelineType pipelineType) {
        this(cameraName, pipelineType, new Transform3d());
    }
    /**
     * Create an instance of PhotonVision class for a camera.
     * @param cameraName The name of the camera.
     * @param robotToCam A Transform3d locating the camera on the robot chassis.
     */
	public PhotonVision(String cameraName, PipelineType pipelineType, Transform3d robotToCam)
	{
        camera = new PhotonCamera(cameraName);
        this.robotToCam = robotToCam;
        fieldLayout = fields.loadAprilTagLayoutField();

        // adds a simulated camera to the vision sim: "real" camera will
        // act just like normal on real robot and in sim!
        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim(cameraName);
            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            PhotonCameraSim cameraSim = new PhotonCameraSim(camera, cameraProp);
            cameraSim.enableDrawWireframe(true);
            visionSim.addCamera(cameraSim, robotToCam);
        }

        selectPipeline(pipelineType);

        if (RobotBase.isSimulation()) setUpSimTargets();    // Must follow pipeline selection.

        // setup the AprilTag pose etimator.
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            camera,
            robotToCam
        );

        setLedMode(ledMode);

		Util.consoleLog("PhotonVision (%s) created!", cameraName);
        SmartDashboard.putData(field);
	}

    /**
     * sets up simulation targets for simulated vision system
     */
    private void setUpSimTargets() {
        visionSim.clearAprilTags();
        visionSim.clearVisionTargets();

        switch (pipelineType) {
            case APRILTAG_TRACKING:
                visionSim.addAprilTags(fieldLayout);
                break;

            case OBJECT_TRACKING:
                // approximate coordinates on on-field notes
                addNoteSimTarget(2.9, 7, 0);
                addNoteSimTarget(2.9, 5.6, 1);
                addNoteSimTarget(2.9, 4.1, 2);
                addNoteSimTarget(8.3, 7.5, 3);
                addNoteSimTarget(8.3, 5.8, 4);
                addNoteSimTarget(8.3, 4.1, 5);
                addNoteSimTarget(8.3, 2.4, 6);
                addNoteSimTarget(8.3, 0.75, 7);
                addNoteSimTarget(13.65, 7, 8);
                addNoteSimTarget(13.65, 5.6, 9);
                addNoteSimTarget(13.65, 4.1, 10);

                // test note
                // addNoteSimTarget(5, 5, 10);
                break;
        }
    }

    /**
     * adds a sim target of a Note
     * @param x field coordinate X
     * @param y field coordinate Y
     * @param id zero-based index of note
     */
    private void addNoteSimTarget(double x, double y, int id) {
        TargetModel noteModel = new TargetModel(0.3556, 0.3556, 0.0508);
        
        VisionTargetSim target = new VisionTargetSim(
            new Pose3d(new Pose2d(x, y, new Rotation2d())),
            noteModel
        );
        
        visionSim.addVisionTargets("note"+Integer.toString(id), target);
    }

    @Override
    public void simulationPeriodic() {
        if (pipelineType == PipelineType.OBJECT_TRACKING) {
            // this stuff allows us to drag around the note in simgui
            // to change position of the note
            for (int noteID = 0; noteID < 11; noteID++) {
                String name = "note" + Integer.toString(noteID);
                Pose2d fieldPose = visionSim.getDebugField().getObject(name).getPose();
                if (fieldPose.getX() == 0 && fieldPose.getY() == 0) continue;
                visionSim.getVisionTargets(name).forEach((target)->target.setPose(new Pose3d(fieldPose)));
            }
        }
    }

    /**
     * Updates the simulated pose of the robot for use in the PhotonVision
     * simulated vision code.
     * @param pose The pose of robot (ONLY BASED OFF OF ODOMETRY: NOT ANY POSE ESTIMATORS).
     */
    public void updateSimulationPose(Pose2d pose) {
        visionSim.update(pose);
    }

    /**
     * Get the lastest target results object returned by the camera.
     * @return Results object.
     */
    public PhotonPipelineResult getLatestResult()
    {
        latestResult = camera.getLatestResult();

        return latestResult;
    }

    /**
     * Indicates if lastest camera results list contains targets. Must 
     * call getLatestResult() before calling.
     * @return True if targets available, false if not.
     */
    public boolean hasTargets()
    {
        getLatestResult();

        return latestResult.hasTargets();
    }

    /**
     * Returns the target with the given Fiducial ID.
     * @param id the desired Fiducial ID.
     * @return The target or null if the ID is not currently being tracked.
     */
    public PhotonTrackedTarget getTarget(int id)
    {
        if (hasTargets() && isFiducialIDValid(id)) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();

            for (int i = 0; i < targets.size(); i++) {
                PhotonTrackedTarget target = targets.get(i);
                if (target.getFiducialId() == id) return target;
            }
        }
        
        return null;
    }

    /**
     * returns the closes target to center of camera crosshair (yaw-wise)
     * @return the raw PhotonTrackedTarget
     */
    public PhotonTrackedTarget getClosestTarget() {
        PhotonTrackedTarget closest;

        if (hasTargets()) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();
            closest = targets.get(0);

            for (int i = 0; i < targets.size(); i++) {
                if (Math.abs(targets.get(i).getYaw()) < Math.abs(closest.getYaw()))
                    closest = targets.get(i);
            }

            return closest;
        }
        else
            return null;
    }
    
    /**
     * Get an array of the currently tracked Fiducial IDs.
     * 
     * @return An ArrayList of the tracked IDs.
     */
    public ArrayList<Integer> getTrackedIDs()
    {
        ArrayList<Integer> ids = new ArrayList<Integer>();

        if (hasTargets()) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();

            for (int i = 0; i < targets.size(); i++) {
                ids.add(targets.get(i).getFiducialId());
            }
        }

        return ids;
    }
    /**
     * Get an array of the currently tracked valid Fiducial IDs
     * @return an ArrayList of the tracked valid IDs
     */
    public ArrayList<Integer> getTrackedValidIDs()
    {
        ArrayList<Integer> ids = getTrackedIDs();
        ids.removeIf(id -> !isFiducialIDValid(id));
        return ids;
    }
    /**
     * Get an array of the currently tracked invalid Fiducial IDs
     * @return an ArrayList of the tracked invalid IDs
     */
    public ArrayList<Integer> getTrackedInvalidIDs()
    {
        ArrayList<Integer> ids = getTrackedIDs();
        ids.removeIf(id -> isFiducialIDValid(id));
        return ids;
    }

    /**
     * Checks whether or not the camera currently sees a target
     * with the given Fiducial ID.
     * 
     * @param id The Fiducial ID.
     * @return True if the camera sees the ID.
     */
    public boolean hasTarget(int id)
    {
        return getTrackedIDs().contains(id);
    }

    // Best Target Methods =============================================================

    /**
     * Returns the yaw angle of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target yaw value from straight ahead or zero. -yaw means
     * target is left of robot center. (degrees)
     */
    public double getYaw()
    {
        PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
        return bestTarget == null ? 0 : getYaw(bestTarget.getFiducialId());
    }
    /**
     * Returns the yaw angle of the fiducialID target if seen. 
     * Must call hasTargets() before calling this function.
     * @param fiducialID
     * @return yaw of best target (tag) with fiducialID if valid and seen,
     * yaw of best target (other) if invalid and seen, otherwise 0.
     */
    public double getYaw(int fiducialID)
    {
        boolean validFiducialID = isFiducialIDValid(fiducialID);
        PhotonTrackedTarget target;

        if (validFiducialID && hasTarget(fiducialID))
            target = getTarget(fiducialID);
        else if (!validFiducialID && hasTargets())
            target = getBestTarget(false).orElse(null);
        else target = null;
        
        return target != null ? target.getYaw() : 0;
    }

    /**
     * Returns the area of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target area value. If no target exists, 0.
     */
    public double getArea()
    {
        PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
        return bestTarget == null ? 0 : getArea(bestTarget.getFiducialId());
    }
    /**
     * Returns the area of the fiducialID target if seen.
     * Must call hasTargets() before calling this function.
     * @param fiducialID
     * @return area of best target (tag) with fiducialID if valid and seen,
     * area of best target (other) if invalid and seen, otherwise 0.
     */
    public double getArea(int fiducialID)
    {
        boolean validFiducialID = isFiducialIDValid(fiducialID);
        PhotonTrackedTarget target;

        if (validFiducialID && hasTarget(fiducialID))
            target = getTarget(fiducialID);
        else if (!validFiducialID && hasTargets())
            target = getBestTarget(false).orElse(null);
        else target = null;
        
        return target != null ? target.getArea() : 0;
    }
    /**
     * Returns the pitch angle of the best target in the latest camera results
     * list. Must call hasTargets() before calling this function.
     * @return Best target pitch value from straight ahead or zero. -pitch means
     * target is below camera center. (degrees)
     */
    public double getPitch()
    {
        PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
        return bestTarget == null ? 0 : getPitch(bestTarget.getFiducialId());
    }
    /**
     * Returns the pitch angle of the fiducialID target if seen.
     * Must call hasTargets() before calling this function.
     * @param fiducialID
     * @return pitch angle of best target (tag) with fiducialID if valid and seen,
     * pitch angle of best target (other) if invalid and seen, otherwise 0.
     */
    public double getPitch(int fiducialID)
    {
        boolean validFiducialID = isFiducialIDValid(fiducialID);
        PhotonTrackedTarget target;

        if (validFiducialID && hasTarget(fiducialID))
            target = getTarget(fiducialID);
        else if (!validFiducialID && hasTargets())
            target = getBestTarget(false).orElse(null);
        else target = null; 
        
        return target != null ? target.getPitch() : 0;
    }

    /**
     * Get best target that either has or does not have an ID, depending on pipelineType
     * @param hasID
     * @return optional of best valid target if tracking april tags, otherwise best
     * invalid target if tracking notes. If none qualify, return Optional.empty()
     */
    public Optional<PhotonTrackedTarget> getBestTarget()
    {
        return getBestTarget(defaultHasID());
    }
    /**
     * Get best target that either has or does not have an ID
     * @param hasID
     * @return optional of best valid target if hasID is true, otherwise best
     * invalid target if hasID is false. If none qualify, return Optional.empty()
     */
    public Optional<PhotonTrackedTarget> getBestTarget(boolean hasID)
    {
        if (hasTargets())
        {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();
            for (int bestTargetIndex = 0; bestTargetIndex < targets.size(); bestTargetIndex++) {
                int targetID = targets.get(bestTargetIndex).getFiducialId();
                if (isFiducialIDValid(targetID) == hasID)
                    return Optional.of(targets.get(bestTargetIndex));
            }
        }

        return Optional.empty();
    }

    /**
     * Returns the Fiducial ID of the current best target, you should call
     * hasTargets() first!
     * @return The ID or -1 if no targets.
     */
    public int getFiducialID()
    {
        if (hasTargets()) 
            return latestResult.getBestTarget().getFiducialId();
        else
            return -1;
    }
    /**
     * Checks whether the id is within the valid range
     * @param id the id you are testing
     * @return whether or not the id is within the valid range
     */
    public boolean isFiducialIDValid(int id)
    {
        return id >= 0 && id <= 16;
    }
 
    // Utility Methods =============================================================

    /**
     * Select camera's image processing pipeline.
     * @param index Zero based number of desired pipeline.
     */
    public void selectPipeline(int index)
    {
        Util.consoleLog("%d", index);

        camera.setPipelineIndex(index);
    }

    /**
     * Sets the pipline of the photonvision camera given an enum type.
     * This also allows us to use simulation pretty well!
     * @param type The type of pipeline.
     */
    public void selectPipeline(PipelineType type) {
        pipelineType = type;
        if (RobotBase.isSimulation()) setUpSimTargets();
        selectPipeline(type.ordinal());
    }

    /**
     * Set the LED mode.
     * @param mode Desired LED mode.
     */
    public void setLedMode(VisionLEDMode mode)
    {
        Util.consoleLog("%d", mode.value);

        camera.setLED(mode);

        ledMode = mode;
    }

    /**
     * Toggle LED mode on/off.
     */
    public void toggleLedMode()
    {
        if (ledMode == VisionLEDMode.kOff)
            ledMode = VisionLEDMode.kOn;
        else
            ledMode = VisionLEDMode.kOff;
        
        setLedMode(ledMode);
    }

    /**
     * Save pre-processed image from camera stream.
     */
    public void inputSnapshot()
    {
        Util.consoleLog();

        camera.takeInputSnapshot();
    }

    /**
     * Save post-processed image from camera stream.
     */
    public void outputSnapshot()
    {
        Util.consoleLog();

        camera.takeOutputSnapshot();
    }
        
    @Override
	public void initSendable(SendableBuilder builder)
	{
        //super.initSendable(builder);
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty("has Targets", () -> hasTargets(), null);
        builder.addDoubleProperty("target yaw", () -> getYaw(), null);
        builder.addDoubleProperty("target pitch", () -> getPitch(), null);
        builder.addDoubleProperty("target area", () -> getArea(), null);
	}
    
    /**
     * Returns an Optional value of the robot's estimated 
     * field-centric pose given current tags that it sees.
     * (and also the timestamp)
     * 
     * @return The Optional estimated pose (empty optional means no pose or uncertain/bad pose).
     */
    public Optional<EstimatedRobotPose> getEstimatedPose()
    {
        Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update();
        
        if (estimatedPoseOptional.isPresent()) {
            EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
            Pose3d pose = estimatedPose.estimatedPose;

            // pose2d to pose3d (ignore the Z axis which is height off ground)
            Pose2d pose2d = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getAngle()));

            // update the field2d object in NetworkTables to visualize where the camera thinks it's at
            field.setRobotPose(pose2d);

            for (int i = 0; i < estimatedPose.targetsUsed.size(); i++)
                if (isFiducialIDValid(estimatedPose.targetsUsed.get(i).getFiducialId()))
                    return Optional.empty();

            return Optional.of(estimatedPose);
        } else return Optional.empty();
    }
    
    /**
     * returns the pose of the best target with valid or invalid ID
     * @return optional of pose of the best target with valid or invalid ID depending on
     * {@code defaultHasID()} 
     */
    public Optional<Pose3d> getBestTargetPose()
    {
        return defaultHasID() ? getBestTagPose() : getBestNotePose();
    }
    /**
     * returns the pose of the best target with a valid ID
     * @return optional of pose of best target with valid ID, empty if none exists
     */
    public Optional<Pose3d> getBestTagPose()
    {
        PhotonTrackedTarget target = getBestTarget(true).orElse(null);
        return target == null ? Optional.empty() : poseEstimator.getFieldTags().getTagPose(target.getFiducialId());
    }
    /**
     * returns the pose of the best target with an invalid ID
     * @return optional of pose of best target with invalid ID, empty if none exists
     */
    public Optional<Pose3d> getBestNotePose()
    {
        Optional<EstimatedRobotPose> optionalRobotWorldPose = getEstimatedPose();
        Optional<Transform3d> optionalNotePose = getRobotToNote();
        if (optionalRobotWorldPose.isEmpty() || optionalNotePose.isEmpty()) return Optional.empty();

        Pose3d robotWorldPose = optionalRobotWorldPose.get().estimatedPose;
        Transform3d robotToNote = optionalNotePose.get();
        
        Pose3d notePose = robotWorldPose.plus(robotToNote);
        return Optional.of(notePose);
    }

    /**
     * returns robot to best target
     * @return optional of Transform3d of robot to best target
     */
    public Optional<Transform3d> getRobotToBestTarget()
    {
        return getRobotToBestTarget(defaultHasID());
    }
    /**
     * returns robot to best valid or invalid target
     * @return optional of Transform3d of robot to best target, valid target if {@code hasID}
     * is true, false if not
     */
    public Optional<Transform3d> getRobotToBestTarget(boolean hasID)
    {
        PhotonTrackedTarget bestTarget = getBestTarget(hasID).orElse(null);
        return bestTarget == null ? Optional.empty() : getRobotToTarget(bestTarget);
    }
    /**
     * returns robot to target {@code target}
     * @param target
     * @return optional transform of robot to {@code target}
     */
    public Optional<Transform3d> getRobotToTarget(PhotonTrackedTarget target)
    {
        return Optional.of(target.getBestCameraToTarget().plus(robotToCam.inverse()));
    }
    /// THESE MAY BE USELESS IF robotToTarget() (getBestCameraToTarget) works as I hope it does
    public Optional<Transform3d> getRobotToTag()
    {
        Optional<EstimatedRobotPose> optionalWorldPose = getEstimatedPose();
        Optional<Pose3d> optionalTagPose = getBestTagPose();
        if (optionalWorldPose.isEmpty() || optionalTagPose.isEmpty()) return Optional.empty();

        Pose3d worldPose = optionalWorldPose.get().estimatedPose;
        Pose3d tagPose = optionalTagPose.get();

        Transform3d tagRelevantTransform = worldPose.minus(tagPose);
        return Optional.of(tagRelevantTransform);
    }
    public Optional<Transform3d> getRobotToNote()
    {
        if (hasTargets()) {
            EstimatedRobotPose robotWorldPose = getEstimatedPose().orElse(null);
            if (robotWorldPose == null) return Optional.empty();

            double bestTargetArea = getArea();
            // Unsure if objects size may get distorted when not looking directly at them, assuming they do not
            // and only distory base on size. If it ends up being true, will adjust once known how.
            double yaw = getYaw();

            final double WIDTH_OF_NOTE_MM = 14 * 25.4; // inch to millimeter
            final double CAM_FOCAL_LENGTH_MM = 50 * 10; // centimeter to millimeter
            final int CAM_TOTAL_PIXELS = 320 * 240;
            double pixelsTakenByNote = bestTargetArea * CAM_TOTAL_PIXELS;
            double robotToNoteMagnitude = (WIDTH_OF_NOTE_MM * CAM_FOCAL_LENGTH_MM) / pixelsTakenByNote;

            // gets a slight translation behind the robot (assuming .div works as I hope)
            Transform3d baseTransform = new Transform3d(robotWorldPose.estimatedPose, 
                                                        robotWorldPose.estimatedPose.div(-0.1));

            double baseTransformMagnitude = Math.sqrt(Math.pow(baseTransform.getX(), 2)
                                             + Math.pow(baseTransform.getY(), 2));
            double magnitudeMultiplier = 1 / baseTransformMagnitude * robotToNoteMagnitude;
            
            double cosRadians = Math.cos(yaw);
            double sinRadians = Math.sin(yaw);
            double adjustedX = baseTransform.getX() * cosRadians
                             - baseTransform.getY() * sinRadians
                             * magnitudeMultiplier;
            double adjustedY = baseTransform.getX() * sinRadians
                             + baseTransform.getY() * cosRadians
                             * magnitudeMultiplier;
            Transform3d robotToNoteTransform = new Transform3d(adjustedX,
                                                               adjustedY,
                                                               0,
                                                               new Rotation3d(0,
                                                                              0,
                                                                              yaw));

            return Optional.of(robotToNoteTransform);
        }
        else
            return Optional.empty();
    }
}