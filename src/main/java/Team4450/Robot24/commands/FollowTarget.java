package Team4450.Robot24.commands;

import Team4450.Lib.Util;
import Team4450.Robot24.subsystems.DriveBase;
import Team4450.Robot24.subsystems.PhotonVision;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;

public class FollowTarget extends Command {
    PIDController rotationController = new PIDController(0.01, 0, 0); // for rotating drivebase
    PIDController translationController = new PIDController(0.1, 0, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    PhotonVision photonVision;

    double maxSpeed;
    float distanceFromTarget;
    float slowdownDistance;

    TargetTypes targetType;
    FollowTypes followType;
    
    public enum TargetTypes { AprilTag, Note }
    public enum FollowTypes { LookAt, DriveTo, LookAndDrive }
    public boolean looksAtTarget()
    {
        return followType != FollowTypes.DriveTo;
    }
    public boolean drivesToTarget()
    {
        return followType != FollowTypes.LookAt;
    }

    /**
     * Drive to a target using functionality based on targetType
     * @param robotDrive the robot drive base
     * @param photonVision the photonvision subsystem
     * @param targetType whether a april tag or note is being followed
     * @param distanceFromTarget camera FOV percentage for note, otherwise magnitude distance
     * @param faceTarget whether the robot rotates to attempt to face the target, IGNORED FOR NOTE
     */
    public FollowTarget(DriveBase robotDrive,
                        PhotonVision photonVision,
                        float maxSpeed,
                        float distanceFromTarget,
                        float slowdownDistance,
                        TargetTypes targetType,
                        FollowTypes followType) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.maxSpeed = maxSpeed;
        this.distanceFromTarget = distanceFromTarget;
        this.slowdownDistance = slowdownDistance;
        this.targetType = targetType;
        this.followType = followType;

        if (drivesToTarget())
            addRequirements(robotDrive);
    }


    @Override
    public void initialize() {
        Util.consoleLog();

        rotationController.setSetpoint(0); // target should be at yaw=0 degrees
        rotationController.setTolerance(0.5); // within 0.5 degrees of 0

        translationController.setSetpoint(distanceFromTarget);
    }

    @Override
    public void execute() {
        Transform3d robotToPose = photonVision.getRobotToBestTarget().orElse(null);
        if (robotToPose == null) return;

        double robotToPoseMagnitude = Math.sqrt(Math.pow(robotToPose.getX(), 2)
                                             + Math.pow(robotToPose.getY(), 2));
        
        double normalizeMultiplier = 1 / robotToPoseMagnitude;
        double distanceMultiplier = robotToPoseMagnitude <= distanceFromTarget + slowdownDistance ?
                                    (robotToPoseMagnitude - distanceFromTarget) / slowdownDistance :
                                    1;
        double speedMultiplier = normalizeMultiplier * distanceMultiplier;
        double rot = looksAtTarget() ? rotationController.calculate(photonVision.getYaw()) : 0;

        if (drivesToTarget())
            robotDrive.driveRobotRelative(robotToPose.getX() * speedMultiplier,
                                          robotToPose.getY() * speedMultiplier,
                                          rot);
        // switch (targetType) {
        //     case AprilTag:
        //         followAprilTag();
        //         break;
        //     case Note:
        //         followNote();
        //         break;
        // }
    }
    /**
     * @deprecated
     */
    private void followAprilTag() {
        Transform3d robotToPose = photonVision.getRobotToTag().orElse(null);
        if (robotToPose == null) return;

        double robotToPoseMagnitude = Math.sqrt(Math.pow(robotToPose.getX(), 2)
                                             + Math.pow(robotToPose.getY(), 2));
        
        double speedMultiplier = translationController.calculate(1 / robotToPoseMagnitude * maxSpeed);
        double rot = looksAtTarget() ? rotationController.calculate(photonVision.getYaw()) : 0;

        if (drivesToTarget())
            robotDrive.driveRobotRelative(robotToPose.getX() * speedMultiplier,
                                          robotToPose.getY() * speedMultiplier,
                                          rot);
    }
    /**
     * @deprecated
     */
    private void followNote() {
        // make sure target centered before we move
        if (!rotationController.atSetpoint()) {
            double rotation = rotationController.calculate(photonVision.getYaw());
            robotDrive.driveRobotRelative(0, 0, rotation);
        }
        // otherwise drive to the target (only forwards backwards)
        else {
            double movement = translationController.calculate(photonVision.getPitch());
            robotDrive.driveRobotRelative(0, -movement, 0); // negative because camera backwards
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        
        robotDrive.driveRobotRelative(0, 0, 0);
    }
}

