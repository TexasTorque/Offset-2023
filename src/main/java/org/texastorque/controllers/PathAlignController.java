package org.texastorque.controllers;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.texastorque.Field;
import org.texastorque.Field.AprilTagType;
import org.texastorque.controllers.SwerveAlignController.AlignState;
import org.texastorque.controllers.SwerveAlignController.GridState;
import org.texastorque.controllers.SwerveAlignController.TranslationState;
import org.texastorque.torquelib.control.TorquePID;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class PathAlignController implements IAlignmentController{
    private final PIDController xController = TorquePID.create(1).build();
    private final PIDController yController = TorquePID.create(1).build();
    private final PIDController thetaController = new PIDController(Math.PI * 2, 0, 0);

    public boolean needsFieldRelative() { return false; }

    private final PPHolonomicDriveController controller;

    private AlignState alignment = AlignState.NONE;
    private GridState gridOverride = GridState.NONE;

    public void setAlignment(final AlignState alignment) {
        this.alignment = alignment;
    }

    public void setGridOverride(final GridState gridOverride) {
        this.gridOverride = gridOverride;
    }

    private final Supplier<Pose2d> poseSupplier;
    private final Runnable onFail;

    private PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();

    public PathAlignController(final Supplier<Pose2d> poseSupplier, final Runnable onFail) {
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        thetaController.setTolerance(Units.degreesToRadians(2));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        controller = new PPHolonomicDriveController(xController, yController, thetaController);

        this.poseSupplier = poseSupplier;
        this.onFail = onFail;

        setAlignment(AlignState.NONE);
        setGridOverride(GridState.NONE);

    }

    private int findClosestAprilTagID() {
        final Pose2d robotPose = poseSupplier.get(); 
        double currentClosestDistance = Double.MAX_VALUE; 
        int closestID = -1;
       
        for (final Map.Entry<Integer, Pose3d> aprilPose : Field.getAprilTagsMap().entrySet()) {

            final double distance = robotPose.getTranslation().getDistance(aprilPose.getValue().toPose2d().getTranslation());
            final int id = aprilPose.getKey();

            if (distance < currentClosestDistance && Field.OUR_TAG_IDS.containsKey(id)) {
                currentClosestDistance = distance;
                closestID = id;
            }
        }

        return closestID;
    } 

    private Optional<TranslationState> getTranslationState(final int closestID) {
        final AprilTagType tagType = Field.getAprilTagType(closestID);

        if (!tagType.isValid()) {
            onFail.run();
            return Optional.empty();
        }

        if (alignment == AlignState.CENTER)
            return Optional.of(TranslationState.GRID_CENTER);
        else if (alignment == AlignState.LEFT)
            return Optional.of(tagType.isGrid() ? TranslationState.GRID_LEFT : TranslationState.LOAD_ZONE_LEFT); 
        else if (alignment == AlignState.RIGHT)
            return  Optional.of(tagType.isGrid() ? TranslationState.GRID_RIGHT : TranslationState.LOAD_ZONE_RIGHT); 
        else
            return Optional.empty();
    }

    public void resetIf(final boolean notInLoop) {
        alignment = AlignState.NONE;
        gridOverride = GridState.NONE;

        if (notInLoop)
            trajectory = null;
    }

    public Optional<ChassisSpeeds> calculateAlignment() {
        if (alignment == AlignState.NONE) {
            onFail.run();
            return Optional.empty();
        }
        
        final Pose2d current = poseSupplier.get();

        if (trajectory == null) {
            final int closestID = (gridOverride == GridState.NONE)
                    ? findClosestAprilTagID()
                    : gridOverride.getID();

            final Pose3d aprilPose = Field.getAprilTagsMap().get(closestID);

            final Optional<TranslationState> translationState = getTranslationState(closestID);
            if (translationState.isEmpty()) {
                onFail.run();
                return Optional.empty();
            }

            final Pose2d goalPose = translationState.get().calculate(aprilPose);

            final double goalX = goalPose.getX(), goalY = goalPose.getY();
            final double offset = Units.inchesToMeters(12);

            trajectory = PathPlanner.generatePath(
                new PathConstraints(3.5, 4),
                new PathPoint(current.getTranslation(), Rotation2d.fromRadians(Math.PI), current.getRotation()),
                new PathPoint(new Translation2d(goalX + offset, goalY), Rotation2d.fromRadians(0), new Rotation2d(Math.PI)),
                new PathPoint(new Translation2d(goalX, goalY), Rotation2d.fromRadians(0), new Rotation2d(Math.PI)));

            timer.reset();
            timer.start();
        }

        final double elapsed = timer.get();
        final PathPlannerState desired = (PathPlannerState) trajectory.sample(elapsed);


        final boolean done = timer.hasElapsed(trajectory.getTotalTimeSeconds());

        SmartDashboard.putBoolean("Align Done", done);

        final ChassisSpeeds speeds = controller.calculate(current, desired);
        
        return Optional.of(new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond));
    }
}
