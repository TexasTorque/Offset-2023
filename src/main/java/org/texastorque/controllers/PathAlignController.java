/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.controllers;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.texastorque.Field;
import org.texastorque.Field.AprilTagType;
import org.texastorque.auto.commands.FollowEventPath;
import org.texastorque.torquelib.control.TorquePID;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public final class PathAlignController extends AbstractController<Optional<TorqueSwerveSpeeds>> {
    public static enum TranslationState {
        NONE(0, 0),
        GRID_CENTER(ALIGN_X_OFFSET_GRID, 0),
        GRID_RIGHT(ALIGN_X_OFFSET_GRID, -Units.inchesToMeters(22)),
        GRID_LEFT(ALIGN_X_OFFSET_GRID, Units.inchesToMeters(22)),
        LOAD_ZONE_RIGHT(ALIGN_X_OFFSET_LOAD_ZONE, -Units.inchesToMeters(30)),
        LOAD_ZONE_LEFT(ALIGN_X_OFFSET_LOAD_ZONE, Units.inchesToMeters(30));

        public Translation3d transl;
        private double x, y;

        private TranslationState(final double x, final double y) {
            this.x = x;
            this.y = y;
        }

        public Pose2d calculate(final Pose3d pose) {
            final Translation3d transl = new Translation3d(x * (pose.getX() > Field.FIELD_LENGTH / 2 ? -1 : 1), y, 0);
            return (new Pose3d(pose.getTranslation().plus(transl), pose.getRotation())).toPose2d();
        }
    }
    public static enum AlignState {
        NONE,
        CENTER,
        RIGHT,
        LEFT;
    }
    public static enum GridState {
        NONE(-1, -1),
        LEFT(1, 6),
        CENTER(2, 7),
        RIGHT(3, 8);

        private final int blueID, redID;

        private GridState(final int redID, final int blueID) {
            this.redID = redID;
            this.blueID = blueID;
        }

        public int getID() { return DriverStation.getAlliance() == DriverStation.Alliance.Blue ? blueID : redID; }
    }

    public static final PathConstraints MAX_PATH_CONSTRAINTS = new PathConstraints(FollowEventPath.MAX_VELOCITY_PATH, FollowEventPath.MAX_ACCELERATION_PATH);

    private static final double DISTANCE_TOLERANCE_TIGHT = Units.inchesToMeters(1);
    private static final double DISTANCE_TOLERANCE_LOOSE = Units.inchesToMeters(3);

    public static double ALIGN_X_OFFSET_GRID = (15.589758 - 14.72) - Units.inchesToMeters(4);

    public static double ALIGN_X_OFFSET_LOAD_ZONE = 1;

    private final PIDController xController = TorquePID.create(1).build();
    private final PIDController yController = TorquePID.create(1).build();

    private final PIDController thetaController = new PIDController(Math.PI * 2, 0, 0);
    private final PPHolonomicDriveController controller;

    private AlignState alignment = AlignState.NONE;

    private GridState gridOverride = GridState.NONE;

    private final Supplier<Pose2d> poseSupplier;

    private final Supplier<TorqueSwerveSpeeds> speedsSupplier;

    private PathPlannerTrajectory trajectory;

    private final Timer timer = new Timer();
    final double LAST_LEG_X_OFFSET_MAX = Units.inchesToMeters(12);

    final double LAST_LEG_X_OFFSET_MIN = Units.inchesToMeters(3);

    public PathAlignController(final Supplier<Pose2d> poseSupplier, final Supplier<TorqueSwerveSpeeds> speedsSupplier) {
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        thetaController.setTolerance(Units.degreesToRadians(2));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        controller = new PPHolonomicDriveController(xController, yController, thetaController);

        this.poseSupplier = poseSupplier;
        this.speedsSupplier = speedsSupplier;

        setAlignment(AlignState.NONE);
        setGridOverride(GridState.NONE);
    }

    public void setAlignment(final AlignState alignment) { this.alignment = alignment; }
    public void setGridOverride(final GridState gridOverride) { this.gridOverride = gridOverride; }

    public void resetIf(final boolean notInLoop) {
        alignment = AlignState.NONE;
        gridOverride = GridState.NONE;

        if (notInLoop) trajectory = null;
    }

    public boolean isDone() { return isDone(DISTANCE_TOLERANCE_LOOSE); }

    public Optional<TorqueSwerveSpeeds> calculate() {
        final Pose2d current = poseSupplier.get();

        if (trajectory == null) generateTrajectory(current);

        final double elapsed = timer.get();
        final PathPlannerState desired = (PathPlannerState)trajectory.sample(elapsed);

        final boolean done = timer.hasElapsed(trajectory.getTotalTimeSeconds());

        final TorqueSwerveSpeeds speeds = TorqueSwerveSpeeds.fromChassisSpeeds(controller.calculate(current, desired));

        return Optional.of(isSuperDone() ? new TorqueSwerveSpeeds() : speeds.times(-1, -1, 1));
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

    private int getTargetID() { return gridOverride == GridState.NONE ? findClosestAprilTagID() : gridOverride.getID(); }
    private Optional<TranslationState> getTranslationState(final int targetID) {
        final AprilTagType tagType = Field.getAprilTagType(targetID);

        if (alignment == AlignState.CENTER)
            return Optional.of(TranslationState.GRID_CENTER);
        else if (alignment == AlignState.LEFT)
            return Optional.of(tagType.isGrid() ? TranslationState.GRID_LEFT : TranslationState.LOAD_ZONE_LEFT);
        else if (alignment == AlignState.RIGHT)
            return Optional.of(tagType.isGrid() ? TranslationState.GRID_RIGHT : TranslationState.LOAD_ZONE_RIGHT);
        else
            return Optional.empty();
    }

    private boolean generateTrajectory(final Pose2d current) {
        final int targetID = getTargetID();

        final Pose3d aprilPose = Field.getAprilTagsMap().get(targetID);

        final Optional<TranslationState> translationState = getTranslationState(targetID);

        final Pose2d goalPose = translationState.get().calculate(aprilPose);

        final double offset = Math.min(Math.max(current.getX(), LAST_LEG_X_OFFSET_MIN), LAST_LEG_X_OFFSET_MAX);

        final double initialSpeed = speedsSupplier.get().getVelocityMagnitude();
        final Rotation2d initialHeading = speedsSupplier.get().getHeading();

        // https://github.com/Spectrum3847/2023-X-Ray/blob/78bb093b7675331c04ca4c66c1ebbebec51e0383/src/main/java/frc/robot/trajectories/commands/GeneratePath.java#L58

        final PathPoint startPoint = new PathPoint(current.getTranslation(), initialHeading, current.getRotation());//, initialSpeed);
        final PathPoint midPoint =
                new PathPoint(new Translation2d(goalPose.getX() + offset, goalPose.getY()), Rotation2d.fromRadians(Math.PI), new Rotation2d(Math.PI));
        final PathPoint endPoint = new PathPoint(goalPose.getTranslation(), Rotation2d.fromRadians(0), new Rotation2d(Math.PI), 0);

        trajectory = PathPlanner.generatePath(MAX_PATH_CONSTRAINTS, startPoint, midPoint, endPoint);
        
        // trajectory = PathPlanner.generatePath(MAX_PATH_CONSTRAINTS, midPoint, endPoint);

        timer.reset();
        timer.start();
        return true;
    }

    private final boolean isSuperDone() { return isDone(DISTANCE_TOLERANCE_TIGHT); }

    private boolean isDone(final double tolerance) {
        if (trajectory == null) return false;
        final Pose2d endPoint = trajectory.getEndState().poseMeters;
        return endPoint.getTranslation().getDistance(poseSupplier.get().getTranslation()) <= tolerance;
    }
}
