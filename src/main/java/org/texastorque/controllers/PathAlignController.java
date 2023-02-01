package org.texastorque.controllers;

import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.texastorque.Field;
import org.texastorque.Field.AprilTagType;
import org.texastorque.subsystems.Drivebase;
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

public final class PathAlignController extends AbstractController<Optional<ChassisSpeeds>> {
    private final PIDController xController = TorquePID.create(1).build();
    private final PIDController yController = TorquePID.create(1).build();
    private final PIDController thetaController = new PIDController(Math.PI * 2, 0, 0);

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
    private final DoubleSupplier speedSupplier;
    private final Supplier<Rotation2d> headingSupplier;

    private PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();

    public PathAlignController(final Supplier<Pose2d> poseSupplier, final DoubleSupplier speedSupplier, final Supplier<Rotation2d> headingSupplier) {
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        thetaController.setTolerance(Units.degreesToRadians(2));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        controller = new PPHolonomicDriveController(xController, yController, thetaController);

        this.poseSupplier = poseSupplier;
        this.speedSupplier = speedSupplier;
        this.headingSupplier = headingSupplier;

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

    private int getTargetID() {
        return gridOverride == GridState.NONE ? findClosestAprilTagID() : gridOverride.getID();
    }

    private Optional<TranslationState> getTranslationState(final int targetID) {
        final AprilTagType tagType = Field.getAprilTagType(targetID);

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

        if (notInLoop) trajectory = null;
    }

    final double LAST_LEG_X_OFFSET_MAX = Units.inchesToMeters(12);
    final double LAST_LEG_X_OFFSET_MIN = Units.inchesToMeters(3);

    private boolean generateTrajectory(final Pose2d current) {
        final int targetID = getTargetID();

        final Pose3d aprilPose = Field.getAprilTagsMap().get(targetID);

        final Optional<TranslationState> translationState = getTranslationState(targetID);

        final Pose2d goalPose = translationState.get().calculate(aprilPose);

        final double  offset = Math.min(Math.max(current.getX(), LAST_LEG_X_OFFSET_MIN), LAST_LEG_X_OFFSET_MAX);

        final PathPoint startPoint = new PathPoint(current.getTranslation(), headingSupplier.get(), current.getRotation(), speedSupplier.getAsDouble());
        final PathPoint midPoint = new PathPoint(new Translation2d(goalPose.getX() + offset, goalPose.getY()), Rotation2d.fromRadians(Math.PI), new Rotation2d(Math.PI), 3);
        final PathPoint endPoint = new PathPoint(goalPose.getTranslation(), Rotation2d.fromRadians(0), new Rotation2d(Math.PI));
       
        trajectory = PathPlanner.generatePath(
            new PathConstraints(3.5, 4),
            startPoint, midPoint, endPoint);


        timer.reset();
        timer.start();
        return true;
    }

    private static final double DISTANCE_TOLERANCE = Units.inchesToMeters(3);

    public boolean isDone() {
        if (trajectory == null) return false;
        final Pose2d endPoint = trajectory.getEndState().poseMeters;
        return endPoint.getTranslation().getDistance(poseSupplier.get().getTranslation()) <= DISTANCE_TOLERANCE;
    }

    public Optional<ChassisSpeeds> calculate() {
        final Pose2d current = poseSupplier.get();

        if (trajectory == null) generateTrajectory(current);

        final double elapsed = timer.get();
        final PathPlannerState desired = (PathPlannerState) trajectory.sample(elapsed);

        final boolean done = timer.hasElapsed(trajectory.getTotalTimeSeconds());

        SmartDashboard.putBoolean("Align Done", done);

        final ChassisSpeeds speeds = controller.calculate(current, desired);
        
        return Optional.of(new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond));
    }

    public static double ALIGN_X_OFFSET_GRID = (15.589758 - 14.72) - Units.inchesToMeters(4);
    public static double ALIGN_X_OFFSET_LOAD_ZONE = 1;

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
        NONE, CENTER, RIGHT, LEFT;
    }

    public static enum GridState {
        NONE(-1, -1), LEFT(1, 6), CENTER(2, 7), RIGHT(3, 8);

        private final int blueID, redID;

        private GridState(final int redID, final int blueID) {
            this.redID = redID;
            this.blueID = blueID;
        }

        public int getID() {
            return DriverStation.getAlliance() == DriverStation.Alliance.Blue ? blueID : redID;
        }
    }

}
