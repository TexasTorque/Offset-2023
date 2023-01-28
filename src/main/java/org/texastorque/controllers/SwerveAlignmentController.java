package org.texastorque.controllers;

import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

import org.texastorque.Field;
import org.texastorque.Field.AprilTagType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class SwerveAlignmentController implements IController {

    private static final TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5, 3.5);
    private static final TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3.5, 3.5);
    public static final TrapezoidProfile.Constraints OMEGA_CONSTRAINTS =   new TrapezoidProfile.Constraints(4 * Math.PI, 4 * Math.PI);    

    private final ProfiledPIDController xController = new ProfiledPIDController(3, 0, 0, X_CONSTRAINTS);
    private final ProfiledPIDController yController = new ProfiledPIDController(3, 0, 0, Y_CONSTRAINTS);
    private final ProfiledPIDController omegaController = new ProfiledPIDController(4 * Math.PI, 0, 0, OMEGA_CONSTRAINTS);

    private int closestID = -1;

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

    public SwerveAlignmentController(final Supplier<Pose2d> poseSupplier, final Runnable onFail) {
        this.poseSupplier = poseSupplier;
        this.onFail = onFail;

        setAlignment(AlignState.NONE);
        setGridOverride(GridState.NONE);

        configurePIDControllerTolerance();
    }

    private void configurePIDControllerTolerance() {
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        omegaController.setTolerance(Units.degreesToRadians(1));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);
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

    private ChassisSpeeds calculateChassisSpeedsAlignment(final Pose2d goalPose) {

  SmartDashboard.putString("ALIGN", alignment.toString());
        SmartDashboard.putString("GRID", gridOverride.toString());

        final Pose2d robotPose = poseSupplier.get();

        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        omegaController.setGoal(goalPose.getRotation().plus(new Rotation2d(Math.PI)).getRadians());

        final boolean xAtGoal = xController.atGoal();
        final boolean yAtGoal = yController.atGoal();
        final boolean omegaAtGoal = omegaController.atGoal();

        final double xSpeed = xAtGoal ? 0 : -xController.calculate(robotPose.getX());
        final double ySpeed = yAtGoal ? 0 : -yController.calculate(robotPose.getY());
        final double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());

        // final double xSpeed =  -xController.calculate(robotPose.getX());
        // final double ySpeed =  -yController.calculate(robotPose.getY());
        // final double omegaSpeed =  omegaController.calculate(robotPose.getRotation().getRadians());
        
        return new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed);
    }

    public void resetIf(final boolean notInLoop) {
        alignment = AlignState.NONE;
        gridOverride = GridState.NONE;

        if (!notInLoop) return;

        closestID = -1;
        final Pose2d robotPose = poseSupplier.get();
        xController.reset(robotPose.getX());
        yController.reset(robotPose.getY());
        omegaController.reset(robotPose.getRotation().getRadians());
    }

    public Optional<ChassisSpeeds> calculateAlignment() {
        if (alignment == AlignState.NONE) {
            onFail.run();
            return Optional.empty();
        }

        if (closestID == -1) {
            closestID = (gridOverride == GridState.NONE)
                ? findClosestAprilTagID()
                : gridOverride.getID();
        }

        final Pose3d aprilPose = Field.getAprilTagsMap().get(closestID);

        final Optional<TranslationState> translationState = getTranslationState(closestID);
        if (translationState.isEmpty()) {
            onFail.run();
            return Optional.empty();
        }

        final Pose2d goalPose = translationState.get().calculate(aprilPose);

        return Optional.of(calculateChassisSpeedsAlignment(goalPose));        
    }


    public static double ALIGN_X_OFFSET_GRID = (15.589758 - 14.72);
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
