/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.commands;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import org.texastorque.Subsystems;
import org.texastorque.auto.EventMap;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public final class FollowEventPath extends TorqueCommand implements Subsystems {
    public static final double MAX_VELOCITY_PATH = 3.5, MAX_ACCELERATION_PATH = 3.5;
    // public static final double MAX_VELOCITY_PATH = 2, MAX_ACCELERATION_PATH = 2;
    // public static final double MAX_VELOCITY_PATH = .5, MAX_ACCELERATION_PATH = .5;

    private final PIDController xController = new PIDController(3, 0, 0);
    private final PIDController yController = new PIDController(3, 0, 0);

    private final PIDController omegaController;
    private final PPHolonomicDriveController controller;

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();

    private final List<EventMarker> unpassed, events;
    private final Map<String, TorqueCommand> commands;
    private final List<TorqueCommand> running;

    public FollowEventPath(final String name) { this(name, MAX_VELOCITY_PATH, MAX_ACCELERATION_PATH); }

    public FollowEventPath(final String name, final double maxSpeed, final double maxAcceleration) {
        this(name, EventMap.get(), maxSpeed, maxAcceleration);
    }

    public FollowEventPath(final String name, final Map<String, TorqueCommand> commands, final double maxSpeed, final double maxAcceleration) {
        omegaController = new PIDController(Math.PI * 2, 0, .0); //kP_intake = .4PI

        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        omegaController.setTolerance(Units.degreesToRadians(2));
        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        controller = new PPHolonomicDriveController(xController, yController, omegaController);

        trajectory = PathPlanner.loadPath(name, maxSpeed, maxAcceleration);
        events = trajectory.getMarkers();
        unpassed = new ArrayList<EventMarker>();
        this.commands = commands;
        running = new ArrayList<TorqueCommand>();

    }

    public void addEvent(final String name, final TorqueCommand command) { commands.put(name, command); }

    @Override
    protected final void init() {
        timer.reset();
        timer.start();

        drivebase.fieldMap.getObject("traj").setTrajectory(this.trajectory);

        PathPlannerServer.sendActivePath(this.trajectory.getStates());

        unpassed.clear();
        unpassed.addAll(events);
        running.clear();

        final Pose2d startingPose = reflect(trajectory.getInitialState()).poseMeters;
        drivebase.resetPose(new Pose2d(startingPose.getTranslation(), Rotation2d.fromRadians(Math.PI)));
    }

    @Override
    protected final void continuous() {
        final double elapsed = timer.get();

        final PathPlannerState desired = reflect(trajectory.sample(elapsed));

        final TorqueSwerveSpeeds speeds = TorqueSwerveSpeeds.fromChassisSpeeds(controller.calculate(drivebase.getPose(), desired));
        drivebase.inputSpeeds = speeds.times(-1.0, -1.0, 1.0);

        if (unpassed.size() > 0 && elapsed >= unpassed.get(0).timeSeconds) {
            final EventMarker marker = unpassed.remove(0);
            for (final String name : marker.names) {
                final TorqueCommand command = commands.getOrDefault(name, null);
                if (command != null) running.add(command);
            }
        }

        for (int i = running.size() - 1; i >= 0; i--)
            if (running.get(i).run()) running.remove(i);

        PathPlannerServer.sendPathFollowingData(new Pose2d(desired.poseMeters.getTranslation(), desired.holonomicRotation), drivebase.getPose());
    }

    @Override
    protected final boolean endCondition() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    protected final void end() {
        timer.stop();
        for (final TorqueCommand command : running) command.reset();
        for (final TorqueCommand command : commands.values()) command.reset();

        drivebase.inputSpeeds = new TorqueSwerveSpeeds();

        drivebase.fieldMap.getObject("traj").close();
    }

    private final PathPlannerState reflect(final Trajectory.State state) {
        return PathPlannerTrajectory.transformStateForAlliance((PathPlannerState)state, DriverStation.getAlliance());
    }
}