package org.texastorque.auto.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.internal.DriverStationModeThread;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.texastorque.Field;
import org.texastorque.Subsystems;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.control.TorquePID;

public final class FollowEventPath extends TorqueCommand implements Subsystems {
    public static final double MAX_VELOCITY_PATH = 3.5, MAX_ACCELERATION_PATH = 3.5;
    
    private final PIDController xController = TorquePID.create(1).build();
    private final PIDController yController = TorquePID.create(1).build();

    private final PIDController thetaController;
    private final PPHolonomicDriveController controller;

    private final PathPlannerTrajectory trajectory;
    private final Timer timer = new Timer();

    private final List<EventMarker> unpassed, events;
    private final Map<String, TorqueCommand> commands;
    private final List<TorqueCommand> running;

    public FollowEventPath(final String name) {
        this(name, MAX_VELOCITY_PATH, MAX_ACCELERATION_PATH);
    }

    public FollowEventPath(final String name, final double maxSpeed, final double maxAcceleration) {
        this(name, new HashMap<String, TorqueCommand>(), maxSpeed, maxAcceleration);
    }

    public FollowEventPath(final String name, final Map<String, TorqueCommand> commands, final double maxSpeed, final double maxAcceleration) {
        thetaController = new PIDController(Math.PI * 2, 0, 0);

        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        thetaController.setTolerance(Units.degreesToRadians(2));
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        controller = new PPHolonomicDriveController(xController, yController, thetaController);

        trajectory = PathPlanner.loadPath(name, maxSpeed, maxAcceleration);
        events = trajectory.getMarkers();
        unpassed = new ArrayList<EventMarker>();
        this.commands = commands;
        running = new ArrayList<TorqueCommand>();

    }

    private final PathPlannerState reflect(final Trajectory.State state) {
        return PathPlannerTrajectory.transformStateForAlliance((PathPlannerState)state, DriverStation.getAlliance()); 
    }

    @Override
    protected final void init() {
        timer.reset();
        timer.start();

        drivebase.fieldMap.getObject("traj").setTrajectory(this.trajectory);

        PathPlannerServer.sendActivePath(this.trajectory.getStates());

        unpassed.clear();
        unpassed.addAll(events);
        running.clear();
    }

    @Override
    protected final void continuous() {
        final double elapsed = timer.get();
       
        final PathPlannerState desired = reflect(trajectory.sample(elapsed));

        final ChassisSpeeds speeds = controller.calculate(drivebase.getPose(), desired);
        
        drivebase.inputSpeeds = new ChassisSpeeds(-speeds.vxMetersPerSecond, -speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

        if (unpassed.size() > 0 && elapsed >= unpassed.get(0).timeSeconds) {
            final EventMarker marker = unpassed.remove(0);
            for (final String name : marker.names) {
                final TorqueCommand command = commands.getOrDefault(name, null);
                if (command != null)
                    running.add(command);
            }
        }

        for (int i = running.size() - 1; i >= 0; i--)
            if (running.get(i).run())
                running.remove(i);

        PathPlannerServer.sendPathFollowingData(new Pose2d(desired.poseMeters.getTranslation(), desired.holonomicRotation), drivebase.getPose());
    }

    @Override
    protected final boolean endCondition() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    @Override
    protected final void end() {
        timer.stop();
        for (final TorqueCommand command : running)
            command.reset();
        drivebase.inputSpeeds = new ChassisSpeeds();

        drivebase.fieldMap.getObject("traj").close();
    }

    public void addEvent(final String name, final TorqueCommand command) {
        commands.put(name, command);
    }
}