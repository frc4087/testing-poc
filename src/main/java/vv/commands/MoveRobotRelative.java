package vv.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static vv.commands.utils.VVPIDControllers.generalRotationController;
import static vv.commands.utils.VVPIDControllers.generalTranslationController;
import vv.config.VVConfig;
import vv.subsystems.drivetrain.DrivetrainSubsystem;

public class MoveRobotRelative extends Command {
    private final boolean debugLogging;
    private final DrivetrainSubsystem drivetrain;
    private final Transform2d transform;
    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController rotController;

    public MoveRobotRelative(VVConfig config, DrivetrainSubsystem drivetrain, Transform2d transform) {
        this(config, drivetrain, transform, false);
    }

    public MoveRobotRelative(VVConfig config, DrivetrainSubsystem drivetrain, Transform2d transform, boolean debugLogging) {
        this.drivetrain = drivetrain;
        this.transform = transform;
        this.debugLogging = debugLogging;

        this.xController = generalTranslationController(config);
        this.yController = generalTranslationController(config);
        this.rotController = generalRotationController(config);
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        if (debugLogging) {
            System.out.println("Initializing MoveRobotRelative command with transform: " + transform);
        }
        var targetPose = this.drivetrain.getState().Pose.transformBy(transform);
        xController.setGoal(new State(targetPose.getX(), 0));
        yController.setGoal(new State(targetPose.getY(), 0));
        rotController.setGoal(new State(targetPose.getRotation().getRadians(), 0));
    }

    @Override
    public void execute() {
        var currentPose = this.drivetrain.getState().Pose;
        var vx = xController.calculate(currentPose.getX());
        var vy = yController.calculate(currentPose.getY());
        var omega = rotController.calculate(currentPose.getRotation().getRadians());        
        drivetrain.driveFieldRelative(vx, vy, omega);

        SmartDashboard.putData("MovePIDX", xController);
        SmartDashboard.putData("MovePIDY", yController);
        SmartDashboard.putData("MovePIDRot", rotController);
    }

    @Override
    public void end(boolean interrupted) {
        if (debugLogging) {
            System.out.println("Ending MoveRobotRelative command, interrupted: " + interrupted);
        }
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        if (debugLogging) {
            System.out.println("Goals - x: " + xController.atGoal() + ", y: " + yController.atGoal() + ", rot: " + rotController.atGoal());
        }
        return xController.atGoal() && yController.atGoal() && rotController.atGoal();
    }
}
