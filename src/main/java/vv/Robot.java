package vv;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import vv.commands.MoveRobotRelative;
import vv.config.VVConfig;
import vv.subsystems.drivetrain.DrivetrainFactory;
import vv.subsystems.drivetrain.DrivetrainSubsystem;

public class Robot extends TimedRobot{
    
    public static Robot instance = null;
    private final VVConfig config;
    private final DrivetrainSubsystem drivetrain;

    private Robot() {
        config = VVConfig.readFromPath("./src/main/deploy/practice-robot.properties");
        drivetrain = DrivetrainFactory.createDrivetrain(config);
    }

    public static Robot start() {
        if (Robot.instance == null) {
            Robot.instance = new Robot();
            DataLogManager.start(); // Mirrors the NetworkTables-logged data to a file on disk
        }
        return Robot.instance;
    }

    @Override
    public void robotInit() {
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putNumber("PoseX", drivetrain.getState().Pose.getX());
        SmartDashboard.putNumber("PoseY", drivetrain.getState().Pose.getY());
        SmartDashboard.putNumber("Rotation", drivetrain.getState().Pose.getRotation().getDegrees());
    }

    @Override
    public void testInit() {
        var xFwd = new Transform2d(new Translation2d(1, 0), Rotation2d.kZero);
        var xBack = new Transform2d(new Translation2d(-1, 0), Rotation2d.kZero);
        Commands.sequence(
            new MoveRobotRelative(config, drivetrain, xFwd),
            new MoveRobotRelative(config, drivetrain, xBack)
        )
        .repeatedly()
        .schedule();
    }

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrain;
    }
  

}
