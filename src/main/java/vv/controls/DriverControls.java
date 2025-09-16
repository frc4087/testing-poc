package vv.controls;

import java.util.function.Consumer;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import vv.config.ControllersConfig;
import vv.config.VVConfig;
import vv.subsystems.drivetrain.DrivetrainSubsystem;

public class DriverControls {
    private final CommandXboxController controller;
    private final XboxControllerSim sim;
    private final double translationalDeadband;
    private final double rotationalDeadband;
    private final double rumbleIntensity;

    public DriverControls(ControllersConfig config) {
        controller = new CommandXboxController(config.driver().port());
        sim = new XboxControllerSim(controller.getHID());
        
        translationalDeadband = config.driver().translationalDeadband();
        rotationalDeadband = config.driver().rotationalDeadband();
        rumbleIntensity = config.rumbleIntensity();
    }

    public void setupTriggers(VVConfig config, DrivetrainSubsystem drivetrain) {
        var maxLinearSpeed = config.drivetrain().constants().maxLinearSpeed().in(MetersPerSecond);
        var maxRotationalSpeed = config.drivetrain().constants().maxRotationsPerSecond().in(RadiansPerSecond);

        drivetrain.setDefaultCommand(
			drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
		        .withDeadband(maxLinearSpeed * this.translationalDeadband)
                .withRotationalDeadband(maxRotationalSpeed * this.rotationalDeadband)
		        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(drivetrain.clampLinearVelocity(-controller.getLeftY() * maxLinearSpeed))
				.withVelocityY(drivetrain.clampLinearVelocity(-controller.getLeftX() * maxLinearSpeed))
				.withRotationalRate(drivetrain.clampAngularVelocity(-controller.getRightX() * maxRotationalSpeed))
			)
            .withName("DriveWithController")
		);
    }

    public Command rumble() {
        return Commands.runEnd(
            () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, this.rumbleIntensity),
            () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        );
}

    public CommandXboxController controls() {
        return controller;
    }

    public XboxControllerSim sim() {
        return sim;
    }

    public void simulate(Consumer<XboxControllerSim> consumer) {
        consumer.accept(sim);
        sim.notifyNewData();
    }
}
