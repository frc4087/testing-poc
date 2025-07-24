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
import vv.config.VVConfig;
import vv.subsystems.drivetrain.Drivetrain;

public class DriverControls {
    private final CommandXboxController controller;
    private final XboxControllerSim sim;
    private final double maxLinearSpeed;
    private final double  maxRotationalSpeed;

    public DriverControls(
        VVConfig config, 
        Drivetrain drivetrain
    ) {
        var driverConfig = config.controllers().driver();
        this.maxLinearSpeed = config.drivetrain().constants().maxLinearSpeed().in(MetersPerSecond);
        this.maxRotationalSpeed = config.drivetrain().constants().maxRotationsPerSecond().in(RadiansPerSecond);

        controller = new CommandXboxController(driverConfig.port());
        drivetrain.setDefaultCommand(
			drivetrain.applyRequest(() -> new SwerveRequest.FieldCentric()
		        .withDeadband(maxLinearSpeed * driverConfig.translationalDeadband())
                .withRotationalDeadband(maxRotationalSpeed * driverConfig.rotationalDeadband())
		        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                .withVelocityX(drivetrain.clampLinearVelocity(-controller.getLeftY() * maxLinearSpeed))
				.withVelocityY(drivetrain.clampLinearVelocity(-controller.getLeftX() * maxLinearSpeed))
				.withRotationalRate(drivetrain.clampAngularVelocity(-controller.getRightX() * maxRotationalSpeed))
			)
            .withName("DriveWithController")
		);

        sim = new XboxControllerSim(controller.getHID());
    }

    public Command rumble() {
        return Commands.runEnd(
            () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.1),
            () -> controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)
        );
}

    public CommandXboxController controls() {
        return controller;
    }

    public void simulate(Consumer<XboxControllerSim> consumer) {
        consumer.accept(sim);
        sim.notifyNewData();
    }
}
