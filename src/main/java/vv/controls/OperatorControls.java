package vv.controls;

import java.util.function.Consumer;

import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import vv.config.ControllersConfig;
import vv.subsystems.arm.ArmSubsystem;
import vv.subsystems.roller.RollerSubsystem;

public class OperatorControls {
    private final CommandXboxController controller;
    private final XboxControllerSim sim;

    public OperatorControls(ControllersConfig config) {
        controller = new CommandXboxController(config.operator().port());
        sim = new XboxControllerSim(controller.getHID());
    }

    public void setupTriggers(DriverControls driverControls, RollerSubsystem roller, ArmSubsystem arm) {
        // Driver controls
        controller.rightBumper()
            .whileTrue(Commands.parallel(
                roller.output(), driverControls.rumble()))
            .onFalse(roller.idle());

        // Roller controls
        controller.leftBumper().whileTrue(roller.intake());

        // Arm controls
        controller.x().onTrue(arm.snapUp());
		controller.b().onTrue(arm.snapDown());
		controller.leftTrigger().whileTrue(arm.runUp());
    }

    public CommandXboxController controls() {
        return controller;
    }

    public void simulate(Consumer<XboxControllerSim> consumer) {
        consumer.accept(sim);
        sim.notifyNewData();
    }

}
