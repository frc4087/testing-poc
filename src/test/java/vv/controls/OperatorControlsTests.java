package vv.controls;

import org.junit.jupiter.api.AfterEach;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import vv.config.RollerConfig;
import vv.subsystems.roller.RollerSubsystem;
import static vv.utils.TestSetup.CONFIG;
import static vv.utils.TestSetup.resetSimulationState;

@SuppressWarnings("unused")
public class OperatorControlsTests {
    
    RollerConfig config;
    RollerSubsystem roller;
    OperatorControls operatorControls;
    DriverControls driverControls;

    @BeforeEach
    void beforeEach() {
        resetSimulationState();
        config = CONFIG.roller();
        roller = new RollerSubsystem(CONFIG);
        roller.simulationPeriodic();
        driverControls = new DriverControls(CONFIG.controllers());
        operatorControls = new OperatorControls(CONFIG.controllers());
        
        operatorControls.setupTriggers(driverControls, roller);
    }

    @AfterEach
    void afterEach() throws Exception {
        roller.close();
    }

    @Test
    void operatorRightBumperPress() {
        // Act
        operatorControls.simulate((c) -> c.setRightBumperButton(true));
        CommandScheduler.getInstance().run();

        // Assert
        assertEquals(
            true,
            operatorControls.controls().getHID().getRightBumperButton(),
            "Operator controller should have right bumper pressed"
        );
        assertEquals(
            CONFIG.controllers().rumbleIntensity(),
            driverControls.sim().getRumble(GenericHID.RumbleType.kBothRumble),
            0.01,
            "Driver controller should be rumbling"
        );
        assertEquals(
            RollerSubsystem.RollerState.OUTPUTTING,
            roller.getState(),
            "Roller should be outputting"
        );
    }

    @Test
    void operatorRightBumperRelease() {
        // Arrange
        operatorControls.simulate((c) -> c.setRightBumperButton(true));
        CommandScheduler.getInstance().run();

        // Act
        operatorControls.simulate((c) -> c.setRightBumperButton(false));
        CommandScheduler.getInstance().run();

        // Assert
        assertEquals(
            false,
            operatorControls.controls().getHID().getRightBumperButton(),
            "Operator controller should not have right bumper pressed"
        );
        assertEquals(
            0,
            driverControls.sim().getRumble(GenericHID.RumbleType.kBothRumble),
            0.01,
            "Driver controller should not be rumbling"
        );

        assertEquals(
            RollerSubsystem.RollerState.IDLING,
            roller.getState(),
            "Roller should be idling"
        );
    }
}
