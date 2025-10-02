package vv.subsystems;

import java.util.concurrent.TimeUnit;

import static org.awaitility.Awaitility.await;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Rotation;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import vv.config.ArmConfig;
import vv.subsystems.arm.ArmSubsystem;
import vv.subsystems.arm.ArmSubsystem.ArmState;
import static vv.utils.TestSetup.CONFIG;
import static vv.utils.TestSetup.resetSimulationState;

@SuppressWarnings("unused")
public class ArmTests {
    
    ArmConfig config;
    ArmSubsystem arm;

    @BeforeEach
    void beforeEach() {
        resetSimulationState();
        config = CONFIG.arm();
        arm = new ArmSubsystem(CONFIG);
        CommandScheduler.getInstance().run();
    }

    @Test
    void testRunUpThenCancel() {
        // Arrange
        var runUpCmd = arm.runUp();

        // Act - Start
        runUpCmd.schedule();

        // Assert
        await().atMost(1, TimeUnit.SECONDS).untilAsserted(() -> {
            CommandScheduler.getInstance().run();

            assertEquals(ArmState.RUNNING_UP, arm.getState());
            assertEquals(config.resetSpeed(), arm.getMotor().getSetSpeed(), 0.01);
        });

        // Act - Cancel
        runUpCmd.cancel();
        CommandScheduler.getInstance().run();

        // Assert
        await().atMost(1, TimeUnit.SECONDS).untilAsserted(() -> {
            CommandScheduler.getInstance().run();

            assertEquals(ArmState.RESET, arm.getState());
            assertEquals(0, arm.getMotor().getSetSpeed(), 0.01);
        });
    }

    @Test
    void testSnapUp() {
        // Arrange
        var runUpCmd = arm.snapUp();

        // Act - Start
        runUpCmd.schedule();

        // Assert
        await().atMost(5, TimeUnit.SECONDS).untilAsserted(() -> {
            CommandScheduler.getInstance().run();

            assertEquals(ArmState.SNAP_UP, arm.getState());
            assertEquals(config.upSetpoint(), arm.getMotor().getPosition().in(Rotation), 0.01);
        });
    }

    @Test
    void testSnapDown() {
        // Arrange
        var runUpCmd = arm.snapDown();

        // Act - Start
        runUpCmd.schedule();

        // Assert
        await().atMost(5, TimeUnit.SECONDS).untilAsserted(() -> {
            CommandScheduler.getInstance().run();

            assertEquals(ArmState.SNAP_DOWN, arm.getState());
            assertEquals(config.downSetpoint(), arm.getMotor().getPosition().in(Rotation), 0.01);
        });
    }

}
