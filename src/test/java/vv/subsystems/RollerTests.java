package vv.subsystems;

import java.time.Duration;
import java.util.concurrent.TimeUnit;

import static org.awaitility.Awaitility.await;
import org.junit.jupiter.api.AfterEach;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import vv.config.RollerConfig;
import vv.subsystems.roller.RollerSubsystem;
import vv.subsystems.roller.RollerSubsystem.RollerState;
import static vv.utils.TestSetup.CONFIG;
import static vv.utils.TestSetup.resetSimulationState;

@SuppressWarnings("unused")
public class RollerTests {
    
    RollerConfig config;
    RollerSubsystem roller;

    @BeforeEach
    void beforeEach() {
        resetSimulationState();
        config = CONFIG.roller();
        roller = new RollerSubsystem(CONFIG);
        roller.simulationPeriodic();
    }

    @AfterEach
    void afterEach() throws Exception {
        roller.close();
    }

    @Test
    void testIdleBehavior() {
        // Act
        roller.idle().schedule();

        // Assert
        await().atMost(1, TimeUnit.SECONDS).untilAsserted(() -> {
            CommandScheduler.getInstance().run();

            assertEquals(RollerState.IDLING, roller.getState());
            assertEquals(config.motor().idleSpeed(), roller.getMotor().getSetSpeed());
            assertTrue(roller.getLED().getColor().isEmpty());    
        });
    }

    @Test
    void testIntakeBehavior() {
        // Act
        roller.intake().schedule();

        // Assert
        await().atMost(1, TimeUnit.SECONDS).untilAsserted(() -> {
            CommandScheduler.getInstance().run();

            assertEquals(RollerState.INTAKING, roller.getState());
            assertEquals(config.motor().intakeSpeed(), roller.getMotor().getSetSpeed(), 0.05);
            assertTrue(roller.getLED().getColor().isPresent());
            assertEquals(Color.kBlue, roller.getLED().getColor().get());
        });
    }


    @Test
    void testOutputBehavior() {
        // Act
        roller.output().schedule();

        // Assert
        await().atMost(1, TimeUnit.SECONDS).untilAsserted(() -> {
            CommandScheduler.getInstance().run();

            assertEquals(RollerState.OUTPUTTING, roller.getState());
            assertEquals(config.motor().outputSpeed(), roller.getMotor().getSetSpeed(), 0.05);
            assertTrue(roller.getLED().getColor().isPresent());
            assertEquals(Color.kGreen, roller.getLED().getColor().get());
        });
    }

    @Test
    void testTimedIntakeBehavior() {
        // Act
        roller.intake(Time.ofRelativeUnits(1, Seconds)).schedule();

        // Assert - Running
        await().atMost(Duration.ofSeconds(1)).until(() -> {
            CommandScheduler.getInstance().run();
            return roller.getState().equals(RollerState.INTAKING);
        });

        await().between(Duration.ofMillis(900), Duration.ofSeconds(2)).untilAsserted(() -> {
            CommandScheduler.getInstance().run();

            assertEquals(RollerState.IDLING, roller.getState());
            assertEquals(config.motor().idleSpeed(), roller.getMotor().getSetSpeed(), 0.05);
            assertTrue(roller.getLED().getColor().isEmpty());
        });
    }
}
