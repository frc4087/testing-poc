package vv.controls;

import java.util.concurrent.TimeUnit;

import static org.awaitility.Awaitility.await;
import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import vv.subsystems.drivetrain.Drivetrain;
import vv.subsystems.drivetrain.DrivetrainFactory;
import static vv.utils.TestSetup.CONFIG;
import static vv.utils.TestSetup.resetSimulationState;

public class DriverControlsTests {
    
    Drivetrain drivetrain;
    DriverControls driverControls;

    @BeforeEach
    @SuppressWarnings("unused")
    void beforeEach() {
        resetSimulationState();
        drivetrain = DrivetrainFactory.createDrivetrain(CONFIG);
        drivetrain.register();
        driverControls = new DriverControls(CONFIG, drivetrain);
    }

    @Test
    void verifySimControllerRelationship() {
        // Arrange
        var leftXTilt = 0.75;
        var leftYTilt = 0.25;

        // Act
        driverControls.simulateControls((controller) -> {
            controller.setAButton(true);
            controller.setBButton(false);
            controller.setLeftX(leftXTilt);
            controller.setLeftY(leftYTilt);
        });

        // Assert
        assertEquals(true, driverControls.controls().getHID().getAButton(), "A button should be pressed");
        assertEquals(false, driverControls.controls().getHID().getBButton(), "B button should not be pressed");
        assertEquals(leftXTilt, driverControls.controls().getLeftX(), "Left X tilt should match simulated value");
        assertEquals(leftYTilt, driverControls.controls().getLeftY(), "Left Y tilt should match simulated value");
    }

    @Test
    void verifyDeadband() {
        // Arrange
        var translationalDeadband = CONFIG.controllers().driver().translationalDeadband();
        var expectedSpeedPastDeadband = CONFIG.drivetrain().constants()
            .freeSpeedAt12Volts().in(MetersPerSecond) * translationalDeadband + 0.01;

        // Act
        driverControls.simulateControls((controller) -> {
            controller.setLeftX(-100);
            controller.setLeftY(-100);
            // controller.setLeftX(-translationalDeadband + 0.01);
            // controller.setLeftY(-translationalDeadband - 0.01);
        });
        System.err.println("Started: " + SimHooks.getProgramStarted());
        SimHooks.stepTiming(1);

        // Assert
        await().atMost(10, TimeUnit.SECONDS).untilAsserted(() -> {
            SimHooks.stepTiming(1);
            CommandScheduler.getInstance().run();
            var speeds = drivetrain.getState().Speeds;
            assertEquals(expectedSpeedPastDeadband, speeds.vxMetersPerSecond, "X speed should be past deadband");
            assertEquals(0, speeds.vyMetersPerSecond, "Y speed should be zero because of deadband");
            assertEquals(0, speeds.omegaRadiansPerSecond, "Angular speed should be zero because no stick input");
        });
    }

}
