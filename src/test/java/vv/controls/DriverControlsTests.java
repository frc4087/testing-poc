package vv.controls;

import java.util.concurrent.TimeUnit;

import static org.awaitility.Awaitility.await;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import vv.subsystems.drivetrain.DrivetrainFactory;
import vv.subsystems.drivetrain.DrivetrainSubsystem;
import static vv.utils.TestSetup.CONFIG;
import static vv.utils.TestSetup.ROTATIONAL_VELOCITY_TEST_TOLERANCE_RAD_PER_SEC;
import static vv.utils.TestSetup.VELOCITY_TEST_TOLERANCE_MPS;
import static vv.utils.TestSetup.resetSimulationState;

public class DriverControlsTests {
    
    DrivetrainSubsystem drivetrain;
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
        driverControls.simulate((controller) -> {
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
    void verifyStickOutsideDeadband() {
        // Arrange
        var translationalDeadband = CONFIG.controllers().driver().translationalDeadband();
        var expectedVx = CONFIG.drivetrain().constants().maxLinearSpeed().in(MetersPerSecond);

        // Act - Within Deadband
        driverControls.simulate((controller) -> {
            controller.setLeftY(-1);
            controller.setLeftX(translationalDeadband*.1);
        });

        // Assert - No Movement
        assertEquals("DriveWithController", drivetrain.getDefaultCommand().getName());
        await().atMost(1, TimeUnit.SECONDS).untilAsserted(() -> {
            CommandScheduler.getInstance().run();
            var speeds = drivetrain.getState().Speeds;
            assertTrue(speeds.vxMetersPerSecond >= expectedVx, "X speed should be past deadband");
            assertEquals(0, speeds.vyMetersPerSecond, VELOCITY_TEST_TOLERANCE_MPS, "Y speed should be zero because of deadband");
            assertEquals(0, speeds.omegaRadiansPerSecond, ROTATIONAL_VELOCITY_TEST_TOLERANCE_RAD_PER_SEC, "Angular speed should be zero because no stick input");
        });
    }

}
