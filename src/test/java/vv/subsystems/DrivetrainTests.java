package vv.subsystems;

import java.util.concurrent.TimeUnit;

import static org.awaitility.Awaitility.await;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import vv.commands.MoveRobotRelative;
import vv.subsystems.drivetrain.Drivetrain;
import vv.subsystems.drivetrain.DrivetrainFactory;
import static vv.utils.TestSetup.CONFIG;
import static vv.utils.TestSetup.POSITION_TEST_TOLERANCE;
import static vv.utils.TestSetup.ROTATIONAL_TEST_TOLERANCE_DEG;
import static vv.utils.TestSetup.isFinished;
import static vv.utils.TestSetup.maxSimulationTime;
import static vv.utils.TestSetup.resetSimulationState;

public class DrivetrainTests {

    Drivetrain drivetrain;

    @BeforeEach
    @SuppressWarnings("unused")
    void beforeEach() {
        resetSimulationState();
        drivetrain = DrivetrainFactory.createDrivetrain(CONFIG);
        drivetrain.register();
    }

    @Test
    void canLoadFromConfig() {
        assertEquals(4, drivetrain.getModules().length);
        assertNotNull(drivetrain.getPigeon2());
        assertNotNull(drivetrain.getKinematics());
        assertEquals(0, drivetrain.getState().Pose.getMeasureX().baseUnitMagnitude(), POSITION_TEST_TOLERANCE,  "Initial X position should be 0");
        assertEquals(0, drivetrain.getState().Pose.getMeasureY().baseUnitMagnitude(),  POSITION_TEST_TOLERANCE, "Initial Y position should be 0");
        assertEquals(0, drivetrain.getState().Pose.getTranslation().getNorm(), POSITION_TEST_TOLERANCE, "Initial translation should be 0");
        assertEquals(0, drivetrain.getState().Pose.getRotation().getRadians(), "Initial rotation should be 0");
    }

    @Test
    @Disabled
    /**
     * Enable this test to see the inaccuracy of the open-loop request.
     * See {@link DrivetrainTests#testMoveForward1MeterCloseLoop} for an
     * example of a closed-loop request that works correctly.
     */
    void testMoveForward1MeterOpenLoop() {        
        // Arrange
        var targetDistance = 1;
        SwerveRequest forwardRequest = new SwerveRequest.RobotCentric()
            .withVelocityX(1.0)
            .withVelocityY(0.0)  
            .withRotationalRate(0.0);

        Command cmd = new TestCommand(drivetrain, forwardRequest).withTimeout(targetDistance);
        double startX = drivetrain.getState().Pose.getMeasureX().baseUnitMagnitude();
        double startY = drivetrain.getState().Pose.getMeasureY().baseUnitMagnitude();
        double startRotation = drivetrain.getState().Pose.getRotation().getDegrees();

        // Act
        drivetrain.logPose();
        CommandScheduler.getInstance().schedule(cmd);
        await().atMost(maxSimulationTime(targetDistance), TimeUnit.SECONDS).until(() -> {
            CommandScheduler.getInstance().run();
            return isFinished(cmd);
        });
        drivetrain.logPose();

        // Assert
        double finalX = drivetrain.getState().Pose.getMeasureX().baseUnitMagnitude();
        double finalY = drivetrain.getState().Pose.getMeasureY().baseUnitMagnitude();
        double finalRotation = drivetrain.getState().Pose.getRotation().getDegrees();
        
        System.out.println("Rel Error X: " + (finalX - startX - targetDistance)/targetDistance * 100);

        assertEquals(targetDistance, Math.abs(finalX - startX), POSITION_TEST_TOLERANCE, "Didn't move far enough in X");
        assertEquals(startY, finalY, POSITION_TEST_TOLERANCE, "Should not have moved in Y");
        assertEquals(startRotation, finalRotation, ROTATIONAL_TEST_TOLERANCE_DEG, "Should not have rotated");    
    }

    /**
     * A closed-loop version of {@link DrivetrainTests#testMoveForward1MeterOpenLoop}.
     * This uses default PID controllers in {@link MoveRobotRelative} to calculate
     * the required velocities and to determine whether the the robot has moved far
     * enough.
     */
    @Test
    void testMoveForward1MeterCloseLoop() {        
        // Arrange
        long targetDistance = 3;
        var move = new Transform2d(new Translation2d(targetDistance, 0), Rotation2d.kZero);
        Command cmd = new MoveRobotRelative(CONFIG, drivetrain, move);
        
        double startX = drivetrain.getState().Pose.getMeasureX().baseUnitMagnitude();
        double startY = drivetrain.getState().Pose.getMeasureY().baseUnitMagnitude();
        double startRotation = drivetrain.getState().Pose.getRotation().getDegrees();
        drivetrain.logPose();

        // Act
        CommandScheduler.getInstance().schedule(cmd);
        try {
            await().atMost(maxSimulationTime(targetDistance), TimeUnit.SECONDS).until(() -> {
                CommandScheduler.getInstance().run();
                return isFinished(cmd);
            });
        } catch (Exception e) {
            System.err.println("Command did not finish in time: " + e.getMessage());
        } finally {
            System.out.println("=== Final Pose ===");
            drivetrain.logPose();
        }

        // Assert
        double finalX = drivetrain.getState().Pose.getMeasureX().baseUnitMagnitude();
        double finalY = drivetrain.getState().Pose.getMeasureY().baseUnitMagnitude();
        double finalRotation = drivetrain.getState().Pose.getRotation().getDegrees();
        
        System.out.println("Rel Error X: " + (finalX - startX - targetDistance)/targetDistance * 100 + "%");

        assertEquals(targetDistance, Math.abs(finalX - startX), POSITION_TEST_TOLERANCE, "Didn't move far enough in X");
        assertEquals(startY, finalY, POSITION_TEST_TOLERANCE, "Should not have moved in Y");
        assertEquals(startRotation, finalRotation, ROTATIONAL_TEST_TOLERANCE_DEG, "Should not have rotated");    
    }

    @Test
    /**
     * This test demonstrates the compounding error that occurs when chaining together multiple 
     * sequential commands. The robot attempts to drive in a square pattern but accumulates
     * significant position error (~0.3m displacement) due to small inaccuracies at each step.
     * We need 5X the tolerance to pass this test, which is a sign of poor accuracy.
     * 
     * {@link DrivetrainTests#testNetDisplacementConsolidated} shows how consolidating steps 
     * (e.g. by rotating and translating simultaneously) can dramatically improve accuracy.
     */
    void testNetDisplacementPoorAccuracy() {
        // Arrange
        long stepLength = 3;
        var xFwd = new Transform2d(new Translation2d(stepLength, 0), Rotation2d.kZero);
        var rotateRight = new Transform2d(new Translation2d(0, 0), Rotation2d.kCW_90deg);
        var startingPose = drivetrain.getState().Pose;

        Command cmd = Commands.sequence(
            Commands.print("Starting"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            new MoveRobotRelative(CONFIG, drivetrain, xFwd),
            Commands.print("Moved fwd X 1"),
            Commands.runOnce(drivetrain::logPose, drivetrain),
            
            new MoveRobotRelative(CONFIG, drivetrain, rotateRight),
            Commands.print("Rotated 90 1"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            new MoveRobotRelative(CONFIG, drivetrain, xFwd),
            Commands.print("Moved fwd X 2"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            new MoveRobotRelative(CONFIG, drivetrain, rotateRight),
            Commands.print("Rotated 90 2"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            new MoveRobotRelative(CONFIG, drivetrain, xFwd),
            Commands.print("Moved fwd X 3"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            new MoveRobotRelative(CONFIG, drivetrain, rotateRight),
            Commands.print("Rotated 90 3"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            new MoveRobotRelative(CONFIG, drivetrain, xFwd),
            Commands.print("Moved fwd X 4"),
            Commands.runOnce(drivetrain::logPose, drivetrain),
            Commands.print("DONE")
        );
        cmd.addRequirements(drivetrain);

        // Act
        CommandScheduler.getInstance().schedule(cmd);
        try {
            await().atMost(maxSimulationTime(stepLength * 8), TimeUnit.SECONDS).until(() -> {
                CommandScheduler.getInstance().run();
                return isFinished(cmd);
            });
        } catch (Exception e) {
            System.err.println("Command did not finish in time: " + e.getMessage());
        } finally {
            System.out.println("=== Final Pose ===");
            drivetrain.logPose();
        }

        // Asset
        var finalPose = drivetrain.getState().Pose;
        var displacement = finalPose.minus(startingPose);
        var deltaX = displacement.getMeasureX().baseUnitMagnitude();
        var deltaY = displacement.getMeasureY().baseUnitMagnitude();
        var deltaRot = displacement.getRotation().getDegrees();

        assertEquals(0, deltaX, 5 * POSITION_TEST_TOLERANCE, "X displacement should be 0");
        assertEquals(0, deltaY, 5 * POSITION_TEST_TOLERANCE, "Y displacement should be 0");
        assertEquals(Rotation2d.kCCW_90deg.getDegrees(), deltaRot, 5 * ROTATIONAL_TEST_TOLERANCE_DEG, "Angle should be 90 degrees CCW from start");
    }

    /**
     * This test demonstrates improved accuracy when combining translation and rotation into 
     * single commands rather than chaining them sequentially. The robot moves in a compact
     * pattern (forward+rotate, right+rotate) and achieves much better position accuracy
     * (~0.1m displacement) compared to {@link DrivetrainTests#testNetDisplacementPoorAccuracy}.
     * 
     * This approach reduces error accumulation by:
     * - Fewer command transitions (4 vs 8 commands)
     * - Coordinated motion where translation and rotation happen simultaneously
     * - Better error cancellation through alternating movement directions
     */
    @Test
    void testNetDisplacementConsolidated() {
        // Arrange
        long stepLength = 3;
        var moveLeg = new Transform2d(new Translation2d(stepLength, 0), Rotation2d.kCW_90deg);
        var startingPose = drivetrain.getState().Pose;

        Command cmd = Commands.sequence(
            Commands.print("Starting"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            new MoveRobotRelative(CONFIG, drivetrain, moveLeg),
            Commands.print("Moved 1"),
            Commands.runOnce(drivetrain::logPose, drivetrain),
            
            new MoveRobotRelative(CONFIG, drivetrain, moveLeg),
            Commands.print("Moved 2"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            new MoveRobotRelative(CONFIG, drivetrain, moveLeg),
            Commands.print("Moved 3"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            new MoveRobotRelative(CONFIG, drivetrain, moveLeg),
            Commands.print("Moved 4"),
            Commands.runOnce(drivetrain::logPose, drivetrain),

            Commands.print("DONE")
        );
        cmd.addRequirements(drivetrain);

        // Act
        CommandScheduler.getInstance().schedule(cmd);
        try {
            await().atMost(maxSimulationTime(stepLength * 4), TimeUnit.SECONDS).until(() -> {
                CommandScheduler.getInstance().run();
                return isFinished(cmd);
            });
        } catch (Exception e) {
            System.err.println("Command did not finish in time: " + e.getMessage());
        } finally {
            System.out.println("=== Final Pose ===");
            drivetrain.logPose();
        }

        // Asset
        var finalPose = drivetrain.getState().Pose;
        var displacement = finalPose.minus(startingPose);
        var deltaX = displacement.getMeasureX().baseUnitMagnitude();
        var deltaY = displacement.getMeasureY().baseUnitMagnitude();
        var deltaRot = displacement.getRotation().getDegrees();

        assertEquals(0, deltaX, 2 * POSITION_TEST_TOLERANCE, "X displacement should be 0");
        assertEquals(0, deltaY, 2 * POSITION_TEST_TOLERANCE, "Y displacement should be 0");
        assertEquals(0, deltaRot, 2 * ROTATIONAL_TEST_TOLERANCE_DEG, "Rotation should be 0");
    }


    static class TestCommand extends Command {
        private final Drivetrain drivetrain;
        private final SwerveRequest req;
        int cycles = 0;

        public TestCommand(Drivetrain drivetrain, SwerveRequest req) {
            this.drivetrain = drivetrain;
            this.req = req;
            addRequirements(drivetrain);
        }

        @Override
        public void initialize() {
            System.out.println("Starting drivetrain command");
        }

        @Override
        public void execute() {
            this.cycles++;
            drivetrain.setControl(req);
        }

        @Override
        public void end(boolean interrupted) {
            System.out.println("Ending drivetrain command after " + this.cycles + " cycles");
            drivetrain.setControl(new SwerveRequest.Idle());
        }
    }
}
