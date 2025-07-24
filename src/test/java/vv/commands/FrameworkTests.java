package vv.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static vv.utils.TestSetup.resetSimulationState;

public class FrameworkTests {
    
    @BeforeEach
    @SuppressWarnings("unused")
    void beforeEach() {
        resetSimulationState();
    }

    @Test
    void commandCancellation() {
        // Arrange
        var subA = new TestSubsystem();
        var subB = new TestSubsystem();
        var subC = new TestSubsystem();

        var cmd1 = new InterruptionTrackerCmd();
        cmd1.addRequirements(subA, subB, subC);
        var cmd2 = new InterruptionTrackerCmd(InterruptionBehavior.kCancelIncoming);
        cmd2.addRequirements(subB);
        var cmd3 = new InterruptionTrackerCmd();
        cmd3.addRequirements(subB);

        // Act - cmd1 interrupted by cmd 2; cmd3 cancelled by cmd2;
        CommandScheduler.getInstance().schedule(cmd1);
        CommandScheduler.getInstance().schedule(cmd2);
        CommandScheduler.getInstance().schedule(cmd3);

        // Assert - cmd1
        assertEquals(true, cmd1.interrupted);
        assertEquals(false, cmd1.isFinished());
        assertEquals(false, cmd1.isScheduled());

        // Assert - cmd2
        assertEquals(false, cmd2.interrupted);
        assertEquals(false, cmd2.isFinished());
        assertEquals(true, cmd2.isScheduled());

        // Assert - cmd3
        assertEquals(false, cmd3.interrupted);
        assertEquals(false, cmd3.isFinished());
        assertEquals(true, cmd2.isScheduled());
    }

    @Test
    void controllerTriggersCommand() {
        // Arrange
        CommandXboxController controller = new CommandXboxController(0);
        XboxControllerSim sim = new XboxControllerSim(controller.getHID());
        Command cmd = new InterruptionTrackerCmd();        
        controller.a().onTrue(cmd);

        // Verify initial state
        assertEquals(false, controller.getHID().getAButton());
        assertEquals(false, controller.a().getAsBoolean());
        assertEquals(false, CommandScheduler.getInstance().isScheduled(cmd));
        
        // Act - Press button to trigger command
        sim.setAButton(true);
        sim.notifyNewData();
        CommandScheduler.getInstance().run(); // Then process scheduled commands

        // Assert
        assertEquals(true, controller.getHID().getAButton());
        assertEquals(true, controller.getHID().getAButtonPressed());
        assertEquals(true, controller.a().getAsBoolean());
        assertEquals(true, CommandScheduler.getInstance().isScheduled(cmd));
    }

    static class TestSubsystem implements Subsystem {}

    static class InterruptionTrackerCmd extends Command {
        public boolean interrupted = false;
        private InterruptionBehavior interruptionBehavior = InterruptionBehavior.kCancelSelf;

        public InterruptionTrackerCmd() {
            super();
        }

        public InterruptionTrackerCmd(InterruptionBehavior interruptionBehavior) {
            this();
            this.interruptionBehavior = interruptionBehavior;
        }

        @Override
        public boolean isFinished() {
            return false; // Never finish
        }

        @Override
        public void end(boolean  interrupted) {
            this.interrupted = interrupted;
        }

        @Override
        public InterruptionBehavior getInterruptionBehavior() {
            return this.interruptionBehavior;
        }
    }
}
