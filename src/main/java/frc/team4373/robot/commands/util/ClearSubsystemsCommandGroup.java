package frc.team4373.robot.commands.util;

import edu.wpi.first.wpilibj2.command.*;
import frc.team4373.robot.subsystems.*;

import java.util.Arrays;

/**
 * A command group that returns all subsystems to their default commands.
 */
public class ClearSubsystemsCommandGroup extends SequentialCommandGroup {
    /**
     * Constructs a ClearSubsystemsCommandGroup for the given subsystems.
     * @param subsystems the subsystems to clear.
     */
    public ClearSubsystemsCommandGroup(Subsystem... subsystems) {
        Arrays.stream(subsystems).map(ClearSubsystemCommand::new).forEach(this::addCommands);
        // addParallel(new ClearSubsystemCommand(Camera.getInstance()));
        // addParallel(new ClearSubsystemCommand(Climber.getInstance()));
        // addParallel(new ClearSubsystemCommand(Drivetrain.getInstance()));
        // addParallel(new ClearSubsystemCommand(Intake.getInstance()));
        // addParallel(new ClearSubsystemCommand(Shooter.getInstance()));
        //// addParallel(new ClearSubsystemCommand(WheelSpinner.getInstance()));
    }

    /**
     * Constructs a ClearSubsystemsCommandGroup.
     */
    public ClearSubsystemsCommandGroup() {
        this(
                Camera.getInstance(),
                Climber.getInstance(),
                Drivetrain.getInstance(),
                Intake.getInstance(),
                Shooter.getInstance()
                // WheelSpinner.getInstance()
        );
    }
}