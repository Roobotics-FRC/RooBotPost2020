package frc.team4373.robot.commands.camera;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.input.OI;
import frc.team4373.robot.subsystems.Camera;
import frc.team4373.robot.subsystems.Drivetrain;

import java.util.function.Function;

public class VisionQuerierCommand extends CommandBase {
    private enum State {
        POLLING, SETTING, WAITING
    }

    private State state;

    private Function<Double, Command> constructor;
    private String visionField;
    private double tolerance;

    private Command command;
    private NetworkTable visionTable;

    private double accumulator = 0;
    private int pollingIterationCount = 0; // # of polls in POLLING state
    private boolean finished = false;

    private int rotationExecutionCount = 0; // total # of rotation adjustments

    private double waitStart;

    /**
     * Constructs a command responsible for managing repeated querying of a vision
     * Network Table field and corresponding PID commands to attain a state
     * within a given tolerance.
     *
     * <p>Note: the vision field, tolerance, and constructor input must all be in the same
     * units.</p>
     *
     * @param visionField the field on the vision NetworkTable to query for values.
     * @param tolerance the absolute value of the input at which the setpoint will be considered
     *                  to be attained and the command will exit. This value <i>must</i> be
     *                  positive.
     * @param constructor the constructor of the PIDCommand that takes an input from the given
     *                    vision table field.
     */
    public VisionQuerierCommand(String visionField, double tolerance,
                                Function<Double, Command> constructor) {
        addRequirements(Camera.getInstance(), Drivetrain.getInstance());

        this.visionField = visionField;
        this.tolerance = tolerance;
        this.constructor = constructor;

        this.visionTable = NetworkTableInstance.getDefault().getTable(RobotMap.VISION_TABLE_NAME);
        this.state = State.POLLING;
    }

    @Override
    public void initialize() {
        if (this.visionTable.getEntry(this.visionField).getType() != NetworkTableType.kDouble) {
            DriverStation.reportError(
                    "Non-double vision field (" + this.visionField
                            + ") passed to VisionQuerierCommand; exiting...", false);
            this.finished = true;
        } else {
            resetStateVars();
            this.finished = false;
        }
        OI.getInstance().getOperatorJoystick().setRumble(GenericHID.RumbleType.kRightRumble,
                RobotMap.OPER_ROTATE_VIB_INTENSITY);
        OI.getInstance().getOperatorJoystick().setRumble(GenericHID.RumbleType.kLeftRumble,
                RobotMap.OPER_ROTATE_VIB_INTENSITY);
    }

    @Override
    public void execute() {
        switch (state) {
            case POLLING:
                this.waitStart = -1;
                SmartDashboard.putString("v/state", "polling");
                if (this.pollingIterationCount >= RobotMap.VISION_SAMPLE_COUNT) {
                    this.state = State.SETTING;
                    break;
                }
                double sample = visionTable.getEntry(visionField).getDouble(0);
                SmartDashboard.putNumber("v/sample", sample);
                this.accumulator += sample;
                ++this.pollingIterationCount;
                break;
            case SETTING:
                SmartDashboard.putString("v/state", "setting");

                // Compute setpoint by taking polling average
                // The Network Tables field gives us the amount by which we're off,
                // so we want to move opposite that direction to reach 0
                double setpoint = -(this.accumulator / this.pollingIterationCount);
                SmartDashboard.putNumber("v/setpt", setpoint);

                // Reset all variables for next iteration before we spin up the new command
                resetStateVars();

                // Check if offset setpoint would be less than acceptable tolerance
                // (i.e., we're close enough)
                if (Math.abs(setpoint) < this.tolerance) {
                    System.out.println("Within tolerance");
                    this.finished = true;
                    break;
                }

                // Start the command—will steal control from us b/c we require the drivetrain
                this.command = this.constructor.apply(setpoint);
                this.command.schedule();
                this.state = State.WAITING;
                ++this.rotationExecutionCount;
                break;
            case WAITING:
                SmartDashboard.putString("v/state", "waiting");
                if (command.isScheduled()) {
                    SmartDashboard.putString("v/auton_cmd_state", "running");
                } else {
                    SmartDashboard.putString("v/auton_cmd_state", "done");
                    if (this.waitStart == -1) {
                        this.waitStart = Timer.getFPGATimestamp();
                    } else if (Timer.getFPGATimestamp()
                            > this.waitStart + RobotMap.INTER_QUERY_DELAY_SEC) {
                        this.state = State.POLLING;
                    }
                }
                break;
            default:
                // We're in an undefined state—get out!
                SmartDashboard.putString("v/state", "undefined");
                this.finished = true;
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return this.finished
                || this.rotationExecutionCount >= RobotMap.MAX_ALLOWABLE_VISION_ITERATIONS;
    }

    /**
     * Resets all state/polling variables to their initial, start-of-polling state.
     */
    private void resetStateVars() {
        this.state = State.POLLING;
        this.accumulator = 0;
        this.pollingIterationCount = 0;
        this.rotationExecutionCount = 0;
    }

    @Override
    public void end(boolean interrupted) {
        OI.getInstance().getOperatorJoystick().setRumble(GenericHID.RumbleType.kRightRumble, 0);
        OI.getInstance().getOperatorJoystick().setRumble(GenericHID.RumbleType.kLeftRumble, 0);
    }
}
