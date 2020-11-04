package frc.team4373.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.commands.util.RooDualPIDCommand;
import frc.team4373.robot.subsystems.Drivetrain;

/**
 * Drives a distance in the specified direction. Field/bot-oriented driving is determined by the
 * drivetrain's current drive mode (own-ship- or field-up).
 */
public class DriveDistanceAuton extends RooDualPIDCommand {
    private static final RobotMap.PID DRIVE_GAINS = new RobotMap.PID(0, 0.01, 0, 0);
    private static final double PID_OUTPUT_THRESHOLD = 0.2;

    private final Drivetrain drivetrain;

    private final double distance;
    private final double speed;
    private final double angle;

    private Translation2d originalPosition;
    private double originalHeading;

    private boolean finished = false;

    /**
     * Constructs a new distance-driving auton.
     * @param distance the distance to drive in inches.
     * @param angle the angle at which to drive.
     */
    public DriveDistanceAuton(double distance, double speed, double angle) {
        super(DRIVE_GAINS, RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS, Drivetrain.getInstance());
        this.drivetrain = Drivetrain.getInstance();

        this.distance = distance;
        this.speed = speed;
        this.angle = angle;
    }

    @Override
    public void initialize() {
        super.initialize();
        originalPosition = drivetrain.getPose().getTranslation();
        originalHeading = returnPIDInput2();
        this.finished = false;
    }

    @Override
    protected double returnPIDInput1() {
        return drivetrain.getPose().getTranslation().getDistance(originalPosition);
    }

    @Override
    protected double returnPIDInput2() {
        return drivetrain.getPose().getRotation().getDegrees();
    }

    @Override
    protected void usePIDOutput(double distancePIDOutput, double rotationPIDOutput) {
        SmartDashboard.putNumber("swerve/auton_output", distancePIDOutput);
        if (Math.abs(distancePIDOutput) <= PID_OUTPUT_THRESHOLD) {
            this.finished = true;
            return;
        }
        double x = Math.cos(Math.toRadians(this.angle)) * speed;
        double y = Math.sin(Math.toRadians(this.angle)) * speed;
        this.drivetrain.drive(rotationPIDOutput, x, y);
    }

    @Override
    protected double getSetpoint1() {
        return Units.inchesToMeters(distance);
    }

    @Override
    protected double getSetpoint2() {
        return originalHeading;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        this.drivetrain.stop();
    }
}