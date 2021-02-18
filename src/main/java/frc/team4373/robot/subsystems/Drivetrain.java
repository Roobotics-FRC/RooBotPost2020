package frc.team4373.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.wpilibj.geometry.*;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.team4373.robot.RobotMap;


/**
 * A Javadoc template. TODO: Update Drivetrain Javadoc.
 */
public class Drivetrain extends SubsystemBase {
    private static volatile Drivetrain instance;
    private final SwerveDriveOdometry odometry;
    private final PigeonIMU pigeon;
    private final double initialAngle;
    private final SwerveWheel frontLeft;
    private final SwerveWheel frontRight;
    private final SwerveWheel backRight;
    private final SwerveWheel backLeft;
    public SwerveDriveKinematics kinematics;
    private DriveMode driveMode;// = DriveMode.NORTH_UP;
    private BrakeMode brakeMode;// = BrakeMode.IMPLODE;
    private SwerveModuleState[] brakeStates = null;

    public Drivetrain() {
        Translation2d frontLeft = new Translation2d(15, 15);
        Translation2d frontRight = new Translation2d(15, 15);
        Translation2d backRight = new Translation2d(15, 15);
        Translation2d backLeft = new Translation2d(15, 15);

        kinematics = new SwerveDriveKinematics(frontLeft, frontRight, backRight, backLeft);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), new Pose2d());

        pigeon = new PigeonIMU(RobotMap.PIGEON_ID);
        initialAngle = getPigeonYawRaw();

        this.frontLeft = new SwerveWheel(RobotMap.left1Drive, RobotMap.left1Rotate);
        this.frontRight = new SwerveWheel(RobotMap.right1Drive, RobotMap.right1Rotate);
        this.backRight = new SwerveWheel(RobotMap.right2Drive, RobotMap.right2Rotate);
        this.backLeft = new SwerveWheel(RobotMap.left2Drive, RobotMap.left2Rotate);
    }

    /**
     * The getter for the Intake class.
     *
     * @return the singleton Intake object.
     */
    public static Drivetrain getInstance() {
        if (instance == null) {
            synchronized (Drivetrain.class) {
                if (instance == null) {
                    instance = new Drivetrain();
                }
            }
        }
        return instance;
    }

    public DriveMode getDriveMode() {
        return driveMode;
    }

    public void setDriveMode(DriveMode driveMode) {
        this.driveMode = driveMode;
    }

    public BrakeMode getBrakeMode() {
        return brakeMode;
    }

    public void setBrakeMode(BrakeMode brakeMode) {
        this.brakeMode = brakeMode;
        switch (brakeMode) {
            case IMPLODE:
                // brakeStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0)).map { it.speedMetersPerSecond = 0.0; return@map it }.reversed().toTypedArray()
                SwerveModuleState[] implodeArr = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));
                for (int i = 0; i < implodeArr.length; i++) {
                    implodeArr[i].speedMetersPerSecond = 0;
                }
                brakeStates = new SwerveModuleState[implodeArr.length];
                for (int i = 0; i < implodeArr.length; i++) {
                    brakeStates[i] = implodeArr[(implodeArr.length - 1) - i];
                }
                break;
            case OCTAGON:
                // brakeStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0)).map { it.speedMetersPerSecond = 0.0; return@map it }.toTypedArray()
                SwerveModuleState[] octagonArr = kinematics.toSwerveModuleStates(new ChassisSpeeds(0.0, 0.0, 1.0));
                for (int i = 0; i < octagonArr.length; i++) {
                    octagonArr[i].speedMetersPerSecond = 0;
                }
                brakeStates = octagonArr;
                break;
            case NONE:
                brakeStates = null;
                break;
        }
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(getPigeonYawRaw()), frontLeft.getState(), frontRight.getState(), backRight.getState(), backLeft.getState());
    }

    /**
     * Returns the raw Pigeon yaw value.
     *
     * @return Pigeon yaw value.
     */
    public double getPigeonYawRaw() {
        double[] ypr = new double[3];
        pigeon.getYawPitchRoll(ypr);
        return ypr[0] * -1;
    }

    public void drive(double rotation, double x, double y) {
        SwerveModuleState[] swerveModuleStates = null;
        switch (driveMode) {
            case NORTH_UP:
                swerveModuleStates = kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotation, Rotation2d.fromDegrees(getPigeonYawRaw())/*fromPigeonUnits(getPigeonYawRaw())*/));
                break;
            case OWN_SHIP_UP:
                swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, rotation));
                break;
        }
        if (swerveModuleStates != null) {
            frontLeft.setState(swerveModuleStates[0]);
            frontRight.setState(swerveModuleStates[1]);
            backRight.setState(swerveModuleStates[2]);
            backLeft.setState(swerveModuleStates[3]);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates,
                RobotMap.MAX_SPEED_METERS_PER_SECOND);
        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }

//    /**
//     * Resets the pigeon's yaw to consider the current orientation field-forward (zero degrees).
//     */
//    fun resetPigeonYaw() {
//        initialAngle = getPigeonYawRaw()
//    }
//
//    /**
//     * Sets the pigeon's yaw to be a given value at the current position.
//     * @param angle the value, in degrees, that the pigeon should have at the current position..
//     */
//    fun setPigeonYaw(angle: Double) {
//        initialAngle = getPigeonYawRaw() + angle
//    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void brake() {
        if (brakeStates != null) {
            setModuleStates(brakeStates);
        }
    }

    /**
     * The drive mode for the swerve bot.
     * <p>
     * <p>
     * The modes are as follows:
     * <p>
     * * North-up mode maintains a constant sense of north using the gyro (i.e., pushing the
     * joystick forward moves in the same direction regardless of the robot's orientation)
     * * Own-ship-up mode always drives relative to the front of the robot
     * (i.e., pushing the joystick forward moves the robot in the direction it is
     * facing)
     */
    public enum DriveMode {
        NORTH_UP, OWN_SHIP_UP
    }

    /**
     * The swerve brake mode for the swerve bot (i.e., what to do when the input is zero).
     * <p>
     * <p>
     * The modes are as follows:
     * <p>
     * * Implode mode points all wheels inward (toward the center of the bot)
     * * Octagon mode points all wheels orthogonally to the diagonal that runs from the wheel
     * to the center (the standard turning configuration)
     * * When swerve brake mode is disabled, the wheels remain in the direction they were
     * facing before stopping
     */
    public enum BrakeMode {
        IMPLODE, OCTAGON, NONE
    }
}