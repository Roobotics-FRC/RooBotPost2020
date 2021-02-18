package frc.team4373.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.team4373.robot.ConversionUtils;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.RobotMap.MotorConfig;
import frc.team4373.robot.RobotMap.PID;


/**
 * A Javadoc template. TODO: Update SwerveWheel Javadoc.
 */
public class SwerveWheel {
    public static final int PID_IDX = 0;
    public static final int FULL_REVOLUTION_TICKS = 4096;

    private final WPI_TalonSRX driveMotor;
    private final WPI_TalonSRX rotatorMotor;

    public SwerveWheel(MotorConfig drive, MotorConfig rotator) {
        this.driveMotor = new WPI_TalonSRX(drive.id);
        this.rotatorMotor = new WPI_TalonSRX(rotator.id);

        this.driveMotor.setInverted(drive.inverted);
        this.rotatorMotor.setInverted(rotator.inverted);

        this.driveMotor.setNeutralMode(drive.neutralMode);
        this.rotatorMotor.setNeutralMode(rotator.neutralMode);

        this.driveMotor.setSensorPhase(drive.encoderPhase);
        this.rotatorMotor.setSensorPhase(rotator.encoderPhase);

        this.driveMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        this.rotatorMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        this.rotatorMotor.configFeedbackNotContinuous(false, RobotMap.TALON_TIMEOUT_MS);

        this.driveMotor.config_kF(PID_IDX, drive.gains.kF);
        this.driveMotor.config_kP(PID_IDX, drive.gains.kP);
        this.driveMotor.config_kI(PID_IDX, drive.gains.kI);
        this.driveMotor.config_kD(PID_IDX, drive.gains.kD);

        this.rotatorMotor.config_kF(PID_IDX, rotator.gains.kF);
        this.rotatorMotor.config_kP(PID_IDX, rotator.gains.kP);
        this.rotatorMotor.config_kI(PID_IDX, rotator.gains.kI);
        this.rotatorMotor.config_kD(PID_IDX, rotator.gains.kD);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                ConversionUtils.encoderUnitsPerDecisecondToMetersPerSecond(
                        driveMotor.getSelectedSensorVelocity()),
                ConversionUtils.fromEncoderUnits(rotatorMotor.getSelectedSensorPosition()));
    }

    public void setState(SwerveModuleState state) {
        driveMotor.set(ControlMode.Velocity,
                ConversionUtils.metersPerSecondToEncoderUnitsPerDecisecond(
                        state.speedMetersPerSecond));
        rotatorMotor.set(ControlMode.Position, ConversionUtils.toEncoderUnits(state.angle));

        /*
            heading *= SwerveConstants.DEGREES_TO_ENCODER_UNITS

            val currentRotation = rotatorMotor.selectedSensorPosition.toDouble()
            var rotationError = Math.IEEEremainder(heading - currentRotation,
                    SwerveConstants.WHEEL_ENCODER_TICKS)

            SmartDashboard.putNumber("swerve/" + this.wheelID.name().toString() + "/Pre-Inv Rot", rotationError)

            // minimize azimuth rotation, reversing drive if necessary

            // minimize azimuth rotation, reversing drive if necessary
            isInverted = Math.abs(rotationError) > 0.25 * SwerveConstants.WHEEL_ENCODER_TICKS
            if (isInverted) {
                rotationError -= Math.copySign(0.5 * SwerveConstants.WHEEL_ENCODER_TICKS,
                        rotationError)
                speed = -speed
            }

            SmartDashboard.putNumber("swerve/" + this.wheelID.name().toString() + "/Rot Offset", rotationError)
            SmartDashboard.putNumber("swerve/" + this.wheelID.name().toString() + "/Rot Setpt",
                    currentRotation + rotationError)
            SmartDashboard.putNumber("swerve/" + this.wheelID.name().toString() + "/Speed",
                    speed * this.maxWheelSpeed)

            rotatorMotor[ControlMode.Position] = currentRotation + rotationError
            if (speed === 0) {
                driveMotor.stopMotor()
            } else {
                driveMotor[ControlMode.Velocity] = speed * this.maxWheelSpeed
            }
            */
    }

    /**
     * Sets drive and rotator PID gains.
     *
     * @param drivePID   a [SwerveConfig.PID] object with new parameters for the drive PID,
     *                   or null to leave unchanged.
     * @param rotatorPID a [SwerveConfig.PID] object with parameters for rotational PID,
     *                   or null to leave unchanged.
     */
    public void setPID(PID drivePID, PID rotatorPID) {
        if (drivePID != null) {
            setDrivePID(drivePID);
        }
        if (rotatorPID != null) {
            setRotatorPID(rotatorPID);
        }
    }

    /**
     * Sets the PID gains for the rotator motor.
     *
     * @param pid the new PID gains.
     */
    private void setRotatorPID(PID pid) {
        rotatorMotor.config_kP(PID_IDX, pid.kP);
        rotatorMotor.config_kI(PID_IDX, pid.kI);
        rotatorMotor.config_kD(PID_IDX, pid.kD);
        rotatorMotor.config_kF(PID_IDX, pid.kF);
    }

    /**
     * Sets the PID gains for the drive motor.
     *
     * @param pid the new PID gains.
     */
    private void setDrivePID(PID pid) {
        driveMotor.config_kP(PID_IDX, pid.kP);
        driveMotor.config_kI(PID_IDX, pid.kI);
        driveMotor.config_kD(PID_IDX, pid.kD);
        driveMotor.config_kF(PID_IDX, pid.kF);
    }

    /**
     * Sets the wheel's motors in percent output (non-closed-loop, non PID) mode.
     *
     * @param drivePower   the percent of maximum output to set the drive motor to (not limited).
     * @param rotatorPower the percent of maximum output to set the rotator motor to (not limited).
     */
    public void setPercentOutput(double drivePower, double rotatorPower) {
        driveMotor.set(ControlMode.PercentOutput, drivePower);
        rotatorMotor.set(ControlMode.PercentOutput, rotatorPower);
    }

    /**
     * Stops all motors.
     */
    public void stop() {
        driveMotor.stopMotor();
        rotatorMotor.stopMotor();
    }

    /**
     * Gets the percent output of the drive motor.
     *
     * @return the percent output, [-1, 1].
     */
    public double getDrivePercentOutput() {
        return driveMotor.getMotorOutputPercent();
    }

    /**
     * Reduces (for the duration of the power cycle) the rotator motor's encoder's position
     * modulo the number of ticks in a full revolution.
     */
    public void modularizeAbsoluteEncoder() {
        rotatorMotor.setSelectedSensorPosition(rotatorMotor.getSelectedSensorPosition()
                % FULL_REVOLUTION_TICKS);
    }

    /**
     * Resets the position of the absolute encoder on the rotator motor.
     * <p>
     * <p>
     * NOTE: this change does not permanently persist and will be lost upon power cycling.
     */
    public void resetAbsoluteEncoder() {
        rotatorMotor.setSelectedSensorPosition(0);
    }

    /**
     * Gets the position of the drive motor in encoder units.
     *
     * @return the position of the drive motor in encoder units.
     */
    public double getDriveMotorPosition() {
        return driveMotor.getSelectedSensorPosition();
    }

    /**
     * Gets the velocity of the drive motor in encoder units.
     *
     * @return the velocity of the drive motor in encoder units.
     */
    public double getDriveMotorVelocity() {
        return driveMotor.getSelectedSensorVelocity();
    }

    /**
     * Gets the position of the rotator motor in encoder units.
     * Since the rotator is a mag encoder, this is the absolute position.
     *
     * @return the absolute position of the rotator motor in encoder units.
     */
    public double getRotatorMotorPosition() {
        return rotatorMotor.getSelectedSensorPosition();
    }

    /**
     * Gets the velocity of the rotator motor in encoder units.
     *
     * @return the velocity of the rotator motor in encoder units.
     */
    public double getRotatorMotorVelocity() {
        return rotatorMotor.getSelectedSensorVelocity();
    }
}