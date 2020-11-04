package frc.team4373.robot.commands.drivetrain;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.*;
import frc.team4373.robot.RobotMap;
import frc.team4373.robot.subsystems.Drivetrain;

public class TimedDriveAuton extends ParallelRaceGroup {
    /**
     * Constructs a time based, driving auton.
     * @param time the time the command runs for.
     * @param speed the speed at which the robot moves.
     * @param angle the angle at which the robot moves.
     */
    public TimedDriveAuton(double time, double speed, double angle) {
        addCommands(new __TimedDriveAuton(speed, angle), new WaitCommand(time));
    }

    @SuppressWarnings("checkstyle:TypeName")
    private static class __TimedDriveAuton extends PIDCommand {
        private Drivetrain drivetrain;

        private double speed;
        private double angle;

        public __TimedDriveAuton(double speed, double angle) {
            super(new PIDController(RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS.kP,
                    RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS.kI,
                    RobotMap.DRIVE_STRAIGHT_ROTATE_GAINS.kD),
                () -> 0, () -> 0, (output) -> { }, Drivetrain.getInstance());
            this.drivetrain = Drivetrain.getInstance();
            this.m_measurement = this::_returnPIDInput;
            initialize();
            this.m_useOutput = this::_usePIDOutput;

            this.speed = speed;
            this.angle = angle;
        }

        @Override
        public void initialize() {
            super.initialize();
            this.setSetpoint(_returnPIDInput());
        }

        @SuppressWarnings("checkstyle:MethodName")
        private double _returnPIDInput() {
            return drivetrain.getPigeonYawRaw();
        }

        @SuppressWarnings("checkstyle:MethodName")
        private void _usePIDOutput(double rotationOutput) {
            double x = Math.cos(Math.toRadians(this.angle)) * speed;
            double y = Math.sin(Math.toRadians(this.angle)) * speed;
            this.drivetrain.drive(rotationOutput * RobotMap.DRIVE_ASSIST_MAX_TURN_SPEED, x, y);
        }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void end(boolean interrupted) {
            drivetrain.stop();
        }

        private void setSetpoint(double setpoint) {
            this.m_setpoint = () -> setpoint;
        }
    }
}