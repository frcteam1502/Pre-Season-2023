package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public final class Constants {

    public static final class Motors {
        //drive
        public static final CANSparkMax DRIVE_FRONT_LEFT = new CANSparkMax(0, MotorType.kBrushless);
        public static final CANSparkMax DRIVE_FRONT_RIGHT = new CANSparkMax(1, MotorType.kBrushless);
        public static final CANSparkMax DRIVE_BACK_LEFT = new CANSparkMax(2, MotorType.kBrushless);
        public static final CANSparkMax DRIVE_BACK_RIGHT = new CANSparkMax(3, MotorType.kBrushless);

        //turn
        public static final CANSparkMax ANGLE_FRONT_LEFT = new CANSparkMax(4, MotorType.kBrushless);
        public static final CANSparkMax ANGLE_FRONT_RIGHT = new CANSparkMax(5, MotorType.kBrushless);
        public static final CANSparkMax ANGLE_BACK_LEFT = new CANSparkMax(6, MotorType.kBrushless);
        public static final CANSparkMax ANGLE_BACK_RIGHT = new CANSparkMax(7, MotorType.kBrushless);
    }

    public static final class DriveConstants {

        //Wheel Base
        // TODO: Change actual
        public static final double wheelBaseWidth = 5.0;
        public static final double wheelBaseLength = 5.0;

        public static final Translation2d FrontLPosition = new Translation2d(wheelBaseLength/2, wheelBaseWidth/2);
        public static final Translation2d FrontRPosition = new Translation2d(wheelBaseLength/2, -wheelBaseWidth/2);
        public static final Translation2d BackLPosition = new Translation2d(-wheelBaseLength/2, wheelBaseWidth/2);
        public static final Translation2d BackRPosition = new Translation2d(-wheelBaseLength/2, -wheelBaseWidth/2);

        public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(
            FrontRPosition, FrontLPosition, BackRPosition, BackLPosition);

    }
}
