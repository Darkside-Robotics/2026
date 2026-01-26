package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.wpilibj.I2C;
//import edu.wpi.first.wpilibj.I2C.Port;

public final class Constants {

    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1 / 6.75;
        public static final double kTurningMotorGearRatio = 1 / 12.8;
        public static final double kDistancePerWheelRotation = Math.PI * kWheelDiameterMeters;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * kDistancePerWheelRotation;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
        public static final double kPTurning = 0.5;
        public static final double kDriveMotorFeedforwardVoltsPerRotationsPerMinute = 1/473;        
        public static final double kDriveMotorFeedforwardVoltsPerRotationsPerSecond = 1/(473 / 60);        
        public static final double kDriveMotorFeedforwardVoltsPerMetersPerSecond = 1/(473 / 60 * kDistancePerWheelRotation * kDriveMotorGearRatio);
    }

    public static final class LEDConstants 
    {       
        public static final int PWMPort = 0;
        public static final int Length = 0;
    }

    public static final class ClimbingConstants {
        public static final int LaserCANPort = 21;
        public static final int LimitSwitchPort = 0; 

    public static final class Motor {
        public static final int MotorPort = 12;
        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;
        public static final int Power = 10;
    }
    }


    public static final class DriveConstants {

        public static final int CurrentFreeLimit = 60;
        public static final int CurrentStalledLimit = 40;


        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(21.55);
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(23.58);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final class Motors {
            public static final class Front {
                public static final class Left {
                    public static final class Drive {
                        public static final int Port = 8;
                        public static final boolean Reversed = false;
                    }
                    public static final class Steer {
                        public static final int Port = 4;
                        public static final boolean Reversed = true;
                    }
                    public static final class AbsoluteEncoder {
                        //Angle, not value.
                        public static final double Offset = 4.362; // NEED
                        public static final boolean Reversed = true;
                        public static final int Port = 0;
                    }
                }

                public static final class Right {
                    public static final class Drive {
                        public static final int Port = 3;
                        public static final boolean Reversed = false;
                    }
                    public static final class Steer {
                        public static final int Port = 7;
                        public static final boolean Reversed = true;
                    }
                    public static final class AbsoluteEncoder {
                        //Angle, not value.
                        public static final double Offset = 4.166; // NEED
                        public static final boolean Reversed = true;
                        public static final int Port = 2;
                    }
                }
            }

            public static final class Back {
                public static final class Left {
                    public static final class Drive {
                        public static final int Port = 5;
                        public static final boolean Reversed = false;
                    }
                    public static final class Steer {

                        public static final int Port = 1;
                        public static final boolean Reversed = true;
                    }
                    public static final class AbsoluteEncoder { 
                        //Angle, not value.
                        public static final double Offset = 1.838; // NEED
                        public static final boolean Reversed = true;
                        public static final int Port = 1;
                    }
                }

                public static final class Right {
                    public static final class Drive {
                        public static final int Port = 6;
                        public static final boolean Reversed = false;
                    }
                    public static final class Steer {

                        public static final int Port = 2;
                        public static final boolean Reversed = true;
                    }
                    public static final class AbsoluteEncoder {
                        //Angle, not value.
                        public static final double Offset = 6.158; // NEED
                        public static final boolean Reversed = true;
                        public static final int Port = 3;
                    }
                }
            }
        }

        public static final double kPhysicalMaxSpeedMetersPerSecond = 2.3;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 1;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 2;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 1.5;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 2;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.4;
        public static final double kPYController = 1.4;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class JoystickConstants {
        public static final int kDriverControllerPort = 0;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final double kDeadband = 0.1;
    }

   
}
