package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
    Pose2d m_pose = new Pose2d();

    // Create Gyro here (Should be a NavX)
    AHRS navx = new AHRS();
    // Create modules here
    SwerveModule[] swerveModules = new SwerveModule[4];
    
    // Create module translations
    Translation2d[] swerveModuleTranslations = new Translation2d[4];
   
    // Create kinematics object
    SwerveDriveKinematics swerveDriveKinematics;
    SwerveDriveOdometry m_Odometry;
    // Define the max speed of the modules
    double maxSpeed = 5;
    public SwerveDrive() {
        // Initalize modules here
        swerveModules[0] = new SwerveModule(4,5);//fl
        swerveModules[1] = new SwerveModule(2,3);//fr
        swerveModules[2] = new SwerveModule(8,9);//bl
        swerveModules[3] = new SwerveModule(6,7);//br

        swerveModuleTranslations[0] = new Translation2d(0.505/2,0.505/2); //You would actually need to measure xy on the robot
        swerveModuleTranslations[1] = new Translation2d(0.505,-0.505);
        swerveModuleTranslations[2] = new Translation2d(-0.505,0.505);
        swerveModuleTranslations[3] = new Translation2d(-0.505,-0.505);

        swerveDriveKinematics = new SwerveDriveKinematics(swerveModuleTranslations);
        
        Rotation2d gyroAngle = navx.getRotation2d();
        m_Odometry = new SwerveDriveOdometry(swerveDriveKinematics, gyroAngle, new SwerveModulePosition[] {
            swerveModules[0].getPos(),
            swerveModules[1].getPos(),
            swerveModules[2].getPos(),
            swerveModules[3].getPos()
        });
    }

    public void drive(ChassisSpeeds speeds) {
        // Convert speeds from field relative to robot relative
        ChassisSpeeds robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, navx.getRotation2d());

        // Convert the speeds to be discrete for tighter control
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);

        // Convert the speeds to individual states for each module
        SwerveModuleState[] moduleStates = swerveDriveKinematics.toSwerveModuleStates(discreteSpeeds);

        // Desaturate wheels speeds to prevent a speed that is impossible
        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, maxSpeed);

        // Loop through the modules and set their states
        for(int i = 0; i < 4; i++){
            swerveModules[i].setState(moduleStates[i]);
        }
    }

    public Pose2d odemetryGetPosition(){
        return m_pose;
    }
    public void periodic(Rotation2d gyroAngle){

        //update the odemetry pose
        m_pose = m_Odometry.update(gyroAngle, 
            new SwerveModulePosition[] {
            swerveModules[0].getPos(),swerveModules[1].getPos(),
            swerveModules[2].getPos(),swerveModules[3].getPos()
        });
    }

}