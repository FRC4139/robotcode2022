package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;

public class SwerveBase {

    private double width, length; // in inches
    private SwerveModuleController fr, fl, bl, br;
    // Constructor for the SwerveBase class which initializes the motors
    public SwerveBase(WPI_TalonFX sfr, WPI_TalonFX sfl, WPI_TalonFX sbl, WPI_TalonFX sbr, WPI_TalonFX dfr, WPI_TalonFX dfl, WPI_TalonFX dbl, WPI_TalonFX dbr, double w, double l) {
        fr = new SwerveModuleController(sfr, dfr);
        fl = new SwerveModuleController(sfl, dfl);
        bl = new SwerveModuleController(sbl, dbl);
        br = new SwerveModuleController(sbr, dbr);
        width = w;
        length = l;
    }

    public void SwerveDrive(double driveX, double driveY, double steerX, double robotAngle, double frAngle, double flAngle, double blAngle, double brAngle) {
        
        double desiredVectorXfr = driveX + steerX*(-length/2 * Math.cos(robotAngle)-width/2 * Math.sin(robotAngle));
        double desiredVectorYfr = driveY + steerX*(-length/2 * Math.sin(robotAngle)+width/2 * Math.cos(robotAngle));
        double desiredVectorXfl = driveX + steerX*(-length/2 * Math.cos(robotAngle)+width/2 * Math.sin(robotAngle));
        double desiredVectorYfl = driveY + steerX*(-length/2 * Math.sin(robotAngle)-width/2 * Math.cos(robotAngle));
        double desiredVectorXbl = driveX + steerX*(length/2 * Math.cos(robotAngle)+width/2 * Math.sin(robotAngle));
        double desiredVectorYbl = driveY + steerX*(length/2 * Math.sin(robotAngle)-width/2 * Math.cos(robotAngle));
        double desiredVectorXbr = driveX + steerX*(length/2 * Math.cos(robotAngle)-width/2 * Math.sin(robotAngle));
        double desiredVectorYbr = driveY + steerX*(length/2 * Math.sin(robotAngle)+width/2 * Math.cos(robotAngle));


        double orientationFr = Math.atan2(desiredVectorYfr, desiredVectorXfr)-90; //subtracted 90 since angle 0 means forward
        double magnitudeFr = Math.sqrt(Math.pow(desiredVectorXfr, 2) + Math.pow(desiredVectorYfr, 2));
        double orientationFl = Math.atan2(desiredVectorYfl, desiredVectorXfl)-90;
        double magnitudeFl = Math.sqrt(Math.pow(desiredVectorXfl, 2) + Math.pow(desiredVectorYfl, 2));
        double orientationBl = Math.atan2(desiredVectorYbl, desiredVectorXbl)-90;
        double magnitudeBl = Math.sqrt(Math.pow(desiredVectorXbl, 2) + Math.pow(desiredVectorYbl, 2));
        double orientationBr = Math.atan2(desiredVectorYbr, desiredVectorXbr)-90;
        double magnitudeBr = Math.sqrt(Math.pow(desiredVectorXbr, 2) + Math.pow(desiredVectorYbr, 2));

        fr.SetTargetAngleAndSpeed(orientationFr, magnitudeFr, frAngle);
        fl.SetTargetAngleAndSpeed(orientationFl, magnitudeFl, flAngle);
        bl.SetTargetAngleAndSpeed(orientationBl, magnitudeBl, blAngle);
        br.SetTargetAngleAndSpeed(orientationBr, magnitudeBr, brAngle);




    }
}
