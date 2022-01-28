package frc.robot;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.wpilibj.XboxController;
import com.kauailabs.navx.*;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
public class RobotOperator {

    private SwerveBase swerveBase;
    private static final float ROBOT_WIDTH = 31.0f; // inches
    private static final float ROBOT_LENGTH = 32.25f; // inches
    private XboxController myController;
    private CANCoder flCanCoder, frCanCoder, blCanCoder, brCanCoder;
    private AHRS navX;

    // construct SwerveBase, which will initialize all the swerve modules
    public RobotOperator(int[] falconPorts, int[] cancoderPorts, XboxController inXboxController) {
        WPI_TalonFX driveFalconFL = new WPI_TalonFX(falconPorts[0]);
        WPI_TalonFX steerFalconFL = new WPI_TalonFX(falconPorts[1]);
        WPI_TalonFX driveFalconFR = new WPI_TalonFX(falconPorts[2]);
        WPI_TalonFX steerFalconFR = new WPI_TalonFX(falconPorts[3]);
        WPI_TalonFX driveFalconBL = new WPI_TalonFX(falconPorts[4]);
        WPI_TalonFX steerFalconBL = new WPI_TalonFX(falconPorts[5]);
        WPI_TalonFX driveFalconBR = new WPI_TalonFX(falconPorts[6]);
        WPI_TalonFX steerFalconBR = new WPI_TalonFX(falconPorts[7]);
        flCanCoder = new CANCoder(cancoderPorts[0]);
        frCanCoder = new CANCoder(cancoderPorts[1]);
        blCanCoder = new CANCoder(cancoderPorts[2]);
        brCanCoder = new CANCoder(cancoderPorts[3]);
        myController = inXboxController; 
        navX = new AHRS(SPI.Port.kMXP);
        swerveBase = new SwerveBase(steerFalconFL, steerFalconFR, steerFalconBL, steerFalconBR,
                driveFalconFL, driveFalconFR, driveFalconBL, driveFalconBR,
                ROBOT_WIDTH, ROBOT_LENGTH);
    }
        
    public void Update() {
        double frAngle = frCanCoder.getAbsolutePosition();
        double flAngle = flCanCoder.getAbsolutePosition();
        double brAngle = brCanCoder.getAbsolutePosition();
        double blAngle = blCanCoder.getAbsolutePosition();
        double driveY = myController.getLeftY();
        double driveX = myController.getRightX();
        double steerX = myController.getLeftX();
        double robotAngle = navX.getAngle();
        swerveBase.SwerveDrive(driveX, driveY, steerX, robotAngle, frAngle, flAngle, blAngle, brAngle);
    }
        
    

}
