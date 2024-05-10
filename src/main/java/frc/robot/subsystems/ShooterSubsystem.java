package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {

    private double UpperRollerRPM = 0.0;
    private double LowerRollerRPM = 0.0;

    private final CANSparkMax UpperRollerMotor;
    private final CANSparkMax LowerRollerMotor;

    private final RelativeEncoder UpperRollerEnc;
    private final RelativeEncoder LowerRollerEnc;

    public ShooterSubsystem(int UpperRollerID, int LowerRollerID) {
        UpperRollerMotor = new CANSparkMax(UpperRollerID, MotorType.kBrushless);
        LowerRollerMotor = new CANSparkMax(LowerRollerID, MotorType.kBrushless);

        UpperRollerEnc = UpperRollerMotor.getEncoder();
        LowerRollerEnc = LowerRollerMotor.getEncoder();

    }

    public Command setPWM(double speedUpper, double speedLower) {
        return runOnce(
                () -> {
                    UpperRollerMotor.set(speedUpper);
                    LowerRollerMotor.set(speedLower);

                });
    }

    public double getUpperRoller() {
        return (UpperRollerRPM);
    }

    public double getLowerRoller() {
        return (LowerRollerRPM);
    }

    @Override
    public void periodic() {
        UpperRollerRPM = UpperRollerEnc.getVelocity();
        LowerRollerRPM = LowerRollerEnc.getVelocity();
    }

}
