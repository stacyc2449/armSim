// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeedforwardConstants;
import frc.robot.Constants.PIDConstants;

public class ArmSim extends SubsystemBase {
  /** Creates a new ArmSim. */
  private final SingleJointedArmSim armSim = new SingleJointedArmSim
    (DCMotor.getNEO(1), 
    0.1, 
    SingleJointedArmSim.estimateMOI(0.5, 2), 
    0.5, 
    Units.degreesToRadians(-70), 
    Units.degreesToRadians(270), 
    2, 
    false);
  private final ArmFeedforward armFeedforward = new ArmFeedforward(FeedforwardConstants.kS, FeedforwardConstants.kG, FeedforwardConstants.kV);
  private final PIDController armPID = new PIDController(PIDConstants.kP, PIDConstants.kI, PIDConstants.kD);

  private final Mechanism2d m_mech = new Mechanism2d(60, 60);
  private final MechanismRoot2d m_point = m_mech.getRoot("point", 30, 30);
  private final MechanismLigament2d m_base = m_point.append(new MechanismLigament2d("base", 20, -90));
  private MechanismLigament2d m_stick = m_point.append(new MechanismLigament2d("stick", 10, armSim.getAngleRads()));
  public ArmSim() {
    SmartDashboard.putData("mech", m_mech);
  }

  public void armUp(){
    armSim.setInputVoltage(0.5);
    armSim.update(0.02);
    m_stick.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
  }

  public void armDown(){
    armSim.setInputVoltage(-0.5);
    armSim.update(0.02);
    m_stick.setAngle(Units.radiansToDegrees(armSim.getAngleRads()));
    System.out.println(armSim.getAngleRads());
  }

  public void setAngle(double angle, double speed){
    double feedforwardVoltage = armFeedforward.calculate(angle, speed);
    double pidVoltage = armPID.calculate(Units.radiansToDegrees(armSim.getAngleRads()), angle);

    armSim.setInputVoltage(feedforwardVoltage + pidVoltage);
    armSim.update(0.02);
  }

  public double getAngle(){
    return armSim.getAngleRads();
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
