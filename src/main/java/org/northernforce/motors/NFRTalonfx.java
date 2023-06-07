package org.northernforce.motors;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.northernforce.encoders.NFRAbsoluteEncoder;
import org.northernforce.encoders.NFREncoder;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public class NFRTalonfx extends TalonFX implements NFRMotorController {
    private ArrayList<TalonFX> followers;
    private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0, false, 0, 0, false);
    private final PositionVoltage m_positionVelocity = new PositionVoltage(0, false, 0, 0, false);
    public NFRTalonfx(int primaryID, int ...followerIDs){
        super(primaryID); 
        followers = new ArrayList<>();
        for (int id : followerIDs){
            followers.add(new TalonFX(id));
        }
    }
    @Override
    public boolean isPresent() {
        // TODO Auto-generated method stub
        return getVersion().getValue() != -1;
    }

    @Override
    public int getNumberOfMotors() {
        // TODO Auto-generated method stub
        return followers.size() + 1;
    }

    @Override
    public List<MotorController> getMotorControllers() {
        // TODO Auto-generated method stub
        ArrayList<MotorController> controllers = new ArrayList<>();
        controllers.add(this);
        controllers.addAll(followers);
        return controllers;
    }

    @Override
    public void setSelectedEncoder(NFREncoder encoder) throws MotorEncoderMismatchException {
        // TODO Auto-generated method stub
        
    }

    @Override
    public NFREncoder getSelectedEncoder() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public NFREncoder getIntegratedEncoder() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Optional<NFREncoder> getExternalQuadratureEncoder() {
        // TODO Auto-generated method stub
        return Optional.empty();
    }

    @Override
    public Optional<NFRAbsoluteEncoder> getAbsoluteEncoder() {
        // TODO Auto-generated method stub
        return Optional.empty();
    }

    @Override
    public void setVelocity(int pidSlot, double velocity) {
        // TODO Auto-generated method stub
        setControl(m_voltageVelocity.withVelocity(velocity).withSlot(pidSlot));
    
    }

    @Override
    public void setVelocity(int pidSlot, double velocity, double arbitraryFeedforward) {
        // TODO Auto-generated method stub
       setControl(m_voltageVelocity.withVelocity(velocity).withSlot(pidSlot).withFeedForward(arbitraryFeedforward));
    }

    @Override
    public void setPosition(int pidSlot, double position) {
        // TODO Auto-generated method stub
        setControl(m_voltageVelocity.withVelocity(position).withSlot(pidSlot));
    }

    @Override
    public void setPosition(int pidSlot, double position, double arbitraryFeedforward) {
        // TODO Auto-generated method stub
        setControl(m_voltageVelocity.withVelocity(position).withSlot(pidSlot).withFeedForward(arbitraryFeedforward));
    }

    @Override
    public void setPositionTrapezoidal(int pidSlot, double position) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public void setPositionTrapezoidal(int pidSlot, double position, double arbitraryFeedforward) {
        // TODO Auto-generated method stub
        
    }

    @Override
    public double getSimulationOutputVoltage() {
        // TODO Auto-generated method stub
        return 0;
    }

    @Override
    public void setFollowerOppose(int idx) {
        // TODO Auto-generated method stub
        
    }
    
}
