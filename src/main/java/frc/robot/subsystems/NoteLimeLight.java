package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteLimeLight extends SubsystemBase {
    NetworkTable table;
    private boolean validTargets;
    private double horizontalOffset;
    private double verticalOffset;
    private double targetArea;

    public NoteLimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight-driving");
        validTargets = isTargetValid();
        horizontalOffset = getHorizontalOffset();
        verticalOffset = getVerticalOffset();
        targetArea = getTargetArea();
    }

    @Override
    public void periodic() {
        validTargets = isTargetValid();
        horizontalOffset = getHorizontalOffset();
        verticalOffset = getVerticalOffset();
        targetArea = getTargetArea();
        SmartDashboard.putBoolean("Note Target Locked", validTargets);
        SmartDashboard.putNumber("Note tx", horizontalOffset);
        SmartDashboard.putNumber("Note tv", verticalOffset);
        SmartDashboard.putNumber("Note ta", targetArea);
    }

    public boolean isTargetValid() {
        var tclass = table.getEntry("tclass").getString("nothing");
        // System.out.println("tclass: "+tclass);
        // System.out.println("Is frisbee: "+(tclass.equals("frisbee")));

        return (table.getEntry("tv").getDouble(0) == 1)&&(tclass.equals("frisbee"));
        // return (table.getEntry("tv").getDouble(0) == 1)&&((tclass == "toilet"||tclass == "frisbee"));
    }

    public double getHorizontalOffset() {
        return table.getEntry("tx").getDouble(0);
    }

    public double getVerticalOffset() {
        return table.getEntry("ty").getDouble(0);
    }

    public double getTargetArea() {
        return table.getEntry("ta").getDouble(0);
    }
    
}
