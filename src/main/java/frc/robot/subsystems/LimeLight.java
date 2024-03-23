package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;

public class LimeLight extends SubsystemBase {
    /* Creates new LimeLight */
    NetworkTable table;

    private boolean validTargets;
    // private double horizontalOffset;
    // private double verticalOffset;
    // private double targetArea;
    private Pose3d botPose3d;
    private boolean visonLockAck = false;
    // private DriverStation.Alliance color;
    // private boolean isRedAlliance = false;

    public LimeLight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        botPose3d = new Pose3d();
        // color = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        // if (color == DriverStation.Alliance.Blue) {
        // isRedAlliance = false;
        // } else {
        // isRedAlliance = true;
        // }

    }

    @Override
    public void periodic() {
        validTargets = isTargetValid();
        // horizontalOffset = getHorizontalOffset();
        // verticalOffset = getVerticalOffset();
        // targetArea = getTargetArea();
        SmartDashboard.putBoolean("LL Valid Target", validTargets);
        SmartDashboard.putNumber("Vision Loc X", botPose3d.getX());
        SmartDashboard.putNumber("Target Latancy", getTargetLatency());
        // color = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        // if (color == DriverStation.Alliance.Blue) {
        // isRedAlliance = false;
        // } else {
        // isRedAlliance = true;
        // }
        if (isTargetValid()) {
            botPose3d = getBotPose3d();
            visonLockAck = true;
        }

    }

    public boolean isTargetValid() {
        return (table.getEntry("tv").getDouble(0) == 1);
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

    public double getTargetLatency() {
        return table.getEntry("tl").getDouble(0);
    }

    public double getCaptureLatency() {
        return table.getEntry("cl").getDouble(0);
    }

    public Pose3d getLastPose3d() {
        return botPose3d;
    }

    public boolean hasLockedVisionTarget() {
        return visonLockAck;
    }

    public Pose3d getBotPose3d() {
        var poseArrary = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
        // if (isRedAlliance) {
        // poseArrary = table.getEntry("botpose_wpired").getDoubleArray(new
        // double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        // } else {
        // poseArrary = table.getEntry("botpose_wpiblue").getDoubleArray(new
        // double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        // }
        poseArrary = table.getEntry("botpose_wpiblue")
                .getDoubleArray(new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 });
        // botPose3d = new Pose3d(poseArrary[0], poseArrary[1], poseArrary[2],
        // new Rotation3d(poseArrary[3], poseArrary[4], poseArrary[5]));
        // return botPose3d;
        // System.out.println(poseArrary[0]);
        return new Pose3d(poseArrary[0], poseArrary[1], poseArrary[2], new Rotation3d(Math.toRadians(poseArrary[3]),
                Math.toRadians(poseArrary[4]), Math.toRadians(poseArrary[5])));
    }
}
