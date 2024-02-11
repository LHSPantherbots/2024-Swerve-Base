package frc.utils;

import com.revrobotics.SparkPIDController;

public class SparkPIDWrapper {
    private SparkPIDController sparkPID;

    SparkPIDWrapper(SparkPIDController sparkPID) {
        this.sparkPID = sparkPID;
    }

    public SparkPIDController getSparkPIDController() {
        return this.sparkPID;
    }

    public void setP(double gain) {

        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setP(gain);
            if (this.sparkPID.getP() == gain) {
                break;
            }
        }
    }

    public void setI(double gain) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setI(gain);
            if (this.sparkPID.getI() == gain) {
                break;
            }
        }
    }

    public void setD(double gain) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setD(gain);
            if (this.sparkPID.getD() == gain) {
                break;
            }
        }

    }

    public void setIZone(double IZone) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setIZone(IZone);
            if (this.sparkPID.getIZone() == IZone) {
                break;
            }
        }
    }

    public void setFF(double gain) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setFF(gain);
            if (this.sparkPID.getFF() == gain) {
                break;
            }
        }
    }

    public void setOutputRange(double min, double max) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setOutputRange(min, max);
            if (this.sparkPID.getOutputMin() == min && this.sparkPID.getOutputMax() == max) {
                break;
            }
        }
    }

    public void setSmartMotionMaxVelocity(double maxVal, int slotID) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setSmartMotionMaxVelocity(maxVal, slotID);
            if (this.sparkPID.getSmartMotionMaxVelocity(slotID) == maxVal) {
                break;
            }
        }
    }

    public void setSmartMotionMinOutputVelocity(double minVel, int slotID) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setSmartMotionMinOutputVelocity(minVel, slotID);
            if (this.sparkPID.getSmartMotionMinOutputVelocity(slotID) == minVel) {
                break;
            }
        }
    }

    public void setSmartMotionMaxAccel(double maxAccel, int slotID) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setSmartMotionMaxAccel(maxAccel, slotID);
            if (this.sparkPID.getSmartMotionMaxAccel(slotID) == maxAccel) {
                break;
            }
        }
    }

    public void setSmartMotionAllowedClosedLoopError(double allowedErr, int slotID) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setSmartMotionAllowedClosedLoopError(allowedErr, slotID);
            if (this.sparkPID.getSmartMotionAllowedClosedLoopError(slotID) == allowedErr) {
                break;
            }
        }
    }

    public void setPositionPIDWrappingEnabled(boolean enable) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setPositionPIDWrappingEnabled(enable);
            if (this.sparkPID.getPositionPIDWrappingEnabled() == enable) {
                break;
            }
        }
    }

    public void setPositionPIDWrappingMinInput(double min) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setPositionPIDWrappingMinInput(min);
            if (this.sparkPID.getPositionPIDWrappingMinInput() == min) {
                break;
            }
        }
    }

    public void setPositionPIDWrappingMaxInput(double max) {
        for (int i = 0; i <= 5; i++) {
            this.sparkPID.setPositionPIDWrappingMaxInput(max);
            if (this.sparkPID.getPositionPIDWrappingMaxInput() == max) {
                break;
            }
        }
    }
}
