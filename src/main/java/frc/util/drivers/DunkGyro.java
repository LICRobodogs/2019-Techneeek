package frc.util.drivers;

import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.AHRSProtocol.AHRSUpdateBase;
import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.ITimestampedDataSubscriber;
import frc.util.math.Rotation2d;

/**
 * Driver for a NavX board. Basically a wrapper for the {@link AHRS} class
 */
public class DunkGyro extends AHRS{
    protected class Callback implements ITimestampedDataSubscriber {
        @Override
        public void timestampedDataReceived(long system_timestamp, long sensor_timestamp, AHRSUpdateBase update,
                Object context) {
            synchronized (DunkGyro.this) {
                // This handles the fact that the sensor is inverted from our coordinate conventions.
                if (mLastSensorTimestampMs != kInvalidTimestamp && mLastSensorTimestampMs < sensor_timestamp) {
                    mYawRateDegreesPerSecond = 1000.0 * (-mYawDegrees - update.yaw)
                            / (double) (sensor_timestamp - mLastSensorTimestampMs);
                }
                mLastSensorTimestampMs = sensor_timestamp;
                mYawDegrees = -update.yaw;
            }
        }
    }

    protected AHRS mAHRS;

    protected Rotation2d mAngleAdjustment = Rotation2d.identity();
    protected double mYawDegrees;
    protected double mYawRateDegreesPerSecond;
    protected final long kInvalidTimestamp = -1;
    protected long mLastSensorTimestampMs;

    
    public DunkGyro(SPI.Port spi_port_id) {
        super(spi_port_id);
        mAHRS = new AHRS(spi_port_id, (byte) 200);
        resetState();
        mAHRS.registerCallback(new Callback(), null);
    }

    public synchronized void reset() {
        super.reset();
        resetState();
    }

    public synchronized void zeroYaw() {
        super.zeroYaw();
        resetState();
    }

    private void resetState() {
        mLastSensorTimestampMs = kInvalidTimestamp;
        mYawDegrees = 0.0;
        mYawRateDegreesPerSecond = 0.0;
    }

    public synchronized void setAngleAdjustment(Rotation2d adjustment) {
        mAngleAdjustment = adjustment;
    }

    protected synchronized double getRawYawDegrees() {
        return mYawDegrees;
    }


    public Rotation2d getMyYaw() {
        return mAngleAdjustment.rotateBy(Rotation2d.fromDegrees(getRawYawDegrees()));
    }
    public AHRS getAHRS() {
        return mAHRS;
    }

    public double getYawRateDegreesPerSec() {
        return mYawRateDegreesPerSecond;
    }

    public double getYawRateRadiansPerSec() {
        return 180.0 / Math.PI * getYawRateDegreesPerSec();
    }
}