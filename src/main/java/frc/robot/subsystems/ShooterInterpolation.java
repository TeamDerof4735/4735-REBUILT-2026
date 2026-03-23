package frc.robot.subsystems;

public class ShooterInterpolation {

    public static class ShotPoint {
        public double distance;
        public double rpm;

        public ShotPoint(double distance, double rpm) {
            this.distance = distance;
            this.rpm = rpm;
        }
    }

    // Insert Distances
    private static final ShotPoint[] table = {
        new ShotPoint(1.5, 2000),
        new ShotPoint(2.0, 1980),
        new ShotPoint(2.5, 2100),
        new ShotPoint(3.0, 2200),
        new ShotPoint(3.5, 2500),
        new ShotPoint(4.0, 5000)
    };


    public double calculateRPM(double limelightDistance) {
        // si esta out of range
        if (limelightDistance <= table[0].distance) {
            return table[0].rpm;
        }

        if (limelightDistance >= table[table.length - 1].distance) {
            return table[table.length - 1].rpm;
        }

        for (int i = 0; i < table.length - 1; i++) {
            ShotPoint p1 = table[i];
            ShotPoint p2 = table[i + 1];

            if (limelightDistance >= p1.distance && limelightDistance <= p2.distance) {

                // Interpolacion
                double t = (limelightDistance - p1.distance) / (p2.distance - p1.distance);

                return p1.rpm + t * (p2.rpm - p1.rpm);
            }
        }
        return 0;
    }
}