package frc.ChenryLib;

public class InverseKinematics {
        private double trackWidth;
        
        public InverseKinematics (double itrackWidth){
            trackWidth = itrackWidth;
        }
        
        public double toRightWheelSpeeds(double linearVelocity, double angularVelocity){
            return (linearVelocity + trackWidth / 2 * angularVelocity);
        }
        public double toLeftWheelSpeeds(double linearVelocity, double angularVelocity){
            return (linearVelocity - trackWidth / 2 * angularVelocity);
        }
        
        public double toRightWheelSpeedsCurvature(double linearVelocity, double curvature){
            return linearVelocity * (2 + curvature * trackWidth) / 2;
        }
        
        public double toLeftWheelSpeedsCurvature(double linearVelocity, double curvature){
            return linearVelocity * (2 - curvature * trackWidth) / 2;
        }
        
}
