using System;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;

namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class Movimiento5
    {
        public float alturaRodilla;
        public float alturaPierna;
        public float posXRodilla;
        public Movimiento5()
        {
        }
        //are open arms and aligned body
        public bool IsAlignedBodyAndArms(Skeleton received)
        {
            double HipCenterPosX = received.Joints[JointType.HipCenter].Position.X;
            double HipCenterPosY = received.Joints[JointType.HipCenter].Position.Y;
            double HipCenterPosZ = received.Joints[JointType.HipCenter].Position.Z;

            double ShoulCenterPosX = received.Joints[JointType.ShoulderCenter].Position.X;
            double ShoulCenterPosY = received.Joints[JointType.ShoulderCenter].Position.Y;
            double ShoulCenterPosZ = received.Joints[JointType.ShoulderCenter].Position.Z;

            double HeadCenterPosX = received.Joints[JointType.Head].Position.X;
            double HeadCenterPosY = received.Joints[JointType.Head].Position.Y;
            double HeadCenterPosZ = received.Joints[JointType.Head].Position.Z;

            double ElbLPosX = received.Joints[JointType.ElbowLeft].Position.X;
            double ElbLPosY = received.Joints[JointType.ElbowLeft].Position.Y;

            double ElbRPosX = received.Joints[JointType.ElbowRight].Position.X;
            double ElbRPosY = received.Joints[JointType.ElbowRight].Position.Y;

            double WriLPosX = received.Joints[JointType.WristLeft].Position.X;
            double WriLPosY = received.Joints[JointType.WristLeft].Position.Y;
            double WriLPosZ = received.Joints[JointType.WristLeft].Position.Z;

            double WriRPosX = received.Joints[JointType.WristRight].Position.X;
            double WriRPosY = received.Joints[JointType.WristRight].Position.Y;
            double WriRPosZ = received.Joints[JointType.WristRight].Position.Z;

            double ShouLPosX = received.Joints[JointType.ShoulderLeft].Position.X;
            double ShouLPosY = received.Joints[JointType.ShoulderLeft].Position.Y;
            double ShouLPosZ = received.Joints[JointType.ShoulderLeft].Position.Z;

            double ShouRPosX = received.Joints[JointType.ShoulderRight].Position.X;
            double ShouRPosY = received.Joints[JointType.ShoulderRight].Position.Y;
            double ShouRPosZ = received.Joints[JointType.ShoulderRight].Position.Z;
            //have to change to correspond to the 5% error
            //distance from Shoulder to Wrist for the projection in line with shoulder
            double distShouLtoWristL = Math.Abs(ShouLPosX - WriLPosX);
            //caldulate admited error 5% that correspond to 9 degrees for each side
            double radian = (9 * Math.PI) / 180;

            //caldulate admited error 5% that correspond to 9 degrees for each side

            double DistError = distShouLtoWristL * Math.Tan(radian);
            //determine of projected point from shoulder to wrist LEFT and RIGHT and then assume error
            double ProjectedPointWristLX = WriLPosX;
            double ProjectedPointWristLY = ShouLPosY;
            double ProjectedPointWristLZ = WriLPosZ;

            double ProjectedPointWristRX = WriRPosX;
            double ProjectedPointWristRY = ShouRPosY;
            double ProjectedPointWristRZ = WriRPosZ;

            double AnkLPosY = received.Joints[JointType.AnkleLeft].Position.Y;
            double distHiptoAnkleL = HipCenterPosY - AnkLPosY;
            alturaPierna = (float)distHiptoAnkleL;
            alturaRodilla = (float)received.Joints[JointType.KneeLeft].Position.Y;
            //Create method to verify if the center of the body is completely aligned
            //head with shoulder center and with hip center
            if (Math.Abs(HeadCenterPosX - ShoulCenterPosX) <= 0.1 && Math.Abs(ShoulCenterPosX - HipCenterPosX) <= 0.1)
            {
                //if position of left wrist is between [ProjectedPointWrist-DistError,ProjectedPointWrist+DistError]
                if (Math.Abs(WriLPosY - ProjectedPointWristLY) <= DistError && Math.Abs(WriRPosY - ProjectedPointWristRY) <= DistError)
                {
                    alturaRodilla = (float)received.Joints[JointType.KneeLeft].Position.Y;
                    return true;
                }
                else return false;
            }
            else return false;

        }
        public bool DrawBoneUpKnee(Skeleton received)
        {
            double AnkLPosX = received.Joints[JointType.AnkleLeft].Position.X;
            double AnkLPosY = received.Joints[JointType.AnkleLeft].Position.Y;
            double AnkLPosZ = received.Joints[JointType.AnkleLeft].Position.Z;

            double AnkRPosX = received.Joints[JointType.AnkleRight].Position.X;
            double AnkRPosY = received.Joints[JointType.AnkleRight].Position.Y;
            double AnkRPosZ = received.Joints[JointType.AnkleRight].Position.Z;

            double HipRPosX = received.Joints[JointType.HipRight].Position.X;
            double HipRPosY = received.Joints[JointType.HipRight].Position.Y;
            double HipRPosZ = received.Joints[JointType.HipRight].Position.Z;

            double KneeLPosY = received.Joints[JointType.KneeLeft].Position.Y;
            double KneeLPosX = received.Joints[JointType.KneeLeft].Position.X;
            double KneeLPosZ = received.Joints[JointType.KneeLeft].Position.Z;

            double KneeRPosY = received.Joints[JointType.KneeRight].Position.Y;
            double KneeRPosX = received.Joints[JointType.KneeRight].Position.X;
            double KneeRPosZ = received.Joints[JointType.KneeRight].Position.Z;

            double distHiptoKneeL = HipRPosY - KneeLPosY;

            double ProjectedAnkRPosX = HipRPosX;
            double ProjectedAnkRPosY = KneeLPosY;
            double ProjectedAnkRPosZ = HipRPosZ;

            double ProjectedKneeRPosX = HipRPosX;
            double ProjectedKneeRPosY = HipRPosY;
            double ProjectedKneeRPosZ = HipRPosZ - distHiptoKneeL;
            double radian = (15 * Math.PI) / 180;
            alturaRodilla = (float) distHiptoKneeL;
            posXRodilla = (float)HipRPosX;
            //alturaRodilla = (float)received.Joints[JointType.HipRight].Position.Y - alturaPierna;
            double DistErrorKneeL = distHiptoKneeL * Math.Tan(radian);

            if (Math.Abs(ProjectedKneeRPosY - KneeRPosY) <= DistErrorKneeL && Math.Abs(ProjectedAnkRPosX - AnkLPosX) <= 0.05)
            {
                alturaRodilla = (float)KneeRPosY;
                posXRodilla = (float)AnkRPosX;
                return true;
            }
            else return false;

        }
        public SkeletonPoint projectedPointAnkL(Skeleton received)
        {
            //each Ankle should be in line with HipCenter and with the ground
            //the wrist will be near by the corresponding hip side
            //test with other Y coordinate (length between Shoulder to Wrist
            float projectedAnkLY = received.Joints[JointType.HipLeft].Position.Y - alturaPierna;
            float projectedAnkLX = received.Joints[JointType.HipCenter].Position.X;
            float projectedAnkLZ = received.Joints[JointType.HipLeft].Position.Z;
            SkeletonPoint projectedAnk = new SkeletonPoint();
            projectedAnk.X = projectedAnkLX;
            projectedAnk.Y = projectedAnkLY;
            projectedAnk.Z = projectedAnkLZ;

            return projectedAnk;
        }
     
        //need to give projected points for visual user interface help
        //projectedPoint3 will give coordinates for the right ankle posicion
        public SkeletonPoint projectedPointAnkR(Skeleton received)
        {
            //each Ankle should be in line with HipCenter and with the ground
            //float projectedWristLY = received.Joints[JointType.HipLeft].Position.Y;
            float projectedAnkRY = received.Joints[JointType.HipRight].Position.Y - alturaRodilla;
            float projectedAnkRX = posXRodilla;
            float projectedAnkRZ = received.Joints[JointType.HipRight].Position.Z + alturaRodilla;
            SkeletonPoint projectedAnk = new SkeletonPoint();
            projectedAnk.X = projectedAnkRX;
            projectedAnk.Y = projectedAnkRY;
            projectedAnk.Z = projectedAnkRZ;

            return projectedAnk;
            
        }
        public bool inicial(Skeleton received)
        {
            if (IsAlignedBodyAndArms(received) && DrawBoneUpKnee(received)) return true;
            else return false;
        }

    }
}
