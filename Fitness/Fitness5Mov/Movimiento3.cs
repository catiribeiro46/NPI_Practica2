using System;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;


namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class Movimiento3
    {
        public float alturaRodilla;
        public float posXRodilla;
        public float alturaPierna;
        public Movimiento3()
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

            double AnkRPosY = received.Joints[JointType.AnkleRight].Position.Y;
            double distHiptoAnkleR = HipCenterPosY - AnkRPosY;

            //caldulate admited error 5% that correspond to 9 degrees for each side

            double DistError = distShouLtoWristL * Math.Tan(radian);


            //determine of projected point from shoulder to wrist LEFT and RIGHT and then assume error
            double ProjectedPointWristLX = WriLPosX;
            double ProjectedPointWristLY = ShouLPosY;
            double ProjectedPointWristLZ = WriLPosZ;

            double ProjectedPointWristRX = WriRPosX;
            double ProjectedPointWristRY = ShouRPosY;
            double ProjectedPointWristRZ = WriRPosZ;

            alturaPierna = (float)distHiptoAnkleR;
            alturaRodilla = (float)received.Joints[JointType.KneeRight].Position.Y;
            //Create method to verify if the center of the body is completely aligned
            //head with shoulder center and with hip center
            if (Math.Abs(HeadCenterPosX - ShoulCenterPosX) <= 0.1 && Math.Abs(ShoulCenterPosX - HipCenterPosX) <= 0.1)
            {
                
                //if position of left wrist is between [ProjectedPointWrist-DistError,ProjectedPointWrist+DistError]
                if (Math.Abs(WriLPosY - ProjectedPointWristLY) <= DistError && Math.Abs(WriRPosY - ProjectedPointWristRY) <= DistError)
                {
                    alturaRodilla = (float)received.Joints[JointType.KneeRight].Position.Y;
                    return true;
                }
                else return false;
            }
            else return false;

        }
        public bool AreFeetSeparate(Skeleton received)
        {

            foreach (Joint joint in received.Joints)
            {
                if (joint.TrackingState == JointTrackingState.Tracked)
                {//first verify if the body is alignet and arms are in a relaxed position


                    //{//here verify if the feet are together
                    //use the same strategy that was used in the previous case of the arms in a  relaxed position
                    double HipCenterPosX = received.Joints[JointType.HipCenter].Position.X;
                    double HipCenterPosY = received.Joints[JointType.HipCenter].Position.Y;
                    double HipCenterPosZ = received.Joints[JointType.HipCenter].Position.Z;

                    //if left ankle is very close to right ankle then verify the rest of the skeleton points
                    //if (received.Joints[JointType.AnkleLeft].Equals(received.Joints[JointType.AnkleRight])) 
                    double AnkLPosX = received.Joints[JointType.AnkleLeft].Position.X;
                    double AnkLPosY = received.Joints[JointType.AnkleLeft].Position.Y;
                    double AnkLPosZ = received.Joints[JointType.AnkleLeft].Position.Z;

                    double AnkRPosX = received.Joints[JointType.AnkleRight].Position.X;
                    double AnkRPosY = received.Joints[JointType.AnkleRight].Position.Y;
                    double AnkRPosZ = received.Joints[JointType.AnkleRight].Position.Z;
                    //assume that the distance Y between HipCenter to each foot is the same
                    double distHiptoAnkleL = HipCenterPosY - AnkLPosY;
                    //caldulate admited error 5% that correspond to 9 degrees for each side
                    double radian1 = (4.5 * Math.PI) / 180;
                    double DistErrorL = distHiptoAnkleL * Math.Tan(radian1);
                    //determine of projected point from HIP CENTER to LEFT ANKLE and RIGHT and then assume error
                    double ProjectedPointFootLX = HipCenterPosX;
                    double ProjectedPointFootLY = AnkLPosY;
                    double ProjectedPointFootLZ = HipCenterPosZ;

                    double radian2 = (30 * Math.PI) / 180;
                    double DistSeparateFoot = distHiptoAnkleL * Math.Tan(radian2);
                    //DrawingVisual MyDrawingVisual = new DrawingVisual();


                    // could variate AnkLposX and AnkLPosY
                    if (Math.Abs(AnkRPosX - AnkLPosX) <= Math.Abs(DistSeparateFoot + DistErrorL) && Math.Abs(AnkRPosX - AnkLPosX) >= Math.Abs((DistSeparateFoot) - DistErrorL))
                        return true;
                    else return false;


                }//CLOSE if (joint.TrackingState == JointTrackingState.Tracked)
                else return false;
            }//close foreach


            return false;
        }
        public bool DrawBoneUpKnee(Skeleton received)
        {
            double AnkLPosX = received.Joints[JointType.AnkleLeft].Position.X;
            double AnkLPosY = received.Joints[JointType.AnkleLeft].Position.Y;
            double AnkLPosZ = received.Joints[JointType.AnkleLeft].Position.Z;

            double AnkRPosX = received.Joints[JointType.AnkleRight].Position.X;
            double AnkRPosY = received.Joints[JointType.AnkleRight].Position.Y;
            double AnkRPosZ = received.Joints[JointType.AnkleRight].Position.Z;

            double HipLPosX = received.Joints[JointType.HipLeft].Position.X;
            double HipLPosY = received.Joints[JointType.HipLeft].Position.Y;
            double HipLPosZ = received.Joints[JointType.HipLeft].Position.Z;

            double KneeLPosY = received.Joints[JointType.KneeLeft].Position.Y;
            double KneeLPosX = received.Joints[JointType.KneeLeft].Position.X;
            double KneeLPosZ = received.Joints[JointType.KneeLeft].Position.Z;

            double KneeRPosY = received.Joints[JointType.KneeRight].Position.Y;
            double KneeRPosX = received.Joints[JointType.KneeRight].Position.X;
            double KneeRPosZ = received.Joints[JointType.KneeRight].Position.Z;

            double distHiptoKneeL = HipLPosY - KneeRPosY;

            double ProjectedAnkLPosX = HipLPosX;
            double ProjectedAnkLPosY = KneeRPosY;
            double ProjectedAnkLPosZ = HipLPosZ;

            double ProjectedKneeLPosX = HipLPosX;
            double ProjectedKneeLPosY = HipLPosY;
            double ProjectedKneeLPosZ = HipLPosZ - distHiptoKneeL;
            double radian = (15 * Math.PI) / 180;
            alturaRodilla = (float)distHiptoKneeL;
            posXRodilla = (float)HipLPosX;
            double DistErrorKneeL = distHiptoKneeL * Math.Tan(radian);

            if (Math.Abs(ProjectedKneeLPosY - KneeLPosY) <= DistErrorKneeL && Math.Abs(ProjectedAnkLPosX - AnkRPosX) <= 0.05)
            {
                alturaRodilla = (float)KneeLPosY;
                posXRodilla = (float)AnkLPosX;
                return true;
            }
            else return false;

        }

        //each Ankle should be in line with HipCenter and with the ground
        //the wrist will be near by the corresponding hip side
        //test with other Y coordinate (length between Shoulder to Wrist)
        public SkeletonPoint projectedPointAnkR(Skeleton received)
        {
            float projectedAnkRY = received.Joints[JointType.HipRight].Position.Y + alturaPierna;
            float projectedAnkRX = received.Joints[JointType.HipCenter].Position.X;
            float projectedAnkRZ = received.Joints[JointType.HipRight].Position.Z;
            SkeletonPoint projectedAnk = new SkeletonPoint();
            projectedAnk.X = projectedAnkRX;
            projectedAnk.Y = projectedAnkRY;
            projectedAnk.Z = projectedAnkRZ;

            return projectedAnk;
        }
        //need to give projected points for visual user interface help
        //projectedPoint3 will give coordinates for the right ankle posicion
        public SkeletonPoint projectedPointAnkL(Skeleton received)
        {
            //each Ankle should be in line with HipCenter and with the ground
            float projectedAnkLY = received.Joints[JointType.HipLeft].Position.Y - alturaRodilla;
            float projectedAnkLX = posXRodilla;
            float projectedAnkLZ = received.Joints[JointType.HipLeft].Position.Z + alturaRodilla;
            SkeletonPoint projectedAnk = new SkeletonPoint();
            projectedAnk.X = projectedAnkLX;
            projectedAnk.Y = projectedAnkLY;
            projectedAnk.Z = projectedAnkLZ;

            return projectedAnk;
        }
        public bool inicial(Skeleton received)
        {
            if (IsAlignedBodyAndArms(received) && DrawBoneUpKnee(received)) return true;
            else return false;
        }
        
    }
}
   
