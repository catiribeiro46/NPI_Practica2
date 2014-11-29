using System;
using System.IO;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Kinect;


namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class Movimiento2
    {
       
        public float alturaBrazo;
        public float alturaPierna;
        public Movimiento2()
        {
        }
        public bool IsAlignedBodyAndArmsOpen(Skeleton received)
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
            double radian = (15 * Math.PI) / 180;

            //caldulate admited error 5% that correspond to 9 degrees for each side

            double DistError = distShouLtoWristL * Math.Tan(radian);

            //determine of projected point from shoulder to wrist LEFT and RIGHT and then assume error
            double ProjectedPointWristLX = WriLPosX;
            double ProjectedPointWristLY = ShouLPosY;
            double ProjectedPointWristLZ = WriLPosZ;

            double ProjectedPointWristRX = WriRPosX;
            double ProjectedPointWristRY = ShouRPosY;
            double ProjectedPointWristRZ = WriRPosZ;

            //alturaBrazo = (float)distShouLtoWristL;
            //Create method to verify if the center of the body is completely aligned
            //head with shoulder center and with hip center
            if (Math.Abs(HeadCenterPosX - ShoulCenterPosX) <= 0.1 && Math.Abs(ShoulCenterPosX - HipCenterPosX) <= 0.1)
            {
                //if position of left wrist is between [ProjectedPointWrist-DistError,ProjectedPointWrist+DistError]
                if (Math.Abs(WriLPosY - ProjectedPointWristLY) <= DistError && Math.Abs(WriRPosY - ProjectedPointWristRY) <= DistError)
                {
                    
                    return true;
                }
                else return false;
            }
            else return false;

        }
        public bool AreFeetTogether(Skeleton received)
        {
            foreach (Joint joint in received.Joints)
            {
                if (joint.TrackingState == JointTrackingState.Tracked)
                {//first verify if the body is alignet and arms are in a relaxed position

                    //{here verify if the feet are together
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
                    double radian1 = (3.0 * Math.PI) / 180;
                    double DistErrorL = distHiptoAnkleL * Math.Tan(radian1);
                    //determine of projected point from HIP CENTER to LEFT ANKLE and RIGHT and then assume error
                    double ProjectedPointFootLX = HipCenterPosX;
                    double ProjectedPointFootLY = AnkLPosY;
                    double ProjectedPointFootLZ = HipCenterPosZ;

                    alturaPierna = (float)distHiptoAnkleL;

                    // could variate AnkLposX and AnkLPosY
                    if (Math.Abs(AnkLPosX - ProjectedPointFootLX) <= DistErrorL && Math.Abs(AnkRPosX - ProjectedPointFootLX) <= DistErrorL)
                    {
                        alturaPierna = (float)distHiptoAnkleL;
                        return true;
                    }
                    else
                        return false;

                }//CLOSE if (joint.TrackingState == JointTrackingState.Tracked)
                else return false;
            }//close foreach


            return false;
        } //close method AreFeetTogether

        //need to give projected points for visual user interface help
        //projectedPoint1 will give coordinates for the left wrist posicion
        public SkeletonPoint projectedPointWristL(Skeleton received)
        {
            float ShouLPosY = received.Joints[JointType.ShoulderLeft].Position.Y;
            double WriLPosX = received.Joints[JointType.WristLeft].Position.X;
            double ShouLPosX = received.Joints[JointType.ShoulderLeft].Position.X;
            double distShouLtoWristL = Math.Abs(ShouLPosX - WriLPosX);
            //the wrist will be near by the corresponding hip side
            //test with other Y coordinate (length between Shoulder to Wrist)
            alturaBrazo = (float)distShouLtoWristL;
            //float projectedWristLY = received.Joints[JointType.HipLeft].Position.Y;
            float projectedWristLY = received.Joints[JointType.ShoulderLeft].Position.Y;
            float projectedWristLX = received.Joints[JointType.ShoulderLeft].Position.X - alturaBrazo;
            float projectedWristLZ = received.Joints[JointType.ShoulderLeft].Position.Z;
            //SkeletonPoint projectedWrist = received.Joints[JointType.WristLeft].Position;
            SkeletonPoint projectedWrist = new SkeletonPoint();
            projectedWrist.X = projectedWristLX;
            projectedWrist.Y = projectedWristLY;
            projectedWrist.Z = projectedWristLZ;

            return projectedWrist;
        }
        //need to give projected points for visual user interface help
        //projectedPoint2 will give coordinates for the right wrist posicion
        public SkeletonPoint projectedPointWristR(Skeleton received)
        {
            float ShouRPosY = received.Joints[JointType.ShoulderRight].Position.Y;
            float ShouLPosY = received.Joints[JointType.ShoulderLeft].Position.Y;
            double WriLPosX = received.Joints[JointType.WristLeft].Position.X;
            double ShouLPosX = received.Joints[JointType.ShoulderLeft].Position.X;
            double distShouLtoWristL = Math.Abs(ShouLPosX - WriLPosX);
            //the wrist will be near by the corresponding hip side
            //test with other Y coordinate (length between Shoulder to Wrist)
            alturaBrazo = (float)distShouLtoWristL;
            //the wrist will be near by the corresponding hip side
            //test with other Y coordinate (length between Shoulder to Wrist)

            //float projectedWristLY = received.Joints[JointType.HipLeft].Position.Y;
            float projectedWristRY = ShouRPosY;
            float projectedWristRX = received.Joints[JointType.ShoulderRight].Position.X + alturaBrazo;
            float projectedWristRZ = received.Joints[JointType.ShoulderRight].Position.Z;
            //SkeletonPoint projectedWrist = received.Joints[JointType.WristLeft].Position;
            SkeletonPoint projectedWrist = new SkeletonPoint();
            projectedWrist.X = projectedWristRX;
            projectedWrist.Y = projectedWristRY;
            projectedWrist.Z = projectedWristRZ;

            return projectedWrist;
        }
        
       
        public bool inicial(Skeleton received)
        {
            if (IsAlignedBodyAndArmsOpen(received) && AreFeetTogether(received)) return true;
            else return false;
        }


    }
}