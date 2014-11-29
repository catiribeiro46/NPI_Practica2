
using System;
using System.Windows.Media.Animation;
using Microsoft.Kinect;
using System.IO;
using System.Windows;
using System.Windows.Media;



namespace Microsoft.Samples.Kinect.SkeletonBasics
{
    class fitness
    {

        public Movimiento1 mov1;
        private Movimiento2 mov2;
        private Movimiento3 mov3;
        private Movimiento4 mov4;
        private Movimiento5 mov5;
        public String estado;
        public int contador;
        public String proxMov;
        public SkeletonPoint projectedWristR;
        public SkeletonPoint projectedWristL;
        public SkeletonPoint projectedAnkR;
        public SkeletonPoint projectedAnkL;
        public int deteccion5;
        private int fase;
        DateTime startTime;
        DateTime finishTime;
        DateTime timeControl;

        
        //public TimeSpan toStartWithValue;
        public fitness()
        {
            mov1 = new Movimiento1();
            mov2 = new Movimiento2();
            mov3 = new Movimiento3();
            mov4 = new Movimiento4();
            mov5 = new Movimiento5();
            estado = "A obter datos";
            deteccion5 = 0;
            fase = 0;
            //note that we need to see the timer in the screen for each start of a new movement will restart.
            //TimeSpan receives int64 in nanoseconds 3s=3000000000nanoseconds
            proxMov = "Primera Posición: Brazos relajados y Piernas cerradas";
           // String contador = TimeSpan.Zero.Seconds.ToString();
            contador = 0;
            startTime = DateTime.Now;
            finishTime = DateTime.Now;
            timeControl = DateTime.Now;
        }
        public String getEstado()
        {
            return estado;
        }
        // new method to get to verify that returns true if its in Time
        public bool inTime(DateTime t1, DateTime t2)
        {
            int diff = t1.Second - t2.Second;
            if (diff >= 0) return true;
            else return false;

        }
        //new method to count time to get the correct movement well done 
        public int newcountTime(DateTime t1, DateTime t2)
        {   
            int diff;
            if (inTime(t1,t2)) return diff = t1.Second - t2.Second;
            else return diff = 0;
        }

        public bool controlTime(DateTime t1, DateTime t2)
        {

            if (t2.CompareTo(t1) <= 0) return true;
            else return false;
        }


        public String countTime(DateTime t1, DateTime t2)
        {   while(t2.CompareTo(t1)<=0){
                String tiempodecresc = t1.Subtract(t2).ToString();
               // t2 = DateTime.Now;
                return tiempodecresc.ToString();
            }
            return TimeSpan.Zero.ToString();
        }
        public void start()
        {
            startTime = DateTime.Now;
            finishTime = startTime.AddSeconds(30.0);
            
            
        }

        public bool deteccion(Skeleton esqueleto)
        {
            start();
            timeControl = startTime.AddSeconds(3.0);
            //need to get more visual informacion in the screen
            // user must see the messages in time sent from the Kinect
            
            if (mov1.inicial(esqueleto) && fase==0 )
            { 
                DateTime t2 = DateTime.Now;
                while (inTime(timeControl, t2))
                {
                    estado = "Posición 1 del movimiento está Correcto";
                    proxMov = "Proxima Posición (2): Brazos en cruz y Piernas cerradas";
                    projectedWristL = mov1.projectedPointWristL(esqueleto);
                    projectedWristR = mov1.projectedPointWristR(esqueleto);
                    projectedAnkL = mov1.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov1.projectedPointAnkR(esqueleto);
                    contador = newcountTime(timeControl, t2);
                    //fase = 1;
                    return true;
                }
                timeControl = timeControl.AddSeconds(2.0);
                fase = 1;
                return true;
            }
            if (mov2.inicial(esqueleto) && fase==1)
            {
                timeControl = timeControl.AddSeconds(3.0);
                DateTime t2=DateTime.Now;
                while (inTime(timeControl, t2))
                {
                    estado = "Posición 2 del movimiento está Correcto";
                    projectedWristL = mov2.projectedPointWristL(esqueleto);
                    projectedWristR = mov2.projectedPointWristR(esqueleto);
                    projectedAnkL = mov1.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov1.projectedPointAnkR(esqueleto);
                    contador = newcountTime(timeControl, t2);
                    proxMov = "Proxima Posición (3): Brazos en cruz y levante la pierna izquierda hacia la cadera";
                    //fase = 2;
                    return true;
                }
                timeControl = timeControl.AddSeconds(2.0);
                fase = 2;
                return true;
            }
            if (mov3.inicial(esqueleto) && fase==2)
           {
               timeControl = timeControl.AddSeconds(3.0);
               DateTime t2 = DateTime.Now;
               while (inTime(timeControl, t2))
               {
                   projectedWristL = mov2.projectedPointWristL(esqueleto);
                    projectedWristR = mov2.projectedPointWristR(esqueleto);
                    projectedAnkL = mov3.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov1.projectedPointAnkR(esqueleto);
                    estado = "Posición 3 del movimiento está Correcto";
                    contador = newcountTime(timeControl, t2);
                    proxMov = "Proxima Posición (4): Brazos en cruz y coloque su pie izquierdo en el suelo";
                    //fase = 3;
                    return true;
                }
               timeControl = timeControl.AddSeconds(2.0);
                fase = 3;
                return true;
            }
            if (mov4.inicial(esqueleto) && fase==3)
            {
                timeControl = timeControl.AddSeconds(3.0);
                DateTime t2 = DateTime.Now;
                while (inTime(timeControl, t2))
                {
                    projectedWristL = mov2.projectedPointWristL(esqueleto);
                    projectedWristR = mov2.projectedPointWristR(esqueleto);
                    projectedAnkL = mov4.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov4.projectedPointAnkR(esqueleto);
                    estado = "Posición 4 del movimiento está Correcto";
                    contador = newcountTime(timeControl, t2);
                    proxMov = "Proxima Posición (5): Brazos en cruz y levante la pierna derecha hacia la cadera";
                    //fase = 4;
                    return true;
                }
                timeControl = timeControl.AddSeconds(2.0);
                fase = 4;
                return true;
            }
            if (mov5.inicial(esqueleto) && fase==4)
            {
                timeControl = timeControl.AddSeconds(3.0);
                DateTime t2 = DateTime.Now;
                while (inTime(timeControl, t2))
                {
                   projectedWristL = mov2.projectedPointWristL(esqueleto);
                    projectedWristR = mov2.projectedPointWristR(esqueleto);
                    projectedAnkL = mov5.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov5.projectedPointAnkR(esqueleto);
                    estado = "Posición 5 del movimiento está Correcto";
                    contador = newcountTime(timeControl, t2);
                    proxMov = "Puede volver ha hacer todos los movimientos.";
                    //fase = 5;
                    return true;
                }
                timeControl = timeControl.AddSeconds(2.0);
                fase = 5;
                return true;  
            }

            else
            {
                
                if (fase == 0)
                {   
                    estado = "Movimiento incorrecto";
                    //contador = new TimeSpan(3000000000).ToString();
                    proxMov = "Vuelve a hacer la primera Posición ";
                    projectedWristL = mov1.projectedPointWristL(esqueleto);
                    projectedWristR = mov1.projectedPointWristR(esqueleto);
                    projectedAnkL = mov1.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov1.projectedPointAnkR(esqueleto);
                    return false;
                }
                if (fase == 1)
                {
                    estado = "Movimiento incorrecto";
                    //contador = new TimeSpan(3000000000).ToString();
                    proxMov = "Posición (2): Brazos en cruz y Piernas cerradas";
                    projectedWristL = mov2.projectedPointWristL(esqueleto);
                    projectedWristR = mov2.projectedPointWristR(esqueleto);
                    projectedAnkL = mov1.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov1.projectedPointAnkR(esqueleto);
                    return false;
                }
                if (fase == 2)
                {
                    estado = "Movimiento incorrecto";
                    //contador = new TimeSpan(3000000000).ToString();
                    proxMov = "Posición (3): Brazos en cruz y levante la pierna izquierda hacia la cadera";
                    projectedWristL = mov2.projectedPointWristL(esqueleto);
                    projectedWristR = mov2.projectedPointWristR(esqueleto);
                    projectedAnkL = mov3.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov1.projectedPointAnkR(esqueleto);
                    return false;
                }
                if (fase == 3)
                {
                    estado = "Movimiento incorrecto";
                    //contador = new TimeSpan(3000000000).ToString();
                    proxMov = "Posición (4): Brazos en cruz y coloque su pie izquierdo en el suelo";
                    projectedWristL = mov2.projectedPointWristL(esqueleto);
                    projectedWristR = mov2.projectedPointWristR(esqueleto);
                    projectedAnkL = mov4.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov4.projectedPointAnkR(esqueleto);
                    return false;
                }
                if (fase == 4)
                {
                    estado = "Movimiento incorrecto";
                    proxMov = "Posición (5): Brazos en cruz y pierna izquierda arriba";
                    
                    //contador = new TimeSpan(3000000000).ToString();
                    projectedWristL = mov2.projectedPointWristL(esqueleto);
                    projectedWristR = mov2.projectedPointWristR(esqueleto);
                    projectedAnkL = mov5.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov5.projectedPointAnkR(esqueleto);
                    return false;
                }
                else if (fase == 5)
                {
                   
                    projectedWristL = mov2.projectedPointWristL(esqueleto);
                    projectedWristR = mov2.projectedPointWristR(esqueleto);
                    projectedAnkL = mov5.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov5.projectedPointAnkR(esqueleto);
                    estado = "Terminado! Muy Bien! :) ";
                    proxMov = "Puede volver ha hacer todos los movimientos.";
                    
                    //contador = new TimeSpan(3000000000).ToString();
                    fase = 0;
                    deteccion5++;
                    if (deteccion5 == 5) fase = 6;
                    return false;
                }
                else if (fase == 6)
                {

                    projectedWristL = mov1.projectedPointWristL(esqueleto);
                    projectedWristR = mov1.projectedPointWristR(esqueleto);
                    projectedAnkL = mov1.projectedPointAnkL(esqueleto);
                    projectedAnkR = mov1.projectedPointAnkR(esqueleto);
                    estado = "Terminado! Muy Bien! :)";
                    proxMov = "Ha hecho 5 repeticiones de la secuencia";

                    //contador = new TimeSpan(3000000000).ToString();
                    return false;
                }
                return false;
                

            }

        }

        
    }

}
