using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using Microsoft.Xna.Framework;

namespace Marionette
{
    public class Server
    {
        private double[] robotPossitionCardinal;
        private double[] robotCommandedCardinal;
        private double[] robotPossitionAxis;
        private double[] robotCommandedAxis;
        private double[] robotPossitionCurrent;
        private double[] robotCommandedCurrent;
        private double[] robotVelocityCardinal;
        private double[] robotAccelerationCardinal;
        private double[] newPossitionCardinal;
        private double[] newVelocityCardinal;
        private double[] newAccelerationCardinal;
        private double[] newPossitionAxis;

        protected double[] PosCard
        {
            get
            {
                return robotPossitionCardinal;
            }
            private set
            {
            }
        }

        protected double[] PosAxis
        {
            get
            {
                return robotPossitionAxis;
            }
            private set
            {
            }
        }

        protected double[] VelCard
        {
            get
            {
                return robotVelocityCardinal;
            }
            private set
            {
            }
        }

        protected double[] AccCard
        {
            get
            {
                return robotAccelerationCardinal;
            }
            set
            {
            }
        }

        protected Quaternion PosQuat
        {
            get
            {
                return MakeQuaternion(PosCard);
            }
        }

        /// <summary>
        /// Creates a quaternion from Kuka coordinates
        /// </summary>
        public Quaternion MakeQuaternion(double[] poses)
        {
            Matrix Rz = Matrix.CreateRotationZ((float)poses[3]);
            Matrix Ry = Matrix.CreateRotationY((float)poses[4]);
            Matrix Rx = Matrix.CreateRotationX((float)poses[5]);
            Matrix Rotation = Matrix.Multiply(Matrix.Multiply(Rz, Ry), Rx);
            return Quaternion.CreateFromRotationMatrix(Rotation);
        }
    }
}
