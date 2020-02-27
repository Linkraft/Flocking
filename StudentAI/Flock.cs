using System.Collections.Generic;
using AI.SteeringBehaviors.Core;

namespace AI.SteeringBehaviors.StudentAI {
    public class Flock {
        public float AlignmentStrength { get; set; }
        public float CohesionStrength { get; set; }
        public float SeparationStrength { get; set; }
        public List<MovingObject> Boids { get; protected set; }
        public Vector3 AveragePosition { get; set; }
        protected Vector3 AverageForward { get; set; }
        public float FlockRadius { get; set; }

        public Flock() {
            // Default constructor
            AverageForward = new Vector3(0, 0, 0);
            AveragePosition = new Vector3(0, 0, 0);

            AlignmentStrength = 1;
            CohesionStrength = 1;
            SeparationStrength = 1;
            FlockRadius = 50;
        }

        public virtual void Update(float deltaTime) {
            AverageForward = getAverageForward(Boids);
            AveragePosition = getAveragePosition(Boids);

            foreach(MovingObject boid in Boids) {
                Vector3 accel = calcAlignmentAccel(boid);
                accel += calcCohesionAccel(boid);
                accel += calcSeparationAccel(boid);
                accel *= boid.MaxSpeed * deltaTime;
                boid.Velocity += accel;

                if (boid.Velocity.Length > boid.MaxSpeed) {
                    boid.Velocity.Normalize();
                    boid.Velocity *= boid.MaxSpeed;
                }

                boid.Update(deltaTime);
            }

        }

        private Vector3 getAverageForward(List<MovingObject> boids) {
            int numBoids = boids.Count;
            Vector3 average = new Vector3(0, 0, 0);

            foreach (MovingObject boid in boids)
                average += boid.Velocity;

            average /= numBoids;
            return average;
        }

        private Vector3 getAveragePosition(List<MovingObject> boids) {
            int numBoids = boids.Count;
            Vector3 average = new Vector3(0, 0, 0);

            foreach (MovingObject boid in boids)
                average += boid.Position;

            average /= numBoids;
            return average;
        }


        private Vector3 calcAlignmentAccel(MovingObject boid) {
            Vector3 accel = AverageForward / boid.MaxSpeed;

            if (accel.Length > 1) accel.Normalize();

            return accel * AlignmentStrength;
        }

        private Vector3 calcCohesionAccel(MovingObject boid) {
            Vector3 accel = AveragePosition - boid.Position;
            float distance = accel.Length;
            accel.Normalize();

            // Set speed based on distance from flock
            if (distance < FlockRadius) accel *= distance / FlockRadius;

            return accel * CohesionStrength;
        }

        private Vector3 calcSeparationAccel(MovingObject boid) {
            Vector3 sum = new Vector3(0, 0, 0);

            foreach (MovingObject sibling in Boids) {
                if (sibling == boid) continue;

                Vector3 accel = boid.Position - sibling.Position;
                float distance = accel.Length;
                float safeDistance = boid.SafeRadius + sibling.SafeRadius;

                // If a collision is likely...
                if (distance < safeDistance) {
                    // Scale vector according to how close boids are
                    accel.Normalize();
                    accel *= (safeDistance - distance) / safeDistance;
                    sum += accel;
                }
            }

            if (sum.Length > 1) sum.Normalize();

            return sum * SeparationStrength;
        }

    }
}
