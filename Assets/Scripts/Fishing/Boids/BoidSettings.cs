using UnityEngine;

namespace Fishing
{
    [CreateAssetMenu(menuName = "Fishing/Boids/Boid Settings", fileName = "BoidSettings")]
    public class BoidSettings : ScriptableObject
    {
        [Header("Neighbourhood")]
        public float perceptionRadius = 2.5f;
        public float avoidanceRadius = 1.0f;
        [Range(0f, 1f)] public float alignmentFOV = 0.9f; // cosine of max angle (Seb often uses FOV gates)

        [Header("Speeds & Steering")]
        public float maxSpeed = 3.5f;
        public float maxSteerForce = 0.5f;

        [Header("Rule Weights")]
        public float separationWeight = 1.5f;
        public float alignmentWeight = 1.0f;
        public float cohesionWeight = 1.0f;

        [Header("World / Bounds")]
        public float boundsRadius = 1.5f;         // soft “stay away from bounds” radius
        public float avoidCollisionWeight = 10f;  // strength of collision avoid steering
        public float collisionAvoidDst = 2.0f;    // how far to ray/sphere cast
        [Range(6, 64)] public int numViewDirections = 26;

        [Header("Obstacle Cast")]
        public float obstacleCastRadius = 0.2f;
        public LayerMask obstacleMask = ~0;
    }
}
