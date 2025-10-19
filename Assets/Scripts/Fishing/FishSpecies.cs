using UnityEngine;

namespace Fishing
{
    [CreateAssetMenu(menuName = "Fishing/Fish Species", fileName = "NewFishSpecies")]
    public class FishSpecies : ScriptableObject
    {
        [Header("Visuals")]
        public GameObject prefab;
        public float lengthMeters = 0.6f;

        [Header("Locomotion")]
        public float cruiseSpeed = 1.2f;
        public float maxSpeed = 3.5f;
        public float acceleration = 3.0f;      // m/s²
        public float turnRateDeg = 140f;       // deg/s

        [Header("Habitat (depth below surface)")]
        public Vector2 preferredDepthRange = new Vector2(1.5f, 6f);
        [Tooltip("Bias toward staying inside volume (0..1).")]
        public float volumeBias = 0.8f;
        [Tooltip("Bias toward staying in preferred depth band (0..1).")]
        public float depthBias = 0.5f;

        [Header("Senses")]
        public float visionRange = 12f;
        [Range(1f, 180f)] public float visionFOV = 120f;
        public float curiosity = 0.5f;
        public float investigateTime = 3.0f;

        [Header("Boids (radii in meters)")]
        [Tooltip("Separation applies to ALL species.")]
        public float sepRadius = 0.8f;
        [Tooltip("Alignment/cohesion only with SAME species.")]
        public float alignRadius = 3.0f;
        public float cohRadius = 3.5f;

        [Header("Boids Weights")]
        public float separationWeight = 1.4f;
        public float alignmentWeight = 0.9f;
        public float cohesionWeight = 0.7f;

        [Header("Obstacle Avoidance")]
        public float obstacleProbeDist = 1.2f;
        public float obstacleRadius = 0.25f;

        [Header("Wander / Life")]
        public float wanderJitter = 0.6f;
        public float wanderRadius = 1.6f;
        public float wanderInterval = 0.6f;

        [Header("Walls")]
        [Tooltip("Distance from walls to start steering away.")]
        public float wallMargin = 1.0f;
        [Tooltip("How strongly walls push you back (relative to accel).")]
        public float wallAvoidStrength = 1.6f;
    }
}
