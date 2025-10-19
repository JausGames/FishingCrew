using System.Collections.Generic;
using UnityEngine;

namespace Fishing
{
    [DefaultExecutionOrder(50)]
    public class FishBoidsController : MonoBehaviour
    {
        [Header("Scope")]
        public WaterArea water;                  // assign the same WaterArea this controller governs
        public BoidSettings settings;            // Seb-like settings
        public bool autoDiscoverFish = true;     // if on, finds Fish every few seconds
        public float rediscoverInterval = 2.0f;

        [Header("Species gating")]
        [Tooltip("Apply Alignment & Cohesion only with same species. Separation always considers all fish.")]
        public bool sameSpeciesForAlignCohesion = true;

        readonly List<Fish> _agents = new List<Fish>();
        Vector3[] _viewDirs;
        float _nextDiscoverTime;

        void OnEnable()
        {
            if (!water) water = GetComponent<WaterArea>();
            _viewDirs = BoidHelper.GetViewDirections(settings ? settings.numViewDirections : 26);
            DiscoverAgents();
        }

        void Update()
        {
            if (!water || !settings) return;

            if (autoDiscoverFish && Time.time >= _nextDiscoverTime)
                DiscoverAgents();

            // Boids update (Seb’s flow): compute steering for each agent then integrate
            // To reduce bias, first compute desired steering vectors in a temp array
            int n = _agents.Count;
            if (n == 0) return;

            var desiredVel = new Vector3[n];

            for (int i = 0; i < n; i++)
            {
                desiredVel[i] = ComputeDesiredVelocity(_agents[i]);
            }

            float dt = Mathf.Clamp(Time.deltaTime, 0f, 0.05f);

            // Integrate like Seb: clamp steer force -> apply to current velocity -> clamp to maxSpeed
            for (int i = 0; i < n; i++)
            {
                var f = _agents[i];
                if (!f || !f.enabled) continue;

                var v = f.velocity;
                Vector3 steer = desiredVel[i] - v;
                steer = Vector3.ClampMagnitude(steer, settings.maxSteerForce);

                v += steer * dt;
                v = Vector3.ClampMagnitude(v, settings.maxSpeed);

                // Apply bounds: softly push away from WaterArea bounds just like Seb’s “boundsRadius”
                v += BoundsSteer(f.transform.position) * dt;

                // Update forward/velocity/position (we keep your Fish Update lightweight by doing it here)
                f.forward = v.sqrMagnitude > 1e-5f ? Vector3.Slerp(f.forward, v.normalized, 10f * dt) : f.forward;
                f.velocity = v + water.SampleFlow(f.transform.position);
                Vector3 pos = f.transform.position + f.velocity * dt;

                // Do not breach surface
                float minDepth = f.species.preferredDepthRange.x * 0.5f;
                pos.y = Mathf.Min(pos.y, water.surfaceY - minDepth);

                // Clamp inside as a last resort
                pos = water.ClampInside(pos, 0.02f);

                // Apply position (CharacterController disabled for kinematic set)
                var cc = f.GetComponent<CharacterController>();
                if (cc)
                {
                    cc.enabled = false; f.transform.position = pos; cc.enabled = true;
                }
                else f.transform.position = pos;

                // Face forward
                f.transform.rotation = Quaternion.Slerp(f.transform.rotation, Quaternion.LookRotation(f.forward, Vector3.up), 10f * dt);
            }
        }

        // -------- Core boids calc (Seb-like) --------
        Vector3 ComputeDesiredVelocity(Fish me)
        {
            Vector3 pos = me.transform.position;
            Vector3 forward = me.forward;

            Vector3 separation = Vector3.zero;
            Vector3 alignment = Vector3.zero;
            Vector3 cohesion = Vector3.zero;

            int sepCount = 0, aliCount = 0, cohCount = 0;

            float perceptionR2 = settings.perceptionRadius * settings.perceptionRadius;
            float avoidR2 = settings.avoidanceRadius * settings.avoidanceRadius;
            float minDotAlign = settings.alignmentFOV; // cosine gate

            // Neighbor loop (simple; upgrade to spatial hash later if needed)
            for (int i = 0; i < _agents.Count; i++)
            {
                var other = _agents[i];
                if (other == me || !other) continue;

                Vector3 to = other.transform.position - pos;
                float d2 = to.sqrMagnitude;
                if (d2 > perceptionR2) continue;

                // Separation applies to ALL species
                if (d2 < avoidR2 && d2 > 1e-6f)
                {
                    separation -= to / d2; // inverse-square push
                    sepCount++;
                }

                // Alignment + Cohesion only with SAME species (if enabled)
                if (!sameSpeciesForAlignCohesion || other.species == me.species)
                {
                    Vector3 dir = to.normalized;
                    if (Vector3.Dot(forward, dir) >= minDotAlign)
                    {
                        alignment += other.forward;
                        cohesion += other.transform.position;
                        aliCount++;
                        cohCount++;
                    }
                }
            }

            if (sepCount > 0) separation /= sepCount;
            if (aliCount > 0) alignment = (alignment / aliCount).normalized;
            if (cohCount > 0) cohesion = (cohesion / cohCount) - pos;

            // Convert rules into desired velocity contributions
            Vector3 desired = Vector3.zero;

            if (sepCount > 0)
            {
                Vector3 sepVel = separation.normalized * settings.maxSpeed;
                desired += sepVel * settings.separationWeight;
            }
            if (aliCount > 0)
            {
                Vector3 aliVel = alignment * settings.maxSpeed;
                desired += aliVel * settings.alignmentWeight;
            }
            if (cohCount > 0)
            {
                Vector3 cohDir = cohesion.normalized;
                Vector3 cohVel = cohDir * settings.maxSpeed;
                desired += cohVel * settings.cohesionWeight;
            }

            // Collision / obstacle avoidance (Seb-style: if about to hit something, pick best free direction)
            if (CheckCollisionAhead(pos, forward))
            {
                Vector3 avoidDir = FindBestAvoidDirection(pos);
                Vector3 avoidVel = avoidDir * settings.maxSpeed;
                desired += (avoidVel - me.velocity).normalized * settings.avoidCollisionWeight;
            }

            // If desired is zero, keep cruising forward
            if (desired.sqrMagnitude < 1e-6f)
                desired = forward * Mathf.Max(me.species.cruiseSpeed, settings.maxSpeed * 0.3f);

            return Vector3.ClampMagnitude(desired, settings.maxSpeed);
        }

        bool CheckCollisionAhead(Vector3 pos, Vector3 forward)
        {
            return Physics.SphereCast(pos, settings.obstacleCastRadius, forward, out _, settings.collisionAvoidDst, settings.obstacleMask, QueryTriggerInteraction.Ignore);
        }

        Vector3 FindBestAvoidDirection(Vector3 pos)
        {
            // Choose the first direction that’s clear; fall back to the least hit distance
            float bestDist = -1f;
            Vector3 bestDir = Vector3.zero;

            foreach (var dir in _viewDirs)
            {
                if (!Physics.SphereCast(pos, settings.obstacleCastRadius, dir, out RaycastHit hit, settings.collisionAvoidDst, settings.obstacleMask, QueryTriggerInteraction.Ignore))
                    return dir; // free path found

                if (hit.distance > bestDist)
                {
                    bestDist = hit.distance;
                    bestDir = dir;
                }
            }
            return (bestDir == Vector3.zero) ? Random.onUnitSphere : bestDir;
        }

        Vector3 BoundsSteer(Vector3 pos)
        {
            // Soft push away from WaterArea bounds within a margin like Seb’s boundsRadius
            var b = water.Bounds;
            float m = settings.boundsRadius;
            float k = settings.avoidCollisionWeight * 0.1f;

            Vector3 force = Vector3.zero;

            float dxMin = (pos.x - b.min.x);
            float dxMax = (b.max.x - pos.x);
            float dyMin = (pos.y - b.min.y);
            float dyMax = (b.max.y - pos.y);
            float dzMin = (pos.z - b.min.z);
            float dzMax = (b.max.z - pos.z);

            if (dxMin < m) force += Vector3.right * (1f - dxMin / m) * k;
            if (dxMax < m) force += Vector3.left * (1f - dxMax / m) * k;
            if (dyMin < m) force += Vector3.up * (1f - dyMin / m) * k;
            if (dyMax < m) force += Vector3.down * (1f - dyMax / m) * k;
            if (dzMin < m) force += Vector3.forward * (1f - dzMin / m) * k;
            if (dzMax < m) force += Vector3.back * (1f - dzMax / m) * k;

            return force;
        }

        void DiscoverAgents()
        {
            _agents.Clear();

            // Prefer the registry (fast) but also filter by this WaterArea
            foreach (var f in FishRegistry.All)
                if (f && f.enabled && f.water == water)
                    _agents.Add(f);

            _nextDiscoverTime = Time.time + rediscoverInterval;
        }
    }
}
