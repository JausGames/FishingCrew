using UnityEngine;
using System.Collections;

namespace Fishing
{
    [RequireComponent(typeof(CharacterController))]
    public class Fish : MonoBehaviour
    {
        public enum State { Cruise, Investigate }

        [Header("Setup")]
        public FishSpecies species;
        public WaterArea water;
        [Tooltip("If set, instantiates this species prefab as a child at runtime.")]
        public bool spawnVisualFromSpecies = true;

        [Header("Runtime (read-only)")]
        public State state = State.Cruise;
        public Vector3 velocity;         // world m/s
        public Vector3 forward = Vector3.forward;
        public float depth;              // meters below surface
        public Transform investigatingTarget;

        [Header("Debug")]
        public bool drawDebug = false;

        // Components
        private CharacterController _cc;

        // Wander
        private Vector3 _wanderTarget;
        private float _nextWanderTime;

        // Cached
        private float _turnRateRad => species != null ? species.turnRateDeg * Mathf.Deg2Rad : 0f;

        void Awake()
        {
            _cc = GetComponent<CharacterController>();

            // (From your version) Only handle visuals here.
            if (spawnVisualFromSpecies && species != null && species.prefab != null)
            {
                var vis = Instantiate(species.prefab, transform);
                vis.transform.localPosition = Vector3.zero;
                vis.transform.localRotation = Quaternion.identity;
                vis.name = species.name + "_Visual";
            }
        }

        void OnEnable()
        {
            // Join registry for boids neighbor queries
            if (!FishRegistry.All.Contains(this)) FishRegistry.All.Add(this);
        }

        void OnDisable()
        {
            FishRegistry.All.Remove(this);
        }

        void Update()
        {
            // Safety: allow your Init(...) to be called after spawn without null refs
            if (species == null || water == null) return;

            float dt = Mathf.Clamp(Time.deltaTime, 0f, 0.05f);

            depth = water.DepthBelowSurface(transform.position);

            // --- State logic ---
            switch (state)
            {
                case State.Cruise:
                    if (TryFindStimulus(out Transform t))
                    {
                        investigatingTarget = t;
                        state = State.Investigate;
                        StartCoroutine(InvestigateTimer(species.investigateTime));
                    }
                    break;

                case State.Investigate:
                    if (investigatingTarget == null) state = State.Cruise;
                    break;
            }

            // --- Steering composition ---
            Vector3 accel = Vector3.zero;

            // Mild wander to keep motion lively (low weight so boids dominate)
            if (Time.time >= _nextWanderTime) PickNewWanderTarget(false);
            accel += SeekOrArrive(_wanderTarget, arriveRadius: 2.0f, weight: 0.35f);

            // Investigate lure stronger than wander
            if (state == State.Investigate && investigatingTarget != null)
                accel += SeekOrArrive(investigatingTarget.position, arriveRadius: 1.2f, weight: 1.1f);

            // Boids: separation (ALL species), alignment+cohesion (SAME species)
            accel += BoidForces();

            // Depth band + predictive wall avoidance + obstacle avoidance
            accel += DepthBias(weight: species.depthBias);
            accel += PredictiveWallAvoidance();
            accel += AvoidObstacles();

            // Water flow (environmental velocity)
            Vector3 flow = water.SampleFlow(transform.position);

            // --- Integrate with limits ---
            Vector3 targetVel = velocity + accel * dt;
            float targetSpeed = Mathf.Clamp(targetVel.magnitude, 0f, species.maxSpeed);
            Vector3 targetDir = targetVel.sqrMagnitude > 1e-4f ? targetVel.normalized : forward;

            // Turn/accel caps
            targetDir = Vector3.RotateTowards(forward, targetDir, _turnRateRad * dt, 999f);
            float speed = Mathf.MoveTowards(velocity.magnitude, targetSpeed, species.acceleration * dt);

            forward = targetDir;
            velocity = forward * speed + flow;

            Vector3 delta = velocity * dt;

            // Surface clamp (avoid breaching)
            float minDepth = species.preferredDepthRange.x * 0.5f; // don't hug the surface
            Vector3 pos = transform.position + delta;
            pos.y = Mathf.Min(pos.y, water.surfaceY - minDepth);

            // Hard clamp inside bounds as a final safety net
            pos = water.ClampInside(pos, margin: 0.02f);

            // Kinematic "move"
            _cc.enabled = false; transform.position = pos; _cc.enabled = true;

            // Face movement direction
            if (forward.sqrMagnitude > 1e-4f)
            {
                Quaternion toRot = Quaternion.LookRotation(forward, Vector3.up);
                transform.rotation = Quaternion.Slerp(transform.rotation, toRot, 10f * dt);
            }
        }

        /// <summary>
        /// Your custom initializer preserved and used as requested.
        /// Call this right after spawning/configuring the Fish.
        /// </summary>
        internal void Init(string name, WaterArea water, bool drawDebug)
        {
            gameObject.name = name;
            this.water = water;
            this.drawDebug = drawDebug;

            if (species == null) { enabled = false; Debug.LogError("Fish: Species not set."); return; }
            if (water == null) { enabled = false; Debug.LogError("Fish: WaterArea not set."); return; }

            // Init direction slightly randomized
            forward = Random.onUnitSphere; forward.y *= 0.25f; forward.Normalize();
            velocity = forward * species.cruiseSpeed;
            PickNewWanderTarget(true);
        }

        // ---------- Boids ----------
        private Vector3 BoidForces()
        {
            Vector3 pos = transform.position;
            Vector3 sep = Vector3.zero;
            Vector3 ali = Vector3.zero;
            Vector3 coh = Vector3.zero;

            int sepCount = 0, aliCount = 0, cohCount = 0;

            float sepR2 = species.sepRadius * species.sepRadius;
            float aliR2 = species.alignRadius * species.alignRadius;
            float cohR2 = species.cohRadius * species.cohRadius;

            // Scan registered fish and filter to same WaterArea
            var list = FishRegistry.All;
            for (int i = 0; i < list.Count; i++)
            {
                Fish other = list[i];
                if (other == this) continue;
                if (other.water != this.water) continue;

                Vector3 to = other.transform.position - pos;
                float d2 = to.sqrMagnitude;
                if (d2 < 1e-6f) continue;

                // Separation from ANY species
                if (d2 <= sepR2)
                {
                    sep -= to / d2; // inverse-squared push away
                    sepCount++;
                }

                // Alignment + cohesion only with SAME species
                if (other.species == this.species)
                {
                    if (d2 <= aliR2) { ali += other.forward; aliCount++; }
                    if (d2 <= cohR2) { coh += other.transform.position; cohCount++; }
                }
            }

            if (sepCount > 0) sep /= sepCount;
            if (aliCount > 0) ali = (ali / aliCount).normalized;
            if (cohCount > 0) coh = ((coh / cohCount) - pos);

            Vector3 accel = Vector3.zero;

            if (sepCount > 0)
            {
                Vector3 desired = sep.normalized * species.maxSpeed;
                accel += (desired - velocity) * species.separationWeight;
            }

            if (aliCount > 0)
            {
                Vector3 desired = ali * species.cruiseSpeed;
                accel += (desired - velocity) * species.alignmentWeight;
            }

            if (cohCount > 0)
            {
                Vector3 dirToCenter = coh.normalized;
                Vector3 desired = dirToCenter * species.cruiseSpeed;
                accel += (desired - velocity) * species.cohesionWeight;
            }

            return accel;
        }

        // ---------- Steering terms ----------
        private Vector3 SeekOrArrive(Vector3 target, float arriveRadius, float weight)
        {
            Vector3 to = target - transform.position;
            float dist = to.magnitude;
            if (dist < 0.001f) return Vector3.zero;

            Vector3 desiredDir = to / dist;
            float desiredSpeed = dist > arriveRadius
                ? species.cruiseSpeed
                : Mathf.Lerp(0.25f * species.cruiseSpeed, species.cruiseSpeed, dist / arriveRadius);

            Vector3 desiredVel = desiredDir * desiredSpeed;
            return (desiredVel - velocity) * weight;
        }

        private Vector3 DepthBias(float weight)
        {
            float targetDepth = Mathf.Clamp(depth, species.preferredDepthRange.x, species.preferredDepthRange.y);
            float delta = targetDepth - depth; // positive if too shallow
            float verticalAccel = Mathf.Clamp(delta, -2f, 2f) * 2.0f * weight;
            return Vector3.up * (-verticalAccel); // depth increases downward
        }

        private Vector3 PredictiveWallAvoidance()
        {
            // Look ahead along forward, push away from nearby walls using normals
            Bounds b = water.Bounds;
            Vector3 p = transform.position;
            float lookAhead = Mathf.Max(velocity.magnitude, species.cruiseSpeed) * 0.6f;
            Vector3 ahead = p + forward * lookAhead;

            float m = species.wallMargin;
            float k = species.wallAvoidStrength * species.acceleration;

            Vector3 force = Vector3.zero;

            float dxMin = (ahead.x - b.min.x);
            float dxMax = (b.max.x - ahead.x);
            float dyMin = (ahead.y - b.min.y);
            float dyMax = (b.max.y - ahead.y);
            float dzMin = (ahead.z - b.min.z);
            float dzMax = (b.max.z - ahead.z);

            if (dxMin < m) force += Vector3.right * (1f - dxMin / m) * k;
            if (dxMax < m) force += Vector3.left * (1f - dxMax / m) * k;
            if (dyMin < m) force += Vector3.up * (1f - dyMin / m) * k;
            if (dyMax < m) force += Vector3.down * (1f - dyMax / m) * k;
            if (dzMin < m) force += Vector3.forward * (1f - dzMin / m) * k;
            if (dzMax < m) force += Vector3.back * (1f - dzMax / m) * k;

            return force;
        }

        private Vector3 AvoidObstacles()
        {
            float probe = species.obstacleProbeDist;
            float radius = species.obstacleRadius;
            Vector3 origin = transform.position;

            if (Physics.SphereCast(origin, radius, forward, out RaycastHit hit, probe, ~0, QueryTriggerInteraction.Ignore))
            {
                Vector3 away = Vector3.ProjectOnPlane((origin - hit.point).normalized, hit.normal);
                Vector3 steer = (hit.normal * 2.0f + away).normalized * species.acceleration * 1.5f;
                return steer;
            }
            return Vector3.zero;
        }

        // ---------- Helpers ----------
        private void PickNewWanderTarget(bool immediate)
        {
            _nextWanderTime = Time.time + (immediate ? 0.01f : species.wanderInterval);

            // Jittered sphere around a forward anchor
            Vector3 anchor = transform.position + forward * species.wanderRadius;
            Vector3 jitter = Random.insideUnitSphere * species.wanderJitter;
            Vector3 candidate = anchor + jitter;

            // Clamp inside and bias to preferred depth band
            candidate = water.ClampInside(candidate, 0.2f);
            float preferredY = water.surfaceY - Mathf.Clamp(
                (species.preferredDepthRange.x + species.preferredDepthRange.y) * 0.5f,
                species.preferredDepthRange.x, species.preferredDepthRange.y);
            candidate.y = Mathf.Clamp(candidate.y, preferredY - 2f, preferredY + 2f);

            _wanderTarget = candidate;
        }

        private bool TryFindStimulus(out Transform t)
        {
            t = null;
            if (LureStimulus.Active.Count == 0) return false;

            float bestScore = 0f;
            Transform best = null;

            foreach (var s in LureStimulus.Active)
            {
                Vector3 to = s.transform.position - transform.position;
                float dist = to.magnitude;
                if (dist > species.visionRange) continue;

                float angle = Vector3.Angle(forward, to);
                if (angle > species.visionFOV * 0.5f) continue;

                float strength = s.StrengthAt(transform.position);
                float score = strength * Mathf.Lerp(0.4f, 1.0f, species.curiosity);
                if (score > bestScore) { bestScore = score; best = s.transform; }
            }

            t = best;
            return best != null;
        }

        private IEnumerator InvestigateTimer(float seconds)
        {
            float t = 0f;
            while (state == State.Investigate && t < seconds)
            {
                t += Time.deltaTime;
                yield return null;
            }
            state = State.Cruise;
            investigatingTarget = null;
        }

#if UNITY_EDITOR
        void OnDrawGizmosSelected()
        {
            if (!drawDebug || species == null) return;

            // Boids radii
            Gizmos.color = new Color(1f, 0.3f, 0.3f, 0.35f); Gizmos.DrawWireSphere(transform.position, species.sepRadius);
            Gizmos.color = new Color(0.3f, 1f, 0.3f, 0.35f); Gizmos.DrawWireSphere(transform.position, species.alignRadius);
            Gizmos.color = new Color(0.3f, 0.3f, 1f, 0.35f); Gizmos.DrawWireSphere(transform.position, species.cohRadius);

            // Forward
            Gizmos.color = Color.green; Gizmos.DrawRay(transform.position, forward * 1.0f);

            // Wander target
            Gizmos.color = Color.cyan; Gizmos.DrawSphere(_wanderTarget, 0.1f);
        }
#endif
    }
}
