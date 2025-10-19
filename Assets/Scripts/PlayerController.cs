using UnityEngine;

/// <summary>
/// Simple but versatile FPS motor with slope-aware movement, step handling, jump buffer + coyote time,
/// ground snapping, controlled sliding when above slope limit, and optional moving-platform compensation.
/// </summary>
[RequireComponent(typeof(CharacterController))]
[RequireComponent(typeof(PlayerInputs))]
public class PlayerController : MonoBehaviour
{
    [Header("References")]
    [Tooltip("Optional: Parent the Camera under this transform (head) and rotate yaw on this object, pitch on the head/camera.")]
    public Transform cameraRoot; // not used for movement; reserved for your look script
    public LayerMask groundMask = ~0; // what counts as ground

    [Header("Movement Speeds")]
    public float walkSpeed = 4.5f;
    public float sprintSpeed = 6.5f;
    public float airMaxSpeed = 6.5f;

    [Header("Acceleration / Damping")]
    [Tooltip("How fast we accelerate on ground towards desired speed.")]
    public float groundAcceleration = 55f;
    [Tooltip("How fast we decelerate on ground when there's little/no input.")]
    public float groundDeceleration = 40f;
    [Tooltip("How much control you have in air.")]
    public float airAcceleration = 12f;
    [Tooltip("Extra drag applied in air (helps stabilize).")]
    public float airDrag = 0.01f;

    [Header("Gravity & Jump")]
    public float gravity = 20f;
    public float jumpHeight = 1.2f;
    [Tooltip("Short-hop: if jump is released early, apply this factor to vertical velocity.")]
    public float jumpReleaseCut = 0.5f;
    [Tooltip("Coyote time in seconds (jump just after leaving ground).")]
    public float coyoteTime = 0.12f;
    [Tooltip("Jump buffer window in seconds (press jump just before landing).")]
    public float jumpBuffer = 0.12f;
    [Tooltip("Clamp downward speed to avoid numerical explosions.")]
    public float terminalVelocity = 50f;

    [Header("Slope & Grounding")]
    [Tooltip("Max walkable slope (deg). Above this, we slide.")]
    [Range(1f, 89f)] public float maxSlopeAngle = 50f;
    [Tooltip("How strongly we stick to ground. Helps avoid micro-hops on small bumps.")]
    public float groundSnapDistance = 0.25f;
    [Tooltip("Sphere radius used to probe ground beneath the capsule.")]
    public float groundProbeRadius = 0.25f;
    [Tooltip("Extra inset so our probe starts inside the capsule bottom.")]
    public float groundProbeInset = 0.02f;
    [Tooltip("Sliding acceleration along too-steep slopes.")]
    public float slideAcceleration = 15f;
    [Tooltip("Friction while sliding (0 = icy, 1+ = very sticky).")]
    [Range(0f, 2f)] public float slideFriction = 0.2f;

    [Header("Steps & Edges")]
    [Tooltip("Max height we can step up without jumping.")]
    public float maxStepHeight = 0.4f;

    [Header("Platforms (Optional)")]
    [Tooltip("Inherit velocity from moving ground (like boats/platforms).")]
    public bool inheritPlatformMotion = true;
    [Tooltip("How quickly to blend into platform velocity when you first step on.")]
    [Range(0f, 1f)] public float platformBlend = 0.25f;

    [Header("Debug")]
    public bool drawDebug = false;

    // Components
    private CharacterController _cc;
    private PlayerInputs _inputs;

    // State
    public bool IsGrounded { get; private set; }
    public bool IsOnWalkableSlope { get; private set; }
    public float CurrentSlopeAngle { get; private set; }
    public Vector3 GroundNormal { get; private set; } = Vector3.up;

    private Vector3 _velocity;          // world-space motor velocity
    private Vector3 _desiredPlanar;     // desired planar (x/z) velocity this frame
    private Vector3 _platformVelocity;  // ground/platform world velocity
    private Transform _groundTransform; // moving ground reference
    private Vector3 _lastGroundPos;
    private Quaternion _lastGroundRot;

    // Timers for jump logic
    private float _timeSinceUngrounded = Mathf.Infinity;
    private float _timeSinceJumpPressed = Mathf.Infinity;
    private bool _jumpHeld;

    private float _controllerBaseStepOffset;

    void Awake()
    {
        _cc = GetComponent<CharacterController>();
        _inputs = GetComponent<PlayerInputs>();

        _controllerBaseStepOffset = _cc.stepOffset;
        _cc.slopeLimit = maxSlopeAngle; // keep CC's limit in sync (we still do our own checks)
    }

    void OnValidate()
    {
        if (_cc == null) _cc = GetComponent<CharacterController>();
        if (_cc != null)
        {
            _cc.slopeLimit = maxSlopeAngle;
        }
        walkSpeed = Mathf.Max(0.1f, walkSpeed);
        sprintSpeed = Mathf.Max(walkSpeed, sprintSpeed);
        airMaxSpeed = Mathf.Max(walkSpeed, airMaxSpeed);
        maxStepHeight = Mathf.Max(0f, maxStepHeight);
    }

    void Update()
    {
        // Read jump press/release timing in Update for better responsiveness.
        if (_inputs.Jump)
        {
            _timeSinceJumpPressed = 0f;
            _jumpHeld = true;
        }
        else
        {
            _jumpHeld = false;
        }

        // We run motor in Update so CharacterController.Move uses frame dt (smoother cameras).
        MotorUpdate(Time.deltaTime);
    }

    private void MotorUpdate(float dt)
    {
        // 1) Ground check + platform velocity
        UpdateGrounding(dt);

        // 2) Build desired planar velocity from input
        Vector2 move = _inputs.Move; // [-1..1] in x/y (y = forward)
        Vector3 inputDir = Vector3.ClampMagnitude(new Vector3(move.x, 0f, move.y), 1f);

        // Transform input from local to world (assumes this object yaw = facing)
        Vector3 worldInput = transform.TransformDirection(inputDir);

        float targetSpeed = walkSpeed;
        // Optional sprint (see note at bottom if you add it to PlayerInputs)
        var hasSprintProp = false; // compile-time: we won't require it
        // If you add PlayerInputs.Sprint => uncomment:
        // targetSpeed = _inputs.Sprint ? sprintSpeed : walkSpeed;

        _desiredPlanar = worldInput * targetSpeed;

        // 3) If grounded and on walkable slope → project onto ground plane for crisp sticking
        Vector3 planarVel = new Vector3(_velocity.x, 0f, _velocity.z);

        if (IsGrounded && IsOnWalkableSlope)
        {
            Vector3 desiredOnPlane = Vector3.ProjectOnPlane(_desiredPlanar, GroundNormal);

            // Choose accel or decel depending on input magnitude vs current planar speed
            float accel = (_desiredPlanar.sqrMagnitude > 0.01f) ? groundAcceleration : groundDeceleration;

            planarVel = Accelerate(planarVel, desiredOnPlane, accel, dt);

            // Optional: slight downhill assist (feel free to comment out)
            planarVel = Vector3.ClampMagnitude(planarVel, targetSpeed + 0.25f);

            // Vertical stays controlled by gravity/jumps below, but we help stick to ground a bit
            if (_velocity.y <= 0f)
            {
                _velocity.y = -Mathf.Max(2f, gravity * 0.25f) * dt; // tiny downward bias to stay grounded
            }
        }
        else
        {
            // In air: accelerate toward desired direction with a max air speed
            Vector3 desiredFlat = _desiredPlanar;
            planarVel = AirAccelerate(planarVel, desiredFlat, airAcceleration, dt, airMaxSpeed);
            // Lightweight air drag
            planarVel *= (1f - airDrag * dt);
        }

        // 4) Too-steep slope sliding
        if (IsGrounded && !IsOnWalkableSlope)
        {
            // Slide direction is the ground tangent (gravity projected onto plane)
            Vector3 slideDir = Vector3.ProjectOnPlane(Vector3.down, GroundNormal).normalized;
            // Accelerate along slide
            planarVel += slideDir * (slideAcceleration * dt);
            // Apply slide friction
            planarVel *= (1f - Mathf.Clamp01(slideFriction) * dt);
        }

        // 5) Compose velocity (planar + vertical)
        _velocity.x = planarVel.x;
        _velocity.z = planarVel.z;

        // Gravity & jump handling (buffer + coyote)
        HandleJumpAndGravity(dt);

        // 6) Add platform velocity if any (last)
        Vector3 frameMotion = _velocity * dt;
        if (inheritPlatformMotion && IsGrounded && _platformVelocity != Vector3.zero)
        {
            frameMotion += _platformVelocity * dt * platformBlend;
        }

        // Respect stepOffset only when grounded & moving up small ledges
        _cc.stepOffset = (IsGrounded && IsOnWalkableSlope) ? Mathf.Min(_controllerBaseStepOffset, maxStepHeight) : 0f;

        // 7) Move character
        var ccMoveFlags = _cc.Move(frameMotion);

        // If CC reports ground, we consider grounded next frame (but we still trust our probe)
        // No-op here; our probe is authoritative.

        // Timers
        _timeSinceJumpPressed += dt;
        if (!IsGrounded) _timeSinceUngrounded += dt;

        // Clamp terminal
        _velocity.y = Mathf.Max(_velocity.y, -terminalVelocity);
    }

    private void HandleJumpAndGravity(float dt)
    {
        // Calculate jump impulse based on height
        float jumpVel = Mathf.Sqrt(2f * gravity * Mathf.Max(0.01f, jumpHeight));

        bool canCoyote = _timeSinceUngrounded <= coyoteTime;
        bool jumpBuffered = _timeSinceJumpPressed <= jumpBuffer;

        if (jumpBuffered && (IsGrounded || canCoyote) && _velocity.y <= 0.5f)
        {
            _velocity.y = jumpVel;
            _timeSinceJumpPressed = Mathf.Infinity; // consume buffer
            IsGrounded = false; // will become airborne this frame
        }
        else
        {
            // Apply gravity
            _velocity.y -= gravity * dt;

            // Early jump release cut (only while moving upward)
            if (!_jumpHeld && _velocity.y > 0f)
            {
                _velocity.y -= gravity * (1f - jumpReleaseCut) * dt;
            }
        }
    }

    private void UpdateGrounding(float dt)
    {
        // Default to air
        bool wasGrounded = IsGrounded;
        IsGrounded = false;
        IsOnWalkableSlope = false;
        CurrentSlopeAngle = 0f;
        GroundNormal = Vector3.up;
        _platformVelocity = Vector3.zero;
        _groundTransform = null;

        // Compute where to start the probe: a bit above feet, then cast down
        Vector3 foot = transform.position + Vector3.up * (_cc.radius + groundProbeInset);
        float castDist = groundSnapDistance + maxStepHeight + 0.05f;

        if (Physics.SphereCast(foot, groundProbeRadius, Vector3.down, out RaycastHit hit, castDist, groundMask, QueryTriggerInteraction.Ignore))
        {
            GroundNormal = hit.normal;
            CurrentSlopeAngle = Vector3.Angle(GroundNormal, Vector3.up);

            // Consider grounded if close enough and moving down or small upward move
            float feetToGround = hit.distance - groundProbeInset;
            bool closeToGround = feetToGround <= groundSnapDistance + maxStepHeight;
            bool movingDown = _velocity.y <= 0.1f;

            if (closeToGround && movingDown)
            {
                IsGrounded = true;
                IsOnWalkableSlope = CurrentSlopeAngle <= maxSlopeAngle + 0.01f;
                _timeSinceUngrounded = 0f;

                // Moving platform support
                _groundTransform = hit.rigidbody ? hit.rigidbody.transform : hit.collider.transform;

                if (inheritPlatformMotion && _groundTransform != null)
                {
                    // Estimate ground velocity from delta transform (pos & rot)
                    Vector3 platformVel = Vector3.zero;

                    if (_lastGroundPos != Vector3.zero)
                    {
                        // Positional velocity
                        platformVel = (_groundTransform.position - _lastGroundPos) / Mathf.Max(dt, 1e-5f);

                        // Rotational velocity contribution (approximate around ground pivot)
                        Quaternion deltaRot = _groundTransform.rotation * Quaternion.Inverse(_lastGroundRot);
                        Vector3 localFromPivot = transform.position - _groundTransform.position;
                        Vector3 rotated = deltaRot * localFromPivot;
                        Vector3 rotContribution = (rotated - localFromPivot) / Mathf.Max(dt, 1e-5f);
                        platformVel += rotContribution;
                    }

                    _platformVelocity = platformVel;
                    _lastGroundPos = _groundTransform.position;
                    _lastGroundRot = _groundTransform.rotation;
                }
                else
                {
                    _lastGroundPos = Vector3.zero;
                }

                // Ground snapping (pull to ground a bit to avoid hover)
                if (feetToGround > 0.01f)
                {
                    // Only snap if the space is clear (avoid penetrating steep edges)
                    Vector3 snap = Vector3.down * Mathf.Min(feetToGround, groundSnapDistance);
                    _cc.Move(snap);
                }
            }
        }

        if (!IsGrounded)
        {
            _lastGroundPos = Vector3.zero;
        }
    }

    private static Vector3 Accelerate(Vector3 current, Vector3 desired, float accel, float dt)
    {
        // Move current vector toward desired by accel per second, capped by desired magnitude
        Vector3 delta = desired - current;
        Vector3 step = Vector3.ClampMagnitude(delta, accel * dt);
        return current + step;
    }

    private static Vector3 AirAccelerate(Vector3 current, Vector3 desired, float accel, float dt, float maxAirSpeed)
    {
        if (desired.sqrMagnitude < 1e-6f) return current;

        Vector3 desiredDir = desired.normalized;
        float currentAlong = Vector3.Dot(current, desiredDir);
        float add = accel * dt;

        // Increase along desired direction up to maxAirSpeed
        float targetAlong = Mathf.Min(currentAlong + add, maxAirSpeed);
        Vector3 along = desiredDir * targetAlong;

        // Preserve sideways component (no air-braking unless desired says so)
        Vector3 sideways = current - desiredDir * currentAlong;
        return along + sideways;
    }

    void OnDrawGizmosSelected()
    {
        if (!drawDebug) return;

        // Ground probe gizmos
        Gizmos.color = Color.yellow;
        Vector3 foot = transform.position + Vector3.up * ((_cc ? _cc.radius : 0.25f) + groundProbeInset);
        Gizmos.DrawWireSphere(foot + Vector3.down * groundSnapDistance, groundProbeRadius);

        // Normal & slope
        Gizmos.color = Color.cyan;
        Gizmos.DrawRay(transform.position, GroundNormal);
        UnityEditor.Handles.Label(transform.position + Vector3.up * 1.8f,
            $"Grounded: {IsGrounded}\nWalkable: {IsOnWalkableSlope}\nSlope: {CurrentSlopeAngle:0.0}°");
    }
}
