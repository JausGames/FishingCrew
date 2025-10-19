using UnityEngine;

/// <summary>
/// FPS look controller: yaw on this transform, pitch on cameraRoot.
/// Works with PlayerInputs.Look (mouse delta or gamepad stick).
/// </summary>
[RequireComponent(typeof(PlayerInputs))]
public class LookController : MonoBehaviour
{
    [Header("References")]
    public Transform cameraRoot;                // Pitch happens here (child of player)
    public Camera fpsCamera;                    // Optional; used for FOV tweaks later

    [Header("Sensitivity")]
    [Tooltip("Mouse delta sensitivity (degrees per pixel).")]
    public float mouseSensitivityX = 0.12f;
    public float mouseSensitivityY = 0.12f;
    [Tooltip("Gamepad stick sensitivity (degrees per second at full deflection).")]
    public float stickSensitivityX = 200f;
    public float stickSensitivityY = 200f;

    [Header("Curves & Deadzones (Stick Only)")]
    [Tooltip("Stick response curve (x^n). 1 = linear, 2 = gentle near center.")]
    [Range(1f, 3f)] public float stickExponent = 1.6f;
    [Tooltip("Ignore tiny stick noise.")]
    [Range(0f, 0.3f)] public float stickDeadzone = 0.1f;

    [Header("Smoothing")]
    [Tooltip("Enable critically-damped smoothing (good for controllers).")]
    public bool smoothLook = true;
    [Tooltip("Lower is snappier. 0.03–0.08 typical for mouse; 0.08–0.15 for controller.")]
    [Range(0.0f, 0.2f)] public float smoothingTime = 0.06f;

    [Header("Limits")]
    [Tooltip("Clamp vertical look.")]
    [Range(1f, 89f)] public float maxPitch = 85f;
    public bool invertY = false;

    [Header("Cursor")]
    public bool lockCursor = true;
    public KeyCode toggleCursorKey = KeyCode.Escape;

    // Runtime
    private PlayerInputs _inputs;
    private float _yaw;          // world yaw (around up)
    private float _pitch;        // local pitch (-down, +up)
    private float _yawVel;       // for SmoothDampAngle
    private float _pitchVel;

    // Recoil (adds to pitch/yaw briefly)
    private float _recoilYaw, _recoilPitch;
    private float _recoilYawVel, _recoilPitchVel;

    void Awake()
    {
        _inputs = GetComponent<PlayerInputs>();
        if (cameraRoot == null)
        {
            Debug.LogError($"{nameof(LookController)}: cameraRoot not set.");
            enabled = false;
            return;
        }

        // Initialize from current transforms
        _yaw = transform.eulerAngles.y;
        _pitch = cameraRoot.localEulerAngles.x;
        if (_pitch > 180f) _pitch -= 360f; // map to [-180,180]
    }

    void OnEnable()
    {
        if (lockCursor)
        {
            Cursor.lockState = CursorLockMode.Locked;
            Cursor.visible = false;
        }
    }

    void Update()
    {
        // Allow cursor toggle
        if (Input.GetKeyDown(toggleCursorKey))
        {
            lockCursor = !lockCursor;
            Cursor.lockState = lockCursor ? CursorLockMode.Locked : CursorLockMode.None;
            Cursor.visible = !lockCursor;
        }

        float dt = Time.unscaledDeltaTime; // keep look responsive even if game slow-mo

        // Read input (your PlayerInputs.Look can be mouse delta or stick)
        Vector2 look = _inputs.Look;

        // Heuristic: if magnitude > 2.5 in a single frame, likely mouse delta (pixels);
        // if <= 1, likely stick (normalized). You can expose a toggle if you prefer.
        bool likelyStick = look.sqrMagnitude <= 1.2f * 1.2f;

        float dYaw, dPitch;

        if (likelyStick)
        {
            // Deadzone + curve
            Vector2 v = ApplyDeadzone(look, stickDeadzone);
            v = PowSign(v, stickExponent);

            dYaw = v.x * stickSensitivityX * dt;
            dPitch = v.y * stickSensitivityY * dt * (invertY ? 1f : -1f);
        }
        else
        {
            // Mouse delta in pixels → degrees
            dYaw = look.x * mouseSensitivityX;
            dPitch = look.y * mouseSensitivityY * (invertY ? 1f : -1f);
        }

        float targetYaw = _yaw + dYaw + _recoilYaw;
        float targetPitch = Mathf.Clamp(_pitch + dPitch + _recoilPitch, -maxPitch, maxPitch);

        if (smoothLook)
        {
            _yaw = Mathf.SmoothDampAngle(_yaw, targetYaw, ref _yawVel, smoothingTime, Mathf.Infinity, dt);
            _pitch = SmoothDampAngleClamped(_pitch, targetPitch, ref _pitchVel, smoothingTime, -maxPitch, maxPitch, dt);
        }
        else
        {
            _yaw = targetYaw;
            _pitch = targetPitch;
        }

        // Apply rotations
        transform.rotation = Quaternion.Euler(0f, _yaw, 0f);
        cameraRoot.localRotation = Quaternion.Euler(_pitch, 0f, 0f);

        // Recoil decay (critically damped)
        _recoilYaw = Mathf.SmoothDamp(_recoilYaw, 0f, ref _recoilYawVel, 0.08f, Mathf.Infinity, dt);
        _recoilPitch = Mathf.SmoothDamp(_recoilPitch, 0f, ref _recoilPitchVel, 0.08f, Mathf.Infinity, dt);
    }

    // Public API: call from weapons/tools to kick the view
    public void AddRecoil(float pitchDegreesUp, float yawDegrees, float recoverTime = 0.08f)
    {
        _recoilPitch -= pitchDegreesUp; // negative (pitch up) because -pitch looks up in our convention
        _recoilYaw += yawDegrees;

        // Optional: scale the smoothing toward recoverTime if you want
        // (kept simple here)
    }

    // ---------- helpers ----------
    private static Vector2 ApplyDeadzone(Vector2 v, float dz)
    {
        float m = v.magnitude;
        if (m <= dz) return Vector2.zero;
        float t = (m - dz) / (1f - dz);
        return v * (t / m);
    }
    private static Vector2 PowSign(Vector2 v, float p)
    {
        return new Vector2(Mathf.Sign(v.x) * Mathf.Pow(Mathf.Abs(v.x), p),
                           Mathf.Sign(v.y) * Mathf.Pow(Mathf.Abs(v.y), p));
    }

    private static float SmoothDampAngleClamped(float current, float target, ref float currentVel,
                                                float smoothTime, float min, float max, float dt)
    {
        target = Mathf.Clamp(target, min, max);
        return Mathf.SmoothDampAngle(current, target, ref currentVel, smoothTime, Mathf.Infinity, dt);
    }
}
