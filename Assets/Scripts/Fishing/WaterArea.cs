using UnityEngine;

namespace Fishing
{
    [ExecuteAlways]
    [RequireComponent(typeof(BoxCollider))]
    public class WaterArea : MonoBehaviour
    {
        public enum FlowType { Uniform, Perlin3D }

        [Header("Surface")]
        [Tooltip("Y world position of water surface (flat).")]
        public float surfaceY = 0f;

        [Header("Flow")]
        public FlowType flowType = FlowType.Uniform;
        public Vector3 uniformFlow = new Vector3(0.5f, 0f, 0f); // m/s
        [Tooltip("Perlin noise scale (lower = larger features).")]
        public float perlinScale = 0.08f;
        public float perlinStrength = 0.8f;
        public float perlinTimeScale = 0.2f;

        [Header("Debug")]
        public bool drawBounds = true;
        public bool drawFlow = false;

        private BoxCollider _box;

        void OnEnable() { _box = GetComponent<BoxCollider>(); }
        public Bounds Bounds => _box ? _box.bounds : new Bounds(transform.position, Vector3.one);

        public Vector3 SampleFlow(Vector3 worldPos)
        {
            if (flowType == FlowType.Uniform) return uniformFlow;

            // Simple 3D-ish perlin: sample 3 axes with offsets
            Vector3 p = worldPos * perlinScale + Vector3.one * (Time.time * perlinTimeScale);
            float nx = Mathf.PerlinNoise(p.y, p.z) * 2f - 1f;
            float ny = Mathf.PerlinNoise(p.z, p.x) * 2f - 1f;
            float nz = Mathf.PerlinNoise(p.x, p.y) * 2f - 1f;
            Vector3 v = new Vector3(nx, ny * 0.15f, nz); // mostly horizontal
            return v.normalized * perlinStrength;
        }

        public Vector3 ClampInside(Vector3 pos, float margin = 0.25f)
        {
            var b = Bounds;
            Vector3 min = b.min + Vector3.one * margin;
            Vector3 max = b.max - Vector3.one * margin;
            return new Vector3(Mathf.Clamp(pos.x, min.x, max.x),
                               Mathf.Clamp(pos.y, min.y, max.y),
                               Mathf.Clamp(pos.z, min.z, max.z));
        }

        public float DepthBelowSurface(Vector3 pos) => Mathf.Max(0f, surfaceY - pos.y);

        void OnDrawGizmos()
        {
            if (!drawBounds || _box == null) return;
            Gizmos.color = new Color(0f, 0.5f, 1f, 0.15f);
            Gizmos.matrix = Matrix4x4.identity;
            Gizmos.DrawCube(_box.bounds.center, _box.bounds.size);

            Gizmos.color = new Color(0f, 0.5f, 1f, 0.6f);
            Gizmos.DrawLine(new Vector3(_box.bounds.min.x, surfaceY, _box.bounds.min.z),
                            new Vector3(_box.bounds.max.x, surfaceY, _box.bounds.max.z));
        }

#if UNITY_EDITOR
        void OnDrawGizmosSelected()
        {
            if (!drawFlow) return;
            var b = Bounds;
            Gizmos.color = Color.cyan;
            int grid = 6;
            for (int x = 0; x <= grid; x++)
                for (int y = 0; y <= grid; y++)
                    for (int z = 0; z <= grid; z++)
                    {
                        Vector3 p = new Vector3(
                            Mathf.Lerp(b.min.x, b.max.x, x / (float)grid),
                            Mathf.Lerp(b.min.y, b.max.y, y / (float)grid),
                            Mathf.Lerp(b.min.z, b.max.z, z / (float)grid));
                        Vector3 v = SampleFlow(p);
                        Gizmos.DrawRay(p, v * 0.3f);
                    }
        }
#endif
    }
}
