using UnityEngine;

namespace Fishing
{
    public static class BoidHelper
    {
        // Returns a stable array of directions with roughly even distribution on the unit sphere.
        public static Vector3[] GetViewDirections(int count)
        {
            count = Mathf.Clamp(count, 6, 256);
            Vector3[] dirs = new Vector3[count];
            const float PHI = 1.61803398875f; // golden ratio
            for (int i = 0; i < count; i++)
            {
                float t = (i + 0.5f) / count;
                float theta = 2f * Mathf.PI * t * PHI;
                float z = 1f - 2f * t;
                float r = Mathf.Sqrt(1f - z * z);
                float x = r * Mathf.Cos(theta);
                float y = r * Mathf.Sin(theta);
                dirs[i] = new Vector3(x, z, y); // swap around to get a pleasant spread
            }
            return dirs;
        }
    }
}
