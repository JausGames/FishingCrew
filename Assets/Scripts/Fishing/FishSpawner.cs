using UnityEngine;

namespace Fishing
{
    public class FishSpawner : MonoBehaviour
    {
        public WaterArea water;
        public FishSpecies species;
        public int count = 12;

        void Start()
        {
            if (water == null || species == null) return;

            var b = water.Bounds;
            for (int i = 0; i < count; i++)
            {
                Vector3 p = new Vector3(
                    Random.Range(b.min.x, b.max.x),
                    Mathf.Lerp(water.surfaceY - species.preferredDepthRange.x,
                               water.surfaceY - species.preferredDepthRange.y, Random.value),
                    Random.Range(b.min.z, b.max.z));
                 
                Instantiate(species.prefab, p, Quaternion.LookRotation(Random.onUnitSphere.WithYScaled(0.2f).normalized, Vector3.up), transform)
                    .Init($"{species.name}_Fish_{i:D2}", water, false); 
            }
        }
    }

    static class VecExt
    {
        public static Vector3 WithYScaled(this Vector3 v, float scale) { v.y *= scale; return v; }
    }
}
