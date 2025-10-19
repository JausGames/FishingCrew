using UnityEngine;
using System.Collections.Generic;

namespace Fishing
{
    public class LureStimulus : MonoBehaviour
    {
        public static readonly List<LureStimulus> Active = new List<LureStimulus>();

        [Tooltip("How strong the signal is at 1m (decays with distance).")]
        public float baseStrength = 1f;

        void OnEnable() { Active.Add(this); }
        void OnDisable() { Active.Remove(this); }

        public float StrengthAt(Vector3 pos)
        {
            float d = Vector3.Distance(pos, transform.position);
            if (d < 0.001f) return baseStrength;
            return baseStrength / (d * d + 1f); // inverse-square-ish
        }
    }
}
