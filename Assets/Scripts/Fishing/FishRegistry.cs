using System.Collections.Generic;

namespace Fishing
{
    /// <summary>
    /// Lightweight registry so fish can find nearby neighbors.
    /// Replace with a spatial grid/quad-tree later if you scale to thousands.
    /// </summary>
    public static class FishRegistry
    {
        public static readonly List<Fish> All = new List<Fish>();
    }
}
