using UnityEngine;

/// <summary>
/// Represents a rectangular avoidance zone in world space.
/// The zone is axis-aligned and evaluated in the horizontal (XZ) plane.
/// </summary>
[System.Serializable]
public struct AvoidanceZone
{
    public Vector3 center;
    public float width;
    public float depth;

    /// <summary>
    /// Determines whether a given point lies inside the avoidance zone.
    /// The check is performed in the XZ plane, ignoring height (Y axis).
    /// </summary>
    /// <param name="point">The world-space point to test.</param>
    /// <returns>
    /// True if the point is inside the avoidance zone, false otherwise.
    /// </returns>
    public bool Contains(Vector3 point)
    {
        return Mathf.Abs(point.x - center.x) <= width * 0.5f &&
                Mathf.Abs(point.z - center.z) <= depth * 0.5f; 
    }
}