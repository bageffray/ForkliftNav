using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;

/// <summary>
/// Utility class used to validate linear or spline-based movements
/// against terrain slope constraints and avoidance zones.
/// </summary>
public static class PathValidator
{
    /// <summary>
    /// Checks whether a straight-line move between two 2D points is valid.
    /// The path is discretized into small steps and tested against:
    /// - avoidance zones
    /// - terrain slope constraints
    /// </summary>
    /// <param name="from">Start position (XZ plane)</param>
    /// <param name="to">End position (XZ plane)</param>
    /// <param name="checkStepInterval">Distance between sampled points</param>
    /// <param name="avoidanceZones">Forbidden rectangular areas</param>
    /// <param name="maxSlopeAngle">Maximum allowed slope angle (degrees)</param>
    /// <param name="minSlopeAngle">Minimum allowed slope angle (degrees)</param>
    /// <param name="carWidth">Width of the car</param>
    /// <param name="widthMarginFraction">Additional margin fraction to add to car width for clearance checks</param>
    public static bool IsMoveValid(Vector2 from, Vector2 to, float checkStepInterval, List<AvoidanceZone> avoidanceZones, float maxSlopeAngle, float minSlopeAngle, float carWidth, float widthMarginFraction)
    {

        if (Vector2.Distance(from, to) == 0f)
        {
            return true; // No movement, so possible
        }

        Vector3 forward = (to - from).normalized;

        List<Vector3> samplePoints = new List<Vector3>();
        // Start position projected onto terrain 
        Vector3 startPoint = new Vector3(from.x, 0f, from.y);
        startPoint.y = Terrain.activeTerrain.SampleHeight(startPoint);
        samplePoints.Add(startPoint);

        // Sampling points along the path
        while (Vector2.Distance(from, to) > 0f)
        {
            Vector2 direction = (to - from).normalized;
            Vector2 nextPoint = from + direction * checkStepInterval;
            if (Vector2.Distance(from, to) <= checkStepInterval)
            {
                nextPoint = to;
            }

            // Convert 2D path point to world space and sample terrain height
            Vector3 point = new Vector3(nextPoint.x, 0f, nextPoint.y);
            point.y = Terrain.activeTerrain.SampleHeight(point);

            samplePoints.Add(point);
            from = nextPoint;
        }
    
        // Validate each sampled point
        for (int i = 0; i < samplePoints.Count; i++)
        {
            Vector3 p = samplePoints[i];

            Vector3 leftDir = Vector3.Cross(forward, Vector3.up);

            // left, right, front and back check points to account for car width
            Vector3 leftCheckPoint = p + leftDir * ((carWidth * (1+widthMarginFraction)) / 2f); // Increase width by widthMarginFraction to be safe
            leftCheckPoint.y = Terrain.activeTerrain.SampleHeight(leftCheckPoint);
            Vector3 rightCheckPoint = p - leftDir * ((carWidth * (1+widthMarginFraction)) / 2f);
            rightCheckPoint.y = Terrain.activeTerrain.SampleHeight(rightCheckPoint); 
            Vector3 frontCheckPoint = p + forward * ((carWidth * (1+widthMarginFraction)) / 2f);
            frontCheckPoint.y = Terrain.activeTerrain.SampleHeight(frontCheckPoint);
            Vector3 backCheckPoint = p - forward * ((carWidth * (1+widthMarginFraction)) / 2f);
            backCheckPoint.y = Terrain.activeTerrain.SampleHeight(backCheckPoint);
            
            // Check if the point is inside any avoidance zone
            foreach (AvoidanceZone zone in avoidanceZones)
            {
                if (zone.Contains(p) || zone.Contains(leftCheckPoint) || zone.Contains(rightCheckPoint) || zone.Contains(frontCheckPoint) || zone.Contains(backCheckPoint))
                {
                    return false;
                }
            }
            
            // Check slope angle between this point and the next

            // TODO add check and right slope angle verification
            if (i < samplePoints.Count-1)
            {
                Vector3 nextPoint = samplePoints[i+1];

                float floorDistance = Mathf.Sqrt(Mathf.Pow(nextPoint.x - p.x, 2) + Mathf.Pow(nextPoint.z - p.z, 2));

                Vector3 nextLeftCheckPoint = nextPoint + leftDir * ((carWidth * (1+widthMarginFraction)) / 2f);
                nextLeftCheckPoint.y = Terrain.activeTerrain.SampleHeight(nextLeftCheckPoint);
                Vector3 nextRightCheckPoint = nextPoint - leftDir * ((carWidth * (1+widthMarginFraction)) / 2f);
                nextRightCheckPoint.y = Terrain.activeTerrain.SampleHeight(nextRightCheckPoint);

                float heightDifference = Mathf.Abs(nextPoint.y - p.y);
                float leftHeightDifference = Mathf.Abs(nextLeftCheckPoint.y - leftCheckPoint.y);
                float rightHeightDifference = Mathf.Abs(nextRightCheckPoint.y - rightCheckPoint.y);

                float angle = Mathf.Atan2(heightDifference, floorDistance) * Mathf.Rad2Deg;
                float leftAngle = Mathf.Atan2(leftHeightDifference, floorDistance) * Mathf.Rad2Deg;
                float rightAngle = Mathf.Atan2(rightHeightDifference, floorDistance) * Mathf.Rad2Deg;
                 
                // Reject path if slope is outside allowed range
                if (angle > maxSlopeAngle || angle < minSlopeAngle ||
                    leftAngle > maxSlopeAngle || leftAngle < minSlopeAngle ||
                    rightAngle > maxSlopeAngle || rightAngle < minSlopeAngle)
                {
                    return false;
                }
            }
        }
        return true;
    }

    /// <summary>
    /// Checks whether a spline path is valid by sampling points along it and verifying
    /// that each consecutive segment represents a valid movement.
    /// The validation is performed in 2D space (XZ plane), ignoring height differences.
    /// </summary>
    /// <param name="testSpline">The spline to be tested for validity.</param>
    /// <param name="splineCutStep">The distance interval used to discretize the spline into sample points.</param>
    /// <param name="checkStepInterval">The step size used when validating movement between two points.</param>
    /// <param name="avoidanceZones">A list of avoidance zones that the path must not intersect.</param>
    /// <param name="maxSlopeAngle">The maximum allowable slope angle for movement between points.</param>
    /// <param name="minSlopeAngle">The minimum allowable slope angle for movement between points.</param>
    /// <param name="carWidth">The width of the car, used to check for clearance in avoidance zones.</param>
    /// <param name="widthMarginFraction">An additional margin fraction added to the car width for clearance checks.</param>
    /// <returns>
    /// A tuple containing:
    /// - A boolean indicating whether the path is valid,
    /// - The first invalid point encountered (if any),
    /// - The point immediately preceding the invalid point.
    /// </returns>
    public static (bool, Vector3, Vector3) IsSplineValid(Spline testSpline, float splineCutStep, float checkStepInterval, List<AvoidanceZone> avoidanceZones, float maxSlopeAngle, float minSlopeAngle, float carWidth, float widthMarginFraction)
    {
        Vector3 invalidPoint = Vector3.zero;
        Vector3 previousPoint = Vector3.zero;

        List<Vector3> samplePoints = new List<Vector3>();

        Vector3 startPoint = testSpline.EvaluatePosition(0f);
        startPoint.y = 0f;
        
        // Discretize spline based on length
        float totalLength = testSpline.GetLength();
        int steps = Mathf.CeilToInt(totalLength / splineCutStep);

        for (int i = 0; i <= steps; i++)
        {
            float t = i / (float)steps;

            Vector3 point = testSpline.EvaluatePosition(t);
            // Ignore height for check
            point.y = 0f;
            samplePoints.Add(point);
        }

        // Check movement validity between sampled points
        for (int i = 0; i < samplePoints.Count - 1; i++)
        {
            previousPoint = samplePoints[i];
            if (!IsMoveValid(new Vector2(samplePoints[i].x, samplePoints[i].z), new Vector2(samplePoints[i + 1].x, samplePoints[i + 1].z), checkStepInterval, avoidanceZones, maxSlopeAngle, minSlopeAngle, carWidth, widthMarginFraction))
            {
                // Invalid segment found
                invalidPoint = samplePoints[i + 1];
                return (false, invalidPoint, previousPoint);
            }
        }
        
        // All segments valid
        return (true, invalidPoint, previousPoint);
    }
}
