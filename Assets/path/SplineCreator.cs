using UnityEngine;
using UnityEngine.Splines;
using System.Collections.Generic;
/// <summary>
/// Classe utilitaire pour la création et la manipulation de splines Bezier dans Unity.
/// </summary>
public static class SplineCreator
{

    private const float NEAR_ZERO_MAGNITUDE = 0.001f;
    // Divisor used to compute control handle length from segment direction.
    private const float HANDLE_DIVISOR = 6f;

    /// <summary>
    /// Computes the signed horizontal angle between an outgoing direction vector
    /// and the direction from a reference point toward a target point.
    /// The angle is measured around the world up axis (Y), in degrees.
    /// </summary>
    /// <param name="from">The origin point from which the target direction is evaluated.</param>
    /// <param name="to">The target point used to compute the direction vector.</param>
    /// <param name="fromOutDir">The reference outgoing direction from the origin point.</param>
    /// <returns>
    /// The signed angle in degrees between the outgoing direction and the target direction.
    /// A positive value indicates a rotation to the right, and a negative value to the left.
    /// </returns>
    static float AngleToPoint(Vector3 from, Vector3 to, Vector3 fromOutDir)
    {
        /// Direction from the origin point to the target point
        Vector3 toDir = (to - from).normalized;
        /// Angle around vertical axis
        float angle = Vector3.SignedAngle(((Vector3)fromOutDir).normalized, toDir, Vector3.up);

        return angle;
    }

    /// <summary>
    /// Determines the most suitable perpendicular direction to a given path direction,
    /// selecting the side that is the most different from the previous outgoing direction.
    /// This allows to avoid S-shaped turns when smoothing spline tangents.
    /// The computation is performed in the horizontal plane (XZ), ignoring vertical components.
    /// </summary>
    /// <param name="prevDir">The direction of the previous path segment.</param>
    /// <param name="prevOut">The previous outgoing direction used as a reference for side selection.</param>
    /// <returns>
    /// A normalized perpendicular direction vector that is the most different from the previous outgoing direction.
    /// </returns>
    static Vector3 BestPerpendicular(Vector3 prevDir, Vector3 prevOut)
    {
        /// Project onto horizontal plane
        prevDir.y = 0f; // ignore y direction
        prevOut.y = 0f; // ignore y direction
        prevDir.Normalize();
        prevOut.Normalize();

        /// Compute both perpendicular directions
        Vector3 perpA = Vector3.Cross(Vector3.up, prevDir).normalized;
        Vector3 perpB = -perpA;

        /// Measure angle to previous outgoing direction
        float angleA = Vector3.Angle(prevOut, perpA);
        float angleB = Vector3.Angle(prevOut, perpB);

        /// Choose angle with largest difference
        return (angleA > angleB) ? perpA : perpB;
    }

    /// <summary>
    /// Computes a smoothed tangent direction at a spline knot by blending the incoming
    /// and outgoing segment directions.
    /// If the two directions cancel each other out (nearly opposite),
    /// a stable perpendicular direction is chosen to avoid S-shaped turns.
    /// </summary>
    /// <param name="prevDir">Normalized direction of the previous spline segment.</param>
    /// <param name="nextDir">Normalized direction of the next spline segment.</param>
    /// <param name="prevOut">The outgoing tangent direction of the previous knot.</param>
    /// <returns>
    /// A normalized direction vector representing the smoothed tangent at the knot.
    /// </returns>
    static Vector3 GetSmoothedTangent(Vector3 prevDir, Vector3 nextDir, Vector3 prevOut)
    {
        /// combine direction to smooth
        Vector3 dir = prevDir + nextDir;

        /// Choose perpendicular if direction cancels out
        if (dir.magnitude < NEAR_ZERO_MAGNITUDE)
        {
            dir = BestPerpendicular(prevDir, prevOut);
        }

        return dir.normalized;
    }

    /// <summary>
    /// Smooths the tangents of a Bezier spline to ensure continuous and drivable curvature.
    /// The method enforces angular constraints at the start and end of the spline by
    /// inserting additional knots when necessary, and computes tangents for each knot
    /// based on neighboring segments.
    /// </summary>
    /// <param name="originalSpline">The original Bezier spline to be smoothed.</param>
    /// <param name="startDirection">Desired outgoing direction at the start of the spline.</param>
    /// <param name="endDirection">Desired incoming direction at the end of the spline.</param>
    /// <param name="tangentFactor">Scaling factor applied to all computed tangent lengths.</param>
    /// <param name="maxArrivingAngle">Maximum allowed angle (in degrees) between the spline direction and the specified start/end directions (to add smoothing knots).</param>
    /// <param name="smoothingKnotDistance">Distance (in x and z) from the start/end knot to place smoothing knots when needed.</param>
    /// <param name="tanLenDivider">Divider used to compute tangent lengths based on segment lengths.</param>
    /// <param name="maximumTangentLength">Maximum allowed length for tangents (if limitMaximumTangentLength is true).</param>
    /// <param name="limitMaximumTangentLength">Whether to enforce the maximum tangent length limit.</param>
    /// <returns>The smoothed Bezier spline with updated tangents.</returns>
    public static Spline SmoothSplineTangents(Spline originalSpline, Vector3 startDirection, Vector3 endDirection, float tangentFactor, float maxArrivingAngle, float smoothingKnotDistance, float tanLenDivider, float maximumTangentLength, bool limitMaximumTangentLength)
    {
        
        startDirection = startDirection.normalized;
        endDirection = endDirection.normalized;
        // compute angle between start direction and first segment
        float startAngle = AngleToPoint(originalSpline[0].Position, originalSpline[1].Position, startDirection);
        // if angle too large, insert smoothing knot
        if(startAngle > maxArrivingAngle || startAngle < -maxArrivingAngle)
        {
            BezierKnot newKnot;
            Vector3 forward = startDirection.normalized;
            // determine side to place depening on angle sign
            if(startAngle <= 0f)
            {
                Vector3 left = Vector3.Cross(Vector3.up, forward).normalized;
                newKnot = new BezierKnot((Vector3)originalSpline[0].Position + forward * smoothingKnotDistance + left * smoothingKnotDistance);
            }
            else
            {
                Vector3 right = Vector3.Cross(forward, Vector3.up).normalized;
                newKnot = new BezierKnot((Vector3)originalSpline[0].Position + forward * smoothingKnotDistance + right * smoothingKnotDistance);
            }
            // Insert smoothing knot after first knot
            originalSpline.Insert(1, newKnot);
        }

        // Compute angle between desired end direction and last spline segment
        float endAngle = AngleToPoint(originalSpline[originalSpline.Count - 1].Position, originalSpline[originalSpline.Count - 2].Position, -endDirection);
        // Insert smoothing knot before last knot if angle too large
        if(endAngle > maxArrivingAngle || endAngle < -maxArrivingAngle)
        {
            BezierKnot newKnot;
            Vector3 backward = -endDirection.normalized;
            if(endAngle <= 0f)
            {
                Vector3 left = Vector3.Cross(Vector3.up, backward).normalized;
                newKnot = new BezierKnot((Vector3)originalSpline[originalSpline.Count - 1].Position + backward * smoothingKnotDistance + left * smoothingKnotDistance);
            }
            else
            {
                Vector3 right = Vector3.Cross(backward, Vector3.up).normalized;
                newKnot = new BezierKnot((Vector3)originalSpline[originalSpline.Count - 1].Position + backward * smoothingKnotDistance + right * smoothingKnotDistance);
            }
            // Insert before last knot
            originalSpline.Insert(originalSpline.Count - 1, newKnot);
        }

        BezierKnot k0 = originalSpline[0];
        //tangent length based on distance to next knot
        float firstLength = Vector3.Distance(k0.Position, originalSpline[1].Position)/tanLenDivider;
        // Clamp tangent length if needed 
        if (limitMaximumTangentLength)
        {
            firstLength = Mathf.Min(maximumTangentLength, firstLength);
        }
        // force tangent to match desired start direction
        k0.TangentOut = startDirection * firstLength * tangentFactor;
        k0.TangentIn  = -k0.TangentOut;

        originalSpline[0] = k0;
        
        // Compute tangents for all intermediate knots
        for (int i = 1; i < originalSpline.Count - 1; i++) // skip first/last
        {
            BezierKnot k = originalSpline[i];
            // previous outgoing tangent for continuity
            Vector3 prevOut = ((Vector3) originalSpline[i-1].TangentOut).normalized;
            // directions of incoming and outgoing segments
            Vector3 prevDir = ((Vector3)k.Position - (Vector3)originalSpline[i - 1].Position).normalized;
            Vector3 nextDir = ((Vector3)originalSpline[i + 1].Position - (Vector3)k.Position).normalized;
            // compute smoothed tangent direction 
            Vector3 dir = GetSmoothedTangent(prevDir, nextDir, prevOut);
            // compute tangent lengths based on adjacent segment lengths
            float inLength = Vector3.Distance(k.Position, originalSpline[i - 1].Position)/tanLenDivider;
            float outLength = Vector3.Distance(k.Position, originalSpline[i + 1].Position)/tanLenDivider;

            if (limitMaximumTangentLength)
            {
                inLength = Mathf.Min(maximumTangentLength, inLength);
                outLength = Mathf.Min(maximumTangentLength, outLength);
            }
            // set tangents based on smoothed direction
            k.TangentIn  = -dir * inLength * tangentFactor;
            k.TangentOut =  dir * outLength * tangentFactor;

            originalSpline[i] = k;
        }

        BezierKnot kN = originalSpline[originalSpline.Count - 1];

        float lastLength = Vector3.Distance(kN.Position, originalSpline[originalSpline.Count - 2].Position)/tanLenDivider;

        if (limitMaximumTangentLength)
        {
            lastLength = Mathf.Min(maximumTangentLength, lastLength);
        }

        // force incoming tangent to match desired end direction
        kN.TangentIn  = -endDirection * lastLength * tangentFactor;
        kN.TangentOut = -kN.TangentIn;

        originalSpline[originalSpline.Count - 1] = kN;

        return originalSpline;
    }

    /// <summary>
    /// Generates a 3D spline from a 2D spline by sampling terrain height
    /// and removing points that are closer than a given distance to the target.
    /// The resulting spline follows the terrain surface and is trimmed near the end
    /// to maintain a safe distance from the target position.
    /// </summary>
    /// <param name="originalSpline">The original spline defined in the XZ plane.</param>
    /// <param name="stepsPerMeter">
    /// Number of sampling steps per meter used to discretize the spline.
    /// Higher values increase accuracy but reduce performance.
    /// </param>
    /// <param name="distanceToKeep">
    /// Minimum distance to maintain from the end of the spline.
    /// Points closer than this distance are removed.
    /// </param>
    /// <returns>
    /// A new spline adjusted to terrain height and trimmed near the target.
    /// </returns>
    public static Spline MakeSpline3D(Spline originalSpline, int stepsPerMeter, float distanceToKeep)
    {
        Spline spline3D = new Spline();
        // total step along the spline (ensure at least one step to avoid division by zero)
        int steps = Mathf.Max(1, Mathf.RoundToInt(stepsPerMeter * originalSpline.GetLength()));

        List<Vector3> points = new List<Vector3>();
        // sample points alorg the spline and project to terrain
        for (int i = 0; i <= steps; i++)
        {
            float t = i / (float)steps;
            Vector3 pos = originalSpline.EvaluatePosition(t);
            pos.y = Terrain.activeTerrain.SampleHeight(pos);

            points.Add(pos);
        }
        // compute final postion and its adjusted terrain height
        Vector3 endPos = originalSpline.EvaluatePosition(1f);
        endPos.y = Terrain.activeTerrain.SampleHeight(endPos);
        // Remove points too close to target position 
        for (int i = points.Count - 1; i >= 0; i--)
        {
            float dist = Vector3.Distance(points[i], endPos);
            if (dist < distanceToKeep)
            {
                points.RemoveAt(i);
            }
        }

        // Build bezier knots with computed tangents
        for (int i = 0; i < points.Count; i++)
        {
            Vector3 prev = i > 0 ? points[i - 1] : points[i];
            Vector3 next = i < points.Count - 1 ? points[i + 1] : points[i];
            // Direction vector in XZ plane
            Vector3 dir = next - prev;
            dir.y = 0f;
            // Handle length
            Vector3 handle = dir / HANDLE_DIVISOR;

            BezierKnot knot = new BezierKnot(points[i]);

            knot.TangentIn = -handle;
            knot.TangentOut = handle;

            spline3D.Add(knot);
        }

        return spline3D;
    }

    /// <summary>
    /// Builds a 2D Bezier spline from a list of world-space points.
    /// The Y coordinate is ignored to ensure a flat spline in the XZ plane,
    /// which can later be lifted onto terrain.
    /// </summary>
    /// <param name="points">
    /// Ordered list of waypoints defining the spline path.
    /// </param>
    /// <returns>
    /// A Bezier spline built from the given points, or null if input is invalid.
    /// </returns>
    public static Spline BuildSpline(List<Vector3> points)
    {
        // Spline needs at least 2 points 
        if (points == null || points.Count < 2) {
            return null;
        }

        Spline spline = new Spline();
        // Create one knot per point 
        for (int i = 0; i < points.Count; i++)
        {
            spline.Add(new BezierKnot(new Vector3(points[i].x, 0, points[i].z))); // keep y constant for now
        }

        return spline;
    }
}