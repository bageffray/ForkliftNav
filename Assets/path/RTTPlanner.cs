using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;

/// <summary>
/// Contains the results of a path planning via RRT (Rapidly-exploring Random Tree).
/// </summary>
public class RRTResult
{
    public List<Vector3> PathPoints = new List<Vector3>();
    public List<TreeNode> ExploredPoints = new List<TreeNode>();
    public List<Vector3> DebugBypasses = new List<Vector3>();
    public bool Success = false;
    public int Iterations = 0;
}


public static class RRTPlanner
{
    /// <summary>
    /// Generates a complete path between a series of points using the RRT algorithm.
    /// </summary>
    /// <param name="points">List of points to connect with the path.</param>
    /// <param name="stepSize">Distance between nodes during random exploration.</param>
    /// <param name="maxIterations">Maximum number of iterations before giving up.</param>
    /// <param name="goalBias">Probability of generating a random point on the target to speed up convergence.</param>
    /// <param name="simplify">Indicates whether the generated path should be simplified.</param>
    /// <param name="pathVerificationStep">Step size for verifying the validity of a path segment.</param>
    /// <param name="avoidanceZones">List of zones to avoid for the path.</param>
    /// <param name="maxSlopeAngle">Maximum allowed slope angle for the path.</param>
    /// <param name="minSlopeAngle">Minimum allowed slope angle for the path.</param>
    /// <param name="carWidth">Width of the vehicle for path verification.</param>
    /// <returns>An RRTResult object containing the final path, explored points, and success status.</returns>
    public static RRTResult FindFullRRTPath(List<Vector3> points, float stepSize, int maxIterations, float goalBias, bool simplify, float pathVerificationStep, List<AvoidanceZone> avoidanceZones, float maxSlopeAngle, float minSlopeAngle, float carWidth, float widthMarginFraction)
    {
        RRTResult result = new RRTResult();

        List<Vector3> rrtPathPoints = new List<Vector3>();
        // Generate RRT path for each segment between consecutive points
        for (int p = 0; p < points.Count - 1; p++)
        {
            (List<Vector3> intermediaryPoints, List<TreeNode> explored) = RRTPath(points[p], points[p + 1], stepSize, maxIterations, goalBias, pathVerificationStep, avoidanceZones, maxSlopeAngle, minSlopeAngle, carWidth, widthMarginFraction);
            result.ExploredPoints.AddRange(explored);
            // If no path found for this segment, abort
            if (intermediaryPoints.Count == 0)
            {
                result.Success = false;
                result.Iterations = maxIterations;
                return result;
            }
            // Simplify path by reducing intermediate nodes if requested
            if (simplify)
            {
                intermediaryPoints = SimplifyPath(intermediaryPoints, pathVerificationStep, avoidanceZones, maxSlopeAngle, minSlopeAngle, carWidth, widthMarginFraction);
            }
            
            rrtPathPoints.AddRange(intermediaryPoints);
            result.PathPoints = rrtPathPoints;
        }

        // remove duplicates
        for (int i = result.PathPoints.Count - 1; i > 0; i--)
        {
            if (result.PathPoints[i] == result.PathPoints[i - 1])
                result.PathPoints.RemoveAt(i);
        }

        result.Success = result.PathPoints.Count > 0;
        return result;
    }

    /// <summary>
    /// Finds a path between two points (startPos and targetPos) using the RRT algorithm.
    /// </summary>
    /// <param name="startPos">Starting point.</param>
    /// <param name="targetPos">Target point.</param>
    /// <param name="stepSize">Distance between nodes during exploration.</param>
    /// <param name="maxIterations">Maximum number of iterations before giving up.</param>
    /// <param name="goalBias">Probability of generating a random point on the target to speed up convergence.</param>
    /// <param name="pathVerificationStep">Step size for verifying the validity of a path segment.</param>
    /// <param name="avoidanceZones">List of zones to avoid for the path.</param>
    /// <param name="maxSlopeAngle">Maximum allowed slope angle for the path.</param>
    /// <param name="minSlopeAngle">Minimum allowed slope angle for the path.</param>
    /// <param name="carWidth">Width of the vehicle for path verification.</param>
    /// <param name="widthMarginFraction">Additional margin fraction to add to car width for clearance checks.</param>
    /// <returns>
    /// A tuple containing: 
    /// - the list of intermediate points forming the path, 
    /// - the list of nodes explored during generation.
    /// </returns>
    static (List<Vector3>, List<TreeNode>) RRTPath(Vector3 startPos, Vector3 targetPos, float stepSize, int maxIterations, float goalBias, float pathVerificationStep, List<AvoidanceZone> avoidanceZones, float maxSlopeAngle, float minSlopeAngle, float carWidth, float widthMarginFraction)
    {
        List<TreeNode> explored = new List<TreeNode>();

        // Default values for map bounds (if no terrain found)
        float mapMinX = -248f;
        float mapMaxX = 2f;
        float mapMinZ = -125f;
        float mapMaxZ = 125f;

        Terrain terrain = Terrain.activeTerrain;
        if (terrain != null)
        {
            Vector3 terrainPos = terrain.transform.position;
            Vector3 terrainSize = terrain.terrainData.size;

            mapMinX = terrainPos.x;
            mapMaxX = terrainPos.x + terrainSize.x;
            mapMinZ = terrainPos.z;
            mapMaxZ = terrainPos.z + terrainSize.z;
        }

        // Initialize RRT with start node 
        List<TreeNode> nodes = new List<TreeNode>();
        nodes.Add(new TreeNode(new Vector2(startPos.x, startPos.z)));

        Unity.Mathematics.Random random = new Unity.Mathematics.Random();
        random.InitState((uint)System.DateTime.Now.Millisecond);

        int step = 0;
        while (step < maxIterations)
        {
            // Generate random point with goal bias (using target point directly sometimes for faster convergence)
            Vector2 randPos;
            if (random.NextFloat() < goalBias)
                randPos = new Vector2(targetPos.x, targetPos.z);
            else
                randPos = new Vector2(random.NextFloat(mapMinX, mapMaxX), random.NextFloat(mapMinZ, mapMaxZ));

            // Find nearest node in tree
            TreeNode nearest = GetNearestNode(nodes, randPos);
            Vector2 nearestPos2D = new Vector2(nearest.Position.x, nearest.Position.z);

            // Steer toward random point
            Vector2 direction = (randPos - nearestPos2D).normalized;
            Vector2 newPos2D = nearestPos2D + direction * stepSize;

            // Clamping new position to map bounds
            newPos2D.x = Mathf.Clamp(newPos2D.x, mapMinX, mapMaxX);
            newPos2D.y = Mathf.Clamp(newPos2D.y, mapMinZ, mapMaxZ);
            
            // Check if path to new point valid
            if (PathValidator.IsMoveValid(nearestPos2D, newPos2D, pathVerificationStep, avoidanceZones, maxSlopeAngle, minSlopeAngle, carWidth, widthMarginFraction))
            {
                TreeNode newNode = new TreeNode(newPos2D, nearest);
                nodes.Add(newNode);
                // Check if target reached
                if (Vector2.Distance(newPos2D, new Vector2(targetPos.x, targetPos.z)) < stepSize)
                {
                    
                    if (!PathValidator.IsMoveValid(newPos2D, new Vector2(targetPos.x, targetPos.z), pathVerificationStep, avoidanceZones, maxSlopeAngle, minSlopeAngle, carWidth, widthMarginFraction))
                    {
                        step++;
                        continue;
                    }
                    // Build path to target
                    Vector2 targetPos2D = new Vector2(targetPos.x, targetPos.z);
                    TreeNode targetNode = new TreeNode(targetPos2D, newNode);
                    List<Vector3> path = new List<Vector3>();
                    TreeNode current = targetNode;
                    while (current != null)
                    {
                        path.Add(current.Position);
                        current = current.Parent;
                    }
                    path.Reverse(); // Reverse to get path from start to target
                    explored.AddRange(nodes);
                    return (path, explored);
                }
            }
            step++;
        }
        // Empty path if no path found 
        explored.AddRange(nodes);
        return (new List<Vector3>(), explored);
    }

    /// <summary>
    /// Simplifies a path by removing unnecessary intermediate nodes while respecting constraints.
    /// </summary>
    /// <param name="path">Path to simplify.</param>
    /// <param name="pathVerificationStep">Step size for validity verification.</param>
    /// <param name="avoidanceZones">Avoidance zones to consider.</param>
    /// <param name="maxSlopeAngle">Maximum allowed slope angle.</param>
    /// <param name="minSlopeAngle">Minimum allowed slope angle.</param>
    /// <param name="carWidth">Width of the vehicle for path verification.</param>
    /// <param name="widthMarginFraction">Additional margin fraction to add to car width for clearance checks.</param>
    /// <returns>Simplified path as a list of Vector3 points.</returns>
    static List<Vector3> SimplifyPath(List<Vector3> path, float pathVerificationStep, List<AvoidanceZone> avoidanceZones, float maxSlopeAngle, float minSlopeAngle, float carWidth, float widthMarginFraction)
    {
        if (path == null || path.Count < 2)
            return path;

        List<Vector3> simplifiedPath = new List<Vector3>(capacity: path.Count);
        int n = path.Count;
        int cur = 0;
        simplifiedPath.Add(path[0]);

        // from current index, jump to the farthest reachable point ahead.
        while (cur < n - 1)
        {
            int farthest = -1;
            for (int j = n - 1; j > cur; j--)
            {
                if (PathValidator.IsMoveValid(new Vector2(path[cur].x, path[cur].z), new Vector2(path[j].x, path[j].z), pathVerificationStep, avoidanceZones, maxSlopeAngle, minSlopeAngle, carWidth, widthMarginFraction))
                {
                    farthest = j;
                    break;
                }
            }

            // If no farther point is reachable, return the original path
            if (farthest == -1) {
                return new List<Vector3>(path); // cannot simplify
            }

            Vector3 nextPoint = path[farthest];
            if (simplifiedPath.Count == 0 || simplifiedPath[simplifiedPath.Count - 1] != nextPoint)
                simplifiedPath.Add(nextPoint);

            cur = farthest;
        }

        return simplifiedPath;
    }

    /// <summary>
    /// Returns the nearest node to a given point from a list of nodes.
    /// </summary>
    /// <param name="nodes">List of existing nodes.</param>
    /// <param name="point">Target point for which the nearest node is to be found.</param>
    /// <returns>The TreeNode closest to the target point.</returns>
    static TreeNode GetNearestNode(List<TreeNode> nodes, Vector2 point)
    {
        TreeNode nearest = null;
        float minDist = float.MaxValue;
        // Find node with minimum distance to point 
        foreach (TreeNode node in nodes)
        {
            Vector2 nodePos2D = new Vector2(node.Position.x, node.Position.z);
            float dist = Vector2.Distance(nodePos2D, point);
            if (dist < minDist)
            {
                minDist = dist;
                nearest = node;
            }
        }
        return nearest;
    }
}
