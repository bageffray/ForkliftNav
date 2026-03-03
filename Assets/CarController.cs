using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Splines;
using Unity.Mathematics;

public enum SpeedAlgorithm
{
    Constant,
    AccelerateStop,
    Trapezoidal
}


public enum PathAlgorithm
{
    Spline,
    RRT,
    RRTSmoothed,
    RRTSimplified,
    RRTSimplifiedSmoothed
}
/// <summary>
/// Car controller, managing path generation, spline following, 
/// speed management, and obstacle avoidance.
/// </summary>
public class CarController : MonoBehaviour
{   
    #region Variables
    public List<AvoidanceZone> avoidanceZones = new List<AvoidanceZone>();

    [Header("Algorithm Settings")]
    public SpeedAlgorithm speedAlgorithm = SpeedAlgorithm.Trapezoidal;
    public PathAlgorithm pathAlgorithm = PathAlgorithm.Spline;

    [Space]
    [Header("View settings")]
    public bool debugView = true;
    
    [Space]
    [Header("References")]
    // Public
    public Transform car;
    public Rigidbody carRigidbody;

    // Private
    private float carWidth = 1.2f;

    [Space]
    [Header("Validation Settings")]
    public float maxSlopeAngle = 30f;
    public float minSlopeAngle = -70f;
    public float widthMarginFraction = 1f;

    [Space]
    [Header("Smoothing Settings")]
    // Public
    public float tangentSmoothFactor = 0.7f;
    public bool limitMaximumTangentLength = false;
    public float maximumTangentLength = 10f;

    // Private
    // Distance from departure or arrival knots when a smoothing knot is added
    private float smoothingKnotDistance = 5f;
    // Maximum angle (in degrees) to consider adding a smoothing knot
    private float maxArrivingAngle = 140f;
    private float tanLenDivider = 2f;

    [Space]
    [Header("Speed Settings")]
    // Public
    public float maxSpeed = 5f;
    public float acceleration = 1f;

    // Private
    private float speed = 0f;

    [Space]
    [Header("RRT Settings")]
    // Public
    public float rrtStepSize = 1f;
    public int rrtMaxIterations = 10000;
    public float rrtGoalBias = 0.05f;
    // Private
    private List<TreeNode> exploredPoints = new List<TreeNode>();
    private List<Vector3> pathPoints = new List<Vector3>();


    [Space]
    [Header("Repath Settings")]
    // Public
    public bool checkValidity = true;
    public int maxRepathAttempts = 15;
    public float repathOffsetDistance = 3f;
    public float repathForwardDistance = 5f;
    public int discretizationStepsPerMeter = 20;
    public float checkStepInterval = 1f;
    public float pathVerificationStep = 0.1f;

    // Private
    private List<Vector3> lastBypassTryPoint = new List<Vector3>();
    private Vector3 debugInvalidPoint;
    private Vector3 debugPreviousPoint;

    // Runtime private variables
    private System.Action onComplete;

    private Spline spline;
    private Spline spline3D;
    private float traveled;
    private float accelTime;
    private float constantTime;
    private float decelTime;
    private float totalTime;
    private float localTime = 0f;

    private float minimumSplineTraveled = 0.999f;

    private bool moving = false;

    #endregion

    #region Update
    void FixedUpdate()
    {
        if (!this.moving || this.spline3D == null) {
            return;
        }

        this.localTime += Time.fixedDeltaTime;

        this.speed = this.GetSpeed(this.localTime);

        this.traveled += this.speed * Time.fixedDeltaTime;

        // normalized traveled distance along spline
        float t = this.traveled / this.spline3D.GetLength();

        if (t >= 1f) // clamp at 1 (end of spline)
        {
            t = 1f;
        }

        // position
        Vector3 splinePos = this.spline3D.EvaluatePosition(t);
        Vector3 currentPos = this.carRigidbody.position;

        Vector3 newPos = new Vector3(
            splinePos.x,
            currentPos.y, // y is subject to gravity
            splinePos.z
        );

        // apply the new position
        this.carRigidbody.MovePosition(newPos);

        // rotation
        Quaternion currentRotation = this.carRigidbody.rotation;
        Vector3 tangent = this.spline3D.EvaluateTangent(t);

        Quaternion yawRotation = Quaternion.LookRotation(tangent.normalized, Vector3.up);

        Quaternion target = Quaternion.Euler(
            currentRotation.eulerAngles.x,
            yawRotation.eulerAngles.y,
            currentRotation.eulerAngles.z
        );

        // apply the new rotation
        this.carRigidbody.MoveRotation(target);

        // Stopping if the end of the spline is reached
        if (t >= 1f || this.speed == 0f && t >= this.minimumSplineTraveled)
        {
            this.moving = false;
            this.onComplete?.Invoke();
        }
    }
    #endregion

    #region Public Entry Point
    /// <summary>
    /// Generates and starts a path-following motion toward a grab target,
    /// using the selected path planning algorithm.
    /// </summary>
    /// <param name="targetPos">World-space target position.</param>
    /// <param name="targetRotation">Desired final car rotation.</param>
    /// <param name="waypoints">Intermediate waypoints.</param>
    /// <param name="distanceToKeep">Stopping distance from the target.</param>
    /// <param name="onComplete">Callback invoked on success.</param>
    /// <param name="onError">Callback invoked on failure.</param>
    public void GoToGrab(Vector3 targetPos, Quaternion targetRotation, List<Vector3> waypoints, float distanceToKeep, System.Action onComplete, System.Action onError) {

        float distToTarget = Vector3.Distance(this.car.position, targetPos);

        // Stop if already within distance to target
        if (Vector3.Distance(this.car.position, targetPos) <= distanceToKeep || distToTarget < 0.1f) {
            Debug.Log("Already within distance to target, no movement needed.");
            onComplete?.Invoke();
            return;
        }

        //reset path related data
        this.spline = null;
        this.spline3D = null;
        this.pathPoints.Clear();
        this.exploredPoints.Clear();
        this.lastBypassTryPoint.Clear();

        // Build initial path waypoint list
        List<Vector3> points = new List<Vector3>();
        points.Add(this.car.position);

        foreach (Vector3 wp in waypoints) {
            points.Add(wp);
        }
        points.Add(new Vector3(targetPos.x, targetPos.y, targetPos.z));

        // select and execute path algorithm
        switch (this.pathAlgorithm) {
            case PathAlgorithm.Spline:
            if(!this.TryMakeValidPath(points, targetPos, distanceToKeep, this.checkValidity))
                {
                    Debug.Log("Failed to generate a valid path to target");
                    this.spline = null;
                    this.spline3D = null;
                    onError?.Invoke();
                    return;
                }
                break;
            case PathAlgorithm.RRT:
            case PathAlgorithm.RRTSmoothed:
            case PathAlgorithm.RRTSimplified:
            case PathAlgorithm.RRTSimplifiedSmoothed:
                RRTResult rrtResult = RRTPlanner.FindFullRRTPath(points, this.rrtStepSize, this.rrtMaxIterations, this.rrtGoalBias, (this.pathAlgorithm == PathAlgorithm.RRTSimplified || this.pathAlgorithm == PathAlgorithm.RRTSimplifiedSmoothed), this.pathVerificationStep, this.avoidanceZones, this.maxSlopeAngle, this.minSlopeAngle, this.carWidth, this.widthMarginFraction);
                if (!rrtResult.Success || rrtResult.PathPoints.Count == 0) {
                    Debug.Log("No RRT path found, cannot proceed to GoToGrab.");
                    onError?.Invoke();
                    return;
                }

                // store debug/explored info
                this.exploredPoints.AddRange(rrtResult.ExploredPoints);
                this.lastBypassTryPoint.AddRange(rrtResult.DebugBypasses);

                List<Vector3> rrtPoints = rrtResult.PathPoints;
                this.pathPoints = rrtPoints;
                // Optional smoothing
                if (this.pathAlgorithm == PathAlgorithm.RRTSimplifiedSmoothed || this.pathAlgorithm == PathAlgorithm.RRTSmoothed)
                {
                    if (!this.TryMakeValidPath(rrtPoints, targetPos, distanceToKeep, this.checkValidity))
                    {
                        Debug.Log("Failed to generate a valid path to target after RRT simplification and smoothing, falling back to non checked path.");
                        this.TryMakeValidPath(rrtPoints, targetPos, distanceToKeep, false);
                    }
                }
                else {
                    this.BuildSpline(rrtPoints);
                    this.MakeSpline3D(this.spline, this.discretizationStepsPerMeter, distanceToKeep);
                }
                break;
            default:
                throw new System.Exception("Unknown path algorithm");
        }
        // speed profile setup
        this.ComputeTrapezoidal();
        this.traveled = 0f;
        // start movement
        this.onComplete = onComplete;
        this.speed = 0f;
        this.localTime = 0f;
        Debug.Log("Starting movement toward target");
        this.moving = true;
    }
    #endregion

    #region Spline Management

    /// <summary>
    /// Smooths the tangents of the current spline to ensure continuous curvature
    /// and limit sharp direction changes at spline knots.
    /// </summary>
    /// <param name="startDirection">Desired direction at spline start.</param>
    /// <param name="endDirection">Desired direction at spline end.</param>
    /// <param name="tangentFactor">Scaling factor applied to tangent lengths.</param>
    void SmoothSplineTangents(Vector3 startDirection, Vector3 endDirection, float tangentFactor)
    {
        this.spline = SplineCreator.SmoothSplineTangents(this.spline, startDirection, endDirection, tangentFactor, this.maxArrivingAngle, this.smoothingKnotDistance, this.tanLenDivider, this.maximumTangentLength, this.limitMaximumTangentLength);
    }

    /// <summary>
    /// Converts a 2D spline into a 3D spline by sampling terrain height 
    /// and stopping at the specified distance from the target.
    /// </summary>
    /// <param name="originalSpline">The base spline in XZ plane.</param>
    /// <param name="stepsPerMeter">Sampling density along the spline.</param>
    /// <param name="distanceToKeep">Final distance offset from the target.</param>
    void MakeSpline3D(Spline originalSpline, int stepsPerMeter, float distanceToKeep)
    {
        this.spline3D = SplineCreator.MakeSpline3D(originalSpline, stepsPerMeter, distanceToKeep);
    }

    /// <summary>
    /// Builds a spline from a list of world-space waypoints.
    /// </summary>
    /// <param name="points">Ordered list of points defining the path.</param>
    void BuildSpline(List<Vector3> points)
    {
        this.spline = SplineCreator.BuildSpline(points);
    }

    /// <summary>
    /// Validates a spline by checking collisions with avoidance zones,
    /// and terrain slope constraints
    /// </summary>
    /// <param name="testSpline">Spline to validate.</param>
    /// <param name="splineCutStep">Distance between spline samples.</param>
    /// <param name="checkStepInterval">Interval used for collision checks.</param>
    /// <returns>
    /// A tuple containing:
    /// - validity flag
    /// - first invalid point (if any)
    /// - last valid point before failure
    /// </returns>
    (bool, Vector3, Vector3) IsPathValid(Spline testSpline, float splineCutStep, float checkStepInterval) {
        return PathValidator.IsSplineValid(testSpline, splineCutStep, checkStepInterval, this.avoidanceZones, this.maxSlopeAngle, this.minSlopeAngle, this.carWidth, this.widthMarginFraction);
    }
    #endregion

    #region Speed Computation

    /// <summary>
    /// Computes the current speed of the vehicle based on the selected speed algorithm.
    /// The algorithm can be constant, acceleration with stop, or trapezoidal speed profile.
    /// </summary>
    /// <param name="time">The elapsed time since the start of the movement.</param>
    /// <returns>
    /// The computed speed at the given time.
    /// </returns>
    float GetSpeed(float time) {
        switch (this.speedAlgorithm) {
            case SpeedAlgorithm.Constant:
                return this.maxSpeed;
            case SpeedAlgorithm.AccelerateStop:
                return Mathf.Min(this.speed + this.acceleration * Time.fixedDeltaTime, this.maxSpeed);
            case SpeedAlgorithm.Trapezoidal:
                return this.GetSpeedTrapezoidal(time);
            default:
                throw new System.Exception("Unknown speed algorithm");
        }

    }

    /// <summary>
    /// Precomputes timing values for a trapezoidal speed profile along the current spline.
    /// The profile consists of an acceleration phase, an optional constant-speed phase,
    /// and a deceleration phase, depending on the total path length.
    /// </summary>
    void ComputeTrapezoidal()
    {
        float timeToMaxSpeed = this.maxSpeed / this.acceleration;
        float distanceToMaxSpeed = 0.5f * this.acceleration * timeToMaxSpeed * timeToMaxSpeed;
        float distanceToStop = distanceToMaxSpeed;

        Debug.Log("Distance to max speed: " + distanceToMaxSpeed);
        Debug.Log("Distance to stop: " + distanceToStop);
        Debug.Log("Total spline length: " + this.spline.GetLength());
        Debug.Log("Total 3Dspline length: " + this.spline3D.GetLength());

        /// if path too short use a "triangular" speed profile
        if (this.spline3D.GetLength() < (distanceToMaxSpeed + distanceToStop))
        {
            this.accelTime = Mathf.Sqrt(this.spline3D.GetLength() / this.acceleration);
            this.decelTime = this.accelTime;
            this.constantTime = 0f;
        }
        /// else use trapezoidal profile
        else
        {
            this.accelTime = timeToMaxSpeed;
            this.decelTime = timeToMaxSpeed;
            this.constantTime = (this.spline3D.GetLength() - distanceToMaxSpeed - distanceToStop) / this.maxSpeed;
        }
        /// Total time accumulated along the splines
        this.totalTime = this.accelTime + this.constantTime + this.decelTime;
    }

    /// <summary>
    /// Computes the vehicle speed at a given time using a trapezoidal speed profile.
    /// The profile includes acceleration, constant speed, and deceleration phases.
    /// </summary>
    /// <param name="time">The elapsed time since the start of the movement.</param>
    /// <returns>
    /// The speed at the given time according to the trapezoidal profile.
    /// </returns>
    float GetSpeedTrapezoidal(float time)
    {
        if (time < this.accelTime)
        {
            /// Acceleration phase
            return this.acceleration * time;
        }
        else if (time < this.accelTime + this.constantTime)
        {
            /// Constant speed phase
            return this.maxSpeed;
        }
        else if (time < this.totalTime)
        {
            /// Deceleration phase
            float dt = this.totalTime - time;
            return this.acceleration * dt;
        }
        else
        {
            /// Movement finished
            return 0f;
        }
    }

    #endregion

    #region Repathing    
    /// <summary>
    /// Attempts to generate a valid bypass point around an obstacle located on a path.
    /// The bypass is computed by offsetting the obstacle point laterally and forward
    /// relative to the path direction.
    /// </summary>
    /// <param name="from"> The starting point of the current path segment.</param>
    /// <param name="to"> The target point of the current path segment.</param>
    /// <param name="obstaclePoint"> The point where the path is obstructed.</param>
    /// <param name="bypassPoint"> The generated bypass point, if a valid one is found.</param>
    /// <returns>
    /// True if a valid bypass point is found, false otherwise.
    /// </returns>
    (bool, Vector3) TryGenerateBypass(Vector3 from, Vector3 to, Vector3 obstaclePoint){
        Vector3 bypassPoint = Vector3.zero;
        /// Compute forward and lateral directions based on path segment
        Vector3 forward = (to - from).normalized;
        Vector3 left = Vector3.Cross(Vector3.up, forward).normalized;
        Vector3 right = -left;
        /// Canditate bypass points placed on each side of the obstacle
        Vector3[] maybePoints =
        {
            obstaclePoint + forward * this.repathForwardDistance + left * this.repathOffsetDistance,
            obstaclePoint + forward * this.repathForwardDistance + right * this.repathOffsetDistance,
            obstaclePoint + left * this.repathOffsetDistance,
            obstaclePoint + right * this.repathOffsetDistance,
            obstaclePoint + -forward * this.repathForwardDistance + left * this.repathOffsetDistance,
            obstaclePoint + -forward * this.repathForwardDistance + right * this.repathOffsetDistance
        };
        /// Test each candidate and keep first valid one
        foreach (Vector3 c in maybePoints)
        {
            Vector3 mbPoint = c;
            /// Adjust candidate point to terrain height
            mbPoint.y = Terrain.activeTerrain.SampleHeight(mbPoint);

            if (PathValidator.IsMoveValid(new Vector2(from.x, from.z), new Vector2(mbPoint.x, mbPoint.z), this.pathVerificationStep, this.avoidanceZones, this.maxSlopeAngle, this.minSlopeAngle, this.carWidth, this.widthMarginFraction)) {
                bypassPoint = mbPoint;
                return (true, bypassPoint);
            }
        }
        return (false, Vector3.zero);
    }
    
    /// <summary>
    /// Attempts to generate a valid path from a list of points towards a target endpoint.
    /// The method iteratively builds a 2D spline through the current points and verifies its validity.
    /// If the path is invalid at any point, it tries to generate a bypass around the obstruction
    /// and inserts it into the path, repeating the process up to a maximum number of attempts.
    /// </summary>
    /// <param name="points">The list of waypoints defining the current path. New points may be inserted during bypass attempts.</param>
    /// <param name="end">The target endpoint the path is aiming to reach.</param>
    /// <param name="distanceToKeep">Minimum distance between points along the spline for discretization.</param>
    /// <param name="checkValidity">If true, the path is verified for obstacles; if false, the spline is built without validation.</param>
    /// <returns>
    /// True if a valid path is successfully generated (either immediately or after bypass adjustments), false otherwise.
    /// </returns>
    bool TryMakeValidPath(List<Vector3> points, Vector3 end, float distanceToKeep, bool checkValidity)
    {
        this.lastBypassTryPoint.Clear();
        // Attempt to build a valid path up to maxRepathAttempts times
        for (int attempt = 0; attempt < this.maxRepathAttempts; attempt++)
        {
            this.BuildSpline(points);
            this.SmoothSplineTangents(this.car.forward.normalized, this.car.forward.normalized, this.tangentSmoothFactor);

            if (!checkValidity)
            {
                Debug.Log("Spline built without validity check");
                this.MakeSpline3D(this.spline, this.discretizationStepsPerMeter, distanceToKeep);
                return true;
            }
            // Verify the validity of the current spline
            (bool res, Vector3 invalidPoint, Vector3 previousPoint) = this.IsPathValid(this.spline, this.checkStepInterval, this.pathVerificationStep);
            // Attempt to generate  a bypass if not valid
            if (!res)
            {
                (bool bypassSuccess, Vector3 bypass) = this.TryGenerateBypass(previousPoint, end, invalidPoint);
                if (!bypassSuccess)
                {
                    Debug.Log("Failed to generate bypass");
                    return false;
                }
                // Find the segment closest to the previous point before invalid one 
                int insertIndex = this.FindClosestSegment(points, previousPoint);
                if (insertIndex < 0)
                {
                    Debug.Log("Failed to locate segment for bypass insertion");
                    return false;
                }
                // Insert the bypass point into the path and record attempt
                points.Insert(insertIndex + 1, bypass);
                this.lastBypassTryPoint.Add(bypass);
                // Retry path generation with the new bypass point
                continue; 
            }
            // Path valid, make 3D spline and return success
            this.MakeSpline3D(this.spline, this.discretizationStepsPerMeter, distanceToKeep);
            Debug.Log($"Valid spline found after {attempt + 1} attempts");
            return true;
        }
        // Max attempts reached without valid path
        Debug.Log("Max repath attempts reached");
        return false;
    }
    
    /// <summary>
    /// Returns the index of the path segment closest to a given point.
    /// </summary>
    /// <param name="points">The ordered list of path points.</param>
    /// <param name="sample">The point used for the distance computation.</param>
    /// <returns>
    /// The index of the closest segment, or -1 if the path contains no segment.
    /// </returns>
    int FindClosestSegment(List<Vector3> points, Vector3 sample)
    {
        float bestDist = float.MaxValue;
        int bestIndex = -1;
        // Iterate over each segment
        for (int i = 0; i < points.Count - 1; i++)
        {
            Vector3 a = points[i];
            Vector3 b = points[i + 1];
            // Project the sample point onto the line defined by the segment
            // to estimate the closest point on this segment
            Vector3 closest = Vector3.Project(sample - a, b - a) + a;
            float dist = Vector3.Distance(sample, closest);
            // Save the segment with the minimal distance 
            if (dist < bestDist)
            {
                bestDist = dist;
                bestIndex = i;
            }
        }
        return bestIndex;
    }
    #endregion

    #region Utils

    /// <summary>
    /// Returns the current world position of the vehicle.
    /// </summary>
    /// <returns>
    /// The current position of the car.
    /// </returns>
    public Vector3 GetCurrentPosition()
    {
        return this.car.position; // already be a copy (not reference)
    }

    /// <summary>
    /// Returns the current world rotation of the vehicle.
    /// </summary>
    /// <returns>
    /// The current rotation of the car.
    /// </returns>
    public Quaternion GetCurrentRotation()
    {
        return this.car.rotation; // already be a copy (not reference)
    }
    #endregion

    #region Visualization
    void OnDrawGizmos()
    {
        if (this.spline3D == null || this.spline3D.Count < 2)
            return;

        Gizmos.color = Color.cyan;

        // 3DSpline
        int stepsPerMeter = 10;
        Vector3 prev = this.spline3D.EvaluatePosition(0f);

        int steps = (int)(stepsPerMeter * this.spline3D.GetLength());

        for (int i = 1; i <= steps; i++)
        {
            float t = i / (float)steps;
            Vector3 curr = this.spline3D.EvaluatePosition(t);
            Gizmos.DrawLine(prev, curr);
            prev = curr;
        }

        if (this.debugView)
        {
            // Original spline
            Gizmos.color = Color.yellow;
            prev = this.spline.EvaluatePosition(0f);
            prev.y = 19.5f; //lift original spline for better visibility
            int steps2D = (int)(stepsPerMeter * this.spline.GetLength());
            for (int i = 1; i <= steps2D; i++)
            {
                float t = i / (float)steps2D;
                Vector3 curr = this.spline.EvaluatePosition(t);
                curr.y = 19.5f; //lift original spline for better visibility
                Gizmos.DrawLine(prev, curr);
                prev = curr;
            }

            // Control points FROM ORIGINAL SPLINE
            for (int i = 0; i < this.spline.Count; i++)
            {
                Vector3 pos = this.spline[i].Position;
                pos.y = 19.5f;

                Gizmos.color = Color.red;
                Gizmos.DrawSphere(pos, 0.15f);

                Vector3 tangentInWorld = pos + new Vector3(this.spline[i].TangentIn.x, this.spline[i].TangentIn.y, this.spline[i].TangentIn.z);
                Vector3 tangentOutWorld = pos + new Vector3(this.spline[i].TangentOut.x, this.spline[i].TangentOut.y, this.spline[i].TangentOut.z);

                Gizmos.color = Color.green;

                Gizmos.DrawLine(pos, tangentInWorld);
                Gizmos.DrawLine(pos, tangentOutWorld);
            }

            // Control points on 3D spline
            for (int i = 0; i < this.spline.Count; i++)
            {
                Vector3 pos = this.spline[i].Position;
                pos.y = Terrain.activeTerrain.SampleHeight(pos);

                Gizmos.color = Color.red;
                Gizmos.DrawSphere(pos, 0.3f);
            }

            Gizmos.color = Color.blue;
            foreach (Vector3 p in this.lastBypassTryPoint)
            {
                Gizmos.DrawSphere(p, 0.3f);
            }

            if (this.avoidanceZones == null)
                return;

            // Avoidance zones
            Gizmos.color = Color.red; 
            foreach (AvoidanceZone zone in this.avoidanceZones)
            {
                float height = 30f;
                float gridStep = 5f;

                Vector3 min = new Vector3(
                    zone.center.x - zone.width * 0.5f,
                    0f,
                    zone.center.z - zone.depth * 0.5f
                );

                Vector3 max = new Vector3(
                    zone.center.x + zone.width * 0.5f,
                    height,
                    zone.center.z + zone.depth * 0.5f
                );

                // Vertical grid lines
                for (float x = min.x; x <= max.x; x += gridStep)
                {
                    for (float z = min.z; z <= max.z; z += gridStep)
                    {
                        Gizmos.DrawLine(new Vector3(x, 0, z), new Vector3(x, height, z));
                    }
                }

                // Horizontal layers
                for (float y = 0; y <= height; y += gridStep)
                {
                    Gizmos.DrawWireCube(
                        new Vector3(zone.center.x, y, zone.center.z),
                        new Vector3(zone.width, 0f, zone.depth)
                    );
                }
            }

            // Explored points
            Vector3 elevationOffset = new Vector3(0, 25f, 0);

            Gizmos.color = Color.magenta;
            foreach (TreeNode node in this.exploredPoints)
            {
                Gizmos.DrawSphere(node.Position + elevationOffset, 0.1f);
                if (node.Parent != null)
                {
                    Gizmos.DrawLine(node.Position + elevationOffset, node.Parent.Position + elevationOffset);
                }
            }

            // Path points
            elevationOffset = new Vector3(0, 25.1f, 0);
            Gizmos.color = Color.blue;
            for (int i = 0; i < this.pathPoints.Count; i++)
            {
                Gizmos.DrawSphere(this.pathPoints[i] + elevationOffset, 0.1f);
                if (i > 0)
                {
                    Gizmos.DrawLine(this.pathPoints[i - 1] + elevationOffset, this.pathPoints[i] + elevationOffset);
                }
            }

        }
    }
    #endregion
}