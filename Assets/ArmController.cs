using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Controls a simple articulated robotic arm mounted on a vehicle.
/// The arm supports inverse kinematics, object grabbing, and smooth joint rotation
/// toward target positions in either world or shoulder-relative coordinates.
/// </summary>
public class ArmController : MonoBehaviour {

    private const double Rad2Deg = 180.0 / System.Math.PI;
    private const double Deg2Rad = System.Math.PI / 180.0;

    [Space]
    [Header("Movement Settings")]
    // Public
    public float rotationSpeed = 90f;

    // Private
    private float maxAngleDiff = 0.1f;

    [Space]
    [Header("References")]
    // Public
    public Transform elbow;
    public Transform shoulder;
    public Transform hand;
    public Transform car;

    // Grab
    private bool stopOnGrab;
    private bool grab = false;
    private float grabRange;
    private bool isHolding = false;
    private Rigidbody heldrb; 

    // Target
    private Vector3 targetPos;
    private Vector3 shoulderTargetRotation;
    private Vector3 elbowTargetRotation;
    private Vector3 targetSelfPos;

    // Runtime variables
    private System.Action onComplete;
    
    private double forearmLength;
    private double upperArmLength;

    private bool moving = false;

    // Start is called before the first frame update
    void Start() {
        this.forearmLength = this.Distance(this.elbow.position, this.hand.position);
        this.upperArmLength = this.Distance(this.shoulder.position, this.elbow.position);

        // Set grabRange to hand size + margin
        BoxCollider box = this.hand.GetComponent<BoxCollider>();
        if (box != null)
        {
            Vector3 size = box.size;
            Vector3 scale = this.hand.lossyScale;
            Vector3 center = box.center;
            Vector3 handWorld = this.hand.position;
            float maxDist = 0f;
            // check the distance to 8 corners of the box to set grab range to the farthest one
            for (int i = 0; i < 8; i++)
            {
                Vector3 localCorner = center + new Vector3(
                    ((i & 1) == 0 ? -0.5f : 0.5f) * size.x,
                    ((i & 2) == 0 ? -0.5f : 0.5f) * size.y,
                    ((i & 4) == 0 ? -0.5f : 0.5f) * size.z
                );
                Vector3 worldCorner = this.hand.TransformPoint(localCorner);
                float dist = Vector3.Distance(handWorld, worldCorner);
                if (dist > maxDist) maxDist = dist;
            }
            this.grabRange = maxDist + 0.05f;
        }

        // Set target positions to current positions
        this.targetSelfPos = this.WorldToShoulderReference(this.hand.position, this.shoulder.position, this.car.rotation);
        this.targetPos = this.ShoulderToWorldReference(this.targetSelfPos, this.shoulder.position, this.car.rotation);
    }

    // Update is called once per frame
    void Update() {
        if (!this.isHolding && this.grab)
        {   
            this.TryGrab();                   
        }

        if (!this.moving) {
            return;
        }

        this.TargetControlRotateTowards();
    }

    /// <summary>
    /// Attempts to grab a nearby object within grab range.
    /// </summary>
    public void TryGrab()
    {       
        if (this.isHolding) {
            return;
        }
        // Check for colliders within grab range
        Collider[] colliders = Physics.OverlapSphere(this.hand.position, this.grabRange);
        foreach (Collider c in colliders)
        {
            if (c == null) {
                continue;
            }
            // Ignore colliders that are part of the car
            if (!c.transform.IsChildOf(this.car))
            {
                Rigidbody rb = c.attachedRigidbody;
                // If the collider has a rigidbody, grab the object
                if (rb != null)
                {
                    Debug.Log("Grabbing object: " + c.name);
                    this.Grab(c);  
                    return;                  
                }
            }
        }
    }

    /// <summary>
    /// Commands the arm to move toward a world-space target position.
    /// </summary>
    public void GoToWorldReference(Vector3 pos, bool grab, bool stopOnGrab, System.Action onComplete, System.Action onError) {
        this.stopOnGrab = stopOnGrab;
        this.grab = grab;
        this.onComplete = onComplete;
        this.targetPos = pos;

        // Convert target position to shoulder-relative coordinates
        this.targetSelfPos = this.WorldToShoulderReference(this.targetPos, this.shoulder.position, this.car.rotation);
        // Compute joint angles for target position
        this.MoveArmToShoulderReference(this.targetSelfPos, onError);

        this.moving = true;
    }

    /// <summary>
    /// Commands the arm to move toward a shoulder-relative target position.
    /// </summary>
    public void GoToShoulderReference(Vector3 pos, bool grab, bool stopOnGrab, System.Action onComplete, System.Action onError) {
        this.stopOnGrab = stopOnGrab;
        this.grab = grab;
        this.onComplete = onComplete;
        this.targetSelfPos = pos;
        // Convert target position to world-space coordinates
        this.targetPos = this.ShoulderToWorldReference(this.targetSelfPos, this.shoulder.position, this.car.rotation);
        // Compute joint angles for target position
        this.MoveArmToShoulderReference(this.targetSelfPos, onError);

        this.moving = true;
    }

    /// <summary>
    /// Calculates the Euclidean distance between two 3D points.
    /// </summary>
    double Distance(Vector3 a, Vector3 b) {
        return System.Math.Sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
    }

    /// <summary>
    /// Computes target joint rotations using geometric inverse kinematics.
    /// </summary>
    void MoveArmToShoulderReference(Vector3 pos, System.Action onError) {
        double L = System.Math.Sqrt(pos.x * pos.x + pos.y * pos.y + pos.z * pos.z);
        // Check reachability
        if (this.upperArmLength + this.forearmLength < L) {
            Debug.Log("Target position is out of reach.");
            onError?.Invoke();
            return;
        }
        // Inverse kinematics calculations
        double elevation = System.Math.Acos(pos.y / L);
        double theta1 = elevation - System.Math.Acos((this.upperArmLength*this.upperArmLength + L*L - this.forearmLength*this.forearmLength) / (2*this.upperArmLength*L));
        double theta2 = System.Math.PI - System.Math.Acos((this.upperArmLength * this.upperArmLength + this.forearmLength * this.forearmLength - L * L) / (2 * this.upperArmLength * this.forearmLength));
        
        double azimuth = System.Math.Atan2(pos.x, pos.z);

        // Set target rotations
        this.shoulderTargetRotation = new Vector3((float)(theta1 * ArmController.Rad2Deg), (float) (azimuth * ArmController.Rad2Deg), 0);
        this.elbowTargetRotation = new Vector3((float)(theta2 * ArmController.Rad2Deg), 0, 0);
    }

    /// <summary>
    /// Converts a position expressed in the shoulder's local reference frame
    /// into a world-space position.
    /// </summary>
    /// <param name="selfReferencePos">
    /// Position relative to the shoulder (local space).
    /// </param>
    /// <param name="shoulderReferencePosition">
    /// World-space position of the shoulder.
    /// </param>
    /// <param name="shoulderRotation">
    /// World-space rotation of the shoulder (or its parent reference).
    /// </param>
    /// <returns>
    /// The corresponding position in world space.
    /// </returns>
    Vector3 ShoulderToWorldReference(Vector3 selfReferencePos, Vector3 shoulderReferencePosition, Quaternion shoulderRotation) {
        return shoulderReferencePosition + shoulderRotation * selfReferencePos;
    }

    /// <summary>
    /// Converts a world-space position into the shoulder's local reference frame.
    /// </summary>
    /// <param name="worldReferencePos">
    /// Position expressed in world space.
    /// </param>
    /// <param name="shoulderReferencePosition">
    /// World-space position of the shoulder.
    /// </param>
    /// <param name="shoulderRotation">
    /// World-space rotation of the shoulder (or its parent reference).
    /// </param>
    /// <returns>
    /// The corresponding position relative to the shoulder (local space).
    /// </returns>
    Vector3 WorldToShoulderReference(Vector3 worldReferencePos, Vector3 shoulderReferencePosition, Quaternion shoulderRotation) {
        return Quaternion.Inverse(shoulderRotation) * (worldReferencePos - shoulderReferencePosition);
    }
    
    /// <summary>
    /// Smoothly rotates joints toward their target orientations.
    /// </summary>
    void TargetControlRotateTowards() {
        // Target joints rotations as quaternions
        Quaternion shoulderTargetQuaternion = Quaternion.Euler(this.shoulderTargetRotation);
        Quaternion elbowTargetQuaternion = Quaternion.Euler(this.elbowTargetRotation);
        // Rotate shoulder joint toward targets
        this.shoulder.localRotation = Quaternion.RotateTowards(this.shoulder.localRotation, shoulderTargetQuaternion, (float)(this.rotationSpeed * Time.deltaTime));
        // Rotate elbow after shoulder is close to target
        this.elbow.localRotation = Quaternion.RotateTowards(this.elbow.localRotation, elbowTargetQuaternion, (float)(this.rotationSpeed * Time.deltaTime));
        // Check if both joints have reached target 
        // Stop if an object is grabbed and stopOnGrab true
        if ((Quaternion.Angle(this.shoulder.localRotation, shoulderTargetQuaternion) < this.maxAngleDiff &&
            Quaternion.Angle(this.elbow.localRotation, elbowTargetQuaternion) < this.maxAngleDiff) ||
            (this.isHolding && this.stopOnGrab)) {
            // Movement complete
            this.moving = false;
            this.onComplete?.Invoke();
        }
    }
    
    /// <summary>
    /// Attaches an object to the hand and disables its physics interactions.
    /// The object will follow the hand while being held.
    /// </summary>
    /// <param name="collider">The collider of the object to grab.</param>
    public void Grab(Collider collider)
    {
        // Rigidbody of the object to grab
        Rigidbody rb = collider.attachedRigidbody;
        if (rb == null || this.isHolding)
        {
            Debug.Log("Cannot grab object because no rigidbody has been found or already holding something.");
            return;
        }

        // Stop object's movement
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        rb.isKinematic = true;
        rb.useGravity = false;

        // Ignore collisions with player

        // All the colliders of the object (needed for complex objects with multiple colliders)
        Collider[] objectCols = rb.GetComponentsInChildren<Collider>();

        // All the colliders of the car
        Collider[] carCols = this.hand.GetComponentsInParent<Collider>();

        foreach (var objCol in objectCols)
        {
            foreach (var carCol in carCols)
            {
                // Don't register collisions between the object and the car
                Physics.IgnoreCollision(objCol, carCol, true);
            }
        }

        // Preserve grab offset and world scale
        Transform t = rb.transform;
        Vector3 originalWorldScale = t.lossyScale;
        t.SetParent(this.hand.transform, true); // keep world position
        t.localRotation = Quaternion.identity;

        // Calculate new localScale to preserve world scale
        Vector3 parentScale = t.parent.lossyScale;
        t.localScale = new Vector3(
            originalWorldScale.x / parentScale.x,
            originalWorldScale.y / parentScale.y,
            originalWorldScale.z / parentScale.z
        );

        this.heldrb = rb;
        this.isHolding = true;
    }

    /// <summary>
    /// Releases the currently held object from the hand, restoring physics and collisions.
    /// </summary>
    /// <param name="onComplete">Callback invoked when the release is finished successfully.</param>
    /// <param name="onError">Callback invoked if no object was held to release.</param>
    public void Release(System.Action onComplete, System.Action onError)
    {
        // if no object held, call onError
        if (this.heldrb == null)
        {
            Debug.Log("No object to release.");
            onError?.Invoke();
            return;
        }

        Debug.Log("Releasing object");

        // All the colliders of the object (needed for complex objects with multiple colliders)
        Collider[] objectCols = this.heldrb.GetComponentsInChildren<Collider>();

        // All the colliders of the car
        Collider[] carCols = this.hand.GetComponentsInParent<Collider>();

        foreach (var objCol in objectCols)
        {
            foreach (var carCol in carCols)
            {
                // Restore collisions
                Physics.IgnoreCollision(objCol, carCol, false);
            }
        }

        // Detach from hand
        this.heldrb.transform.SetParent(null);
        // restore physics behavior
        this.heldrb.isKinematic = false;
        this.heldrb.useGravity = true;
        // clear reference and state
        this.heldrb = null;
        this.isHolding = false;
        this.grab = false;

        onComplete?.Invoke();
    }
 
    void OnDrawGizmos()
    {
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(this.hand.position, this.grabRange);
    }
}
