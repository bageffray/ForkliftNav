using System.Collections;
using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Orchestrates complete grab-and-drop mission involving a vehicle and a robotic arm.
/// The mission includes navigation to a target, object pickup, return to origin,
/// and object release, with optional error recovery.
/// </summary>
public class MissionHandler : MonoBehaviour {

    [Header("Run Mission")]
    public bool runMission = false;

    [Space]
    [Header("Mission Settings")]
    public Vector3 targetPosition;
    public List<Vector3> waypoints;
    public float distanceToKeep = 2f;

    private float delayBetweenActions = 1f;

    [Space]
    [Header("Controllers")]
    public ArmController armController;
    public CarController carController;

    // Arm positions
    private Vector3 defaultArmPosition = new Vector3(0, 1, 0);
    private Vector3 dropArmPosition = new Vector3(0, 1f, 1f);

    // Runtime variables
    private Vector3 originPosition;
    private Quaternion originRotation;
    private bool errorOccurred = false;

    private bool previousRunMission;

    void Start() {

        this.previousRunMission = this.runMission;
    }

    void Update() {
        if (this.runMission && !this.previousRunMission) {
            this.previousRunMission = this.runMission;
            this.StartGrabMission(this.targetPosition, this.carController.GetCurrentRotation(), this.waypoints);
        }
        this.previousRunMission = this.runMission;
    }

    /// <summary>
    /// Starts a grab mission by chaining a sequence of vehicle and arm actions.
    /// The sequence is executed asynchronously using callbacks and delayed actions.
    /// </summary>
    /// <param name="targetPosition">World position of the object to grab.</param>
    /// <param name="targetRotation">Desired vehicle rotation at the target.</param>
    /// <param name="waypoints">Intermediate navigation points for vehicle movement.</param>
    void StartGrabMission(Vector3 targetPosition, Quaternion targetRotation, List<Vector3> waypoints) {
        this.errorOccurred = false;

        Debug.Log("Starting mission");
        // Save origin position and rotation for return
        this.originPosition = this.carController.GetCurrentPosition();
        this.originRotation = this.carController.GetCurrentRotation();
        // Define mission sequence
        // Step 7 : Move arm back to base position
        System.Action basePos = () =>
        {
            this.Delay(this.delayBetweenActions, () =>
            {
                System.Action onComplete = this.OnMissionComplete;

                Debug.Log("Moving arm to base position");
                this.MoveArmShoulderReference(this.defaultArmPosition, false, false, onComplete,
                () => this.OnError(false, onComplete));
            });
        };
        //Step 6 : Release object
        System.Action releaseObject = () =>
        {
            this.Delay(this.delayBetweenActions, () =>
            {
                System.Action onComplete = basePos;

                Debug.Log("Releasing Object");
                this.ReleaseObject(onComplete, () => this.OnError(false, onComplete));
            });
        };
        // Step 5 : Move arm to drop position
        System.Action armToDrop = () => {
            this.Delay(this.delayBetweenActions, () => {

                System.Action onComplete = releaseObject;

                Debug.Log("Moving arm to drop position");
                this.MoveArmShoulderReference(this.dropArmPosition, false, false, onComplete, () => this.OnError(false, onComplete));
            });
        };
       // Step 4 : Move back to origin position
        System.Action goToOrigin = () => {
            this.Delay(this.delayBetweenActions, () => {

                System.Action onComplete = armToDrop;

                Debug.Log("Moving back to origin");
                List<Vector3> inverted = new List<Vector3>(waypoints);
                inverted.Reverse();
                this.MoveCar(this.originPosition, this.originRotation, inverted, 0f, onComplete, () => this.OnError(true, null)); // going all the way
            });
        };
        // Step 3 : Put arm up after grab
        System.Action putArmUp = () => {
            this.Delay(this.delayBetweenActions, () => {

                System.Action onComplete = goToOrigin;

                Debug.Log("Moving arm up");
                this.MoveArmShoulderReference(this.defaultArmPosition, false, false, onComplete, () => this.OnError(false, onComplete));
            });
        };
        // Step 2 : Move hand to target postion to grab object
        System.Action handToTarget = () => {
            this.Delay(this.delayBetweenActions, () => {
                System.Action onComplete = putArmUp;

                Debug.Log("Moving arm to target");
                this.MoveArmWorldReference(targetPosition, true, true, onComplete, () => this.OnError(false, onComplete));
            });
        };

        // Step 1 : Move car to target position
        Debug.Log("Moving to target");
        this.MoveCar(targetPosition, targetRotation, waypoints, this.distanceToKeep, handToTarget, () => this.OnError(true, null));
    }


    /// <summary>
    /// Commands the vehicle to move to a target position following waypoints.
    /// </summary>
    void MoveCar(Vector3 targetPosition, Quaternion targetRotation, List<Vector3> waypoints, float distanceToKeep, System.Action onComplete, System.Action onError) {
        this.carController.GoToGrab(targetPosition, targetRotation, waypoints, distanceToKeep, onComplete, onError);
    }

    /// <summary>
    /// Moves the robotic arm to a world-space target position.
    /// </summary>
    void MoveArmWorldReference(Vector3 targetPosition, bool grab, bool stopOnGrab, System.Action onComplete, System.Action onError) {
        this.armController.GoToWorldReference(targetPosition, grab, stopOnGrab, onComplete, onError);
    }

    /// <summary>
    /// Moves the robotic arm using shoulder-relative coordinates.
    /// </summary>
    void MoveArmShoulderReference(Vector3 targetShoulderPos, bool grab, bool stopOnGrab, System.Action onComplete, System.Action onError) {
        this.armController.GoToShoulderReference(targetShoulderPos, grab, stopOnGrab, onComplete, onError);
    }

    /// <summary>
    /// Releases the currently held object.
    /// </summary>
    void ReleaseObject(System.Action onComplete, System.Action onError)
    {
        this.armController.Release(onComplete, onError);
    }

    /// <summary>
    /// Handles mission errors and decides whether to stop or continue.
    /// </summary>
    /// <param name="shouldStop">If true, the mission is aborted.</param>
    /// <param name="onComplete">Fallback action if recovery is allowed.</param>
    void OnError(bool shouldStop, System.Action onComplete) {
        Debug.Log("Mission error");
        this.errorOccurred = true;

        if (shouldStop) {
            this.runMission = false;
            Debug.Log("Stopping the mission.");
        }
        else {
            onComplete?.Invoke();
        }
    }

    /// <summary>
    /// Called when the entire mission sequence completes successfully.
    /// </summary>
    void OnMissionComplete() {
        if (this.errorOccurred) {
            Debug.Log("Mission ended with errors.");
        }
        else {
            Debug.Log("Mission completed successfully.");
        }
        this.runMission = false;
    }

    /// <summary>
    /// Executes an action after a delay using a coroutine.
    /// </summary>
    void Delay(float seconds, System.Action action)
    {
        this.StartCoroutine(this.DelayRoutine(seconds, action));
    }

    IEnumerator DelayRoutine(float seconds, System.Action action)
    {
        yield return new WaitForSeconds(seconds);
        action?.Invoke();
    }
}