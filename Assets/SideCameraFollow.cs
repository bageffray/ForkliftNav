using UnityEngine;

[RequireComponent(typeof(Camera))]
public class SideCameraFollow : MonoBehaviour
{
    public Transform target;
    public Vector3 sideOffset = new Vector3(6f, 5f, 5f);
    public float followSpeed = 10f;
    public float lookAtHeightOffset = 2f;
    public bool invertSide = false;
    public float forwardBias = 3f;

    void Start()
    {
        if (transform.position == Vector3.zero && target != null)
        {
            Vector3 desired = ComputeDesiredPosition();
            transform.position = desired;
            transform.LookAt(TargetLookPoint());
        }
    }

    void LateUpdate()
    {
        if (target == null) {
            return;
        }
        
        Vector3 desired = ComputeDesiredPosition();
        transform.position = Vector3.Lerp(transform.position, desired, Mathf.Clamp01(followSpeed * Time.deltaTime));
        transform.LookAt(TargetLookPoint());
    }

    Vector3 ComputeDesiredPosition()
    {
        float sideSign = invertSide ? -1f : 1f;
        Vector3 side = target.right * (sideOffset.x * sideSign);
        Vector3 up = Vector3.up * sideOffset.y;
        Vector3 forward = target.forward * sideOffset.z;
        return target.position + side + up + forward;
    }

    Vector3 TargetLookPoint()
    {
        return target.position + Vector3.up * lookAtHeightOffset + target.forward * forwardBias;
    }
}