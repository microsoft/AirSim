using UnityEngine;

public class SmoothFollow : MonoBehaviour {
    public bool shouldRotate = true;

    // The target we are following
    public Transform target;

    private float wantedRotationAngle;
    private float currentRotationAngle;

    private Quaternion currentRotation;

    private Vector3 offsetPostion, finalPosition;

    private void Start() {
        offsetPostion = new Vector3(0, 5, -10);
    }

    private void LateUpdate() {
        if (!target) {
            return;
        }

        // Calculate the current rotation angles
        wantedRotationAngle = target.eulerAngles.y;
        currentRotationAngle = transform.eulerAngles.y;

        // Damp the rotation around the y-axis
        currentRotationAngle = Mathf.LerpAngle(currentRotationAngle, wantedRotationAngle, 0.3f);

        // Convert the angle into a rotation
        currentRotation = Quaternion.Euler(0, currentRotationAngle, 0);

        // Set the position of the camera on the x-z plane to:
        // distance meters behind the target
        transform.position = target.position + (currentRotation * offsetPostion);

        // Always look at the target
        transform.LookAt(target);
    }
}