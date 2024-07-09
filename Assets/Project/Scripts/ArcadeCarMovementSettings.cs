using UnityEngine;

[CreateAssetMenu(fileName = "ArcadeCarMovementSettings", menuName = "Scriptable Objects/ArcadeCarMovementSettings")]
public class ArcadeCarMovementSettings : ScriptableObject
{
    public LayerMask movableMask;
    public float wheelRadius;

    // Speed Physics Setting Values
    public float accelerationVal = 15f;
    public float decelerationVal = 10f;
    public float maxSpeed = 100f;
    public float steerStrength = 15f;
    public AnimationCurve turnCurve;
    public float dragCoefficient = 1f;

    // Spin Wheel
    public float maxSteeringAngle = 30f;
    public float wheelRotationSpeed = 3000f;

    // Suspension Physics Setting Values
    public float extraLength;
    public float carFlexibillity;
    public float springStiffness;
    public float damperStiffness;
}
