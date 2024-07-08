using UnityEngine;

public abstract class Movement : MonoBehaviour
{
    public abstract void UpdateMovement();
    public abstract void UpdateMovementState();
    public abstract void UpdateDirection(Vector3 direction);
    public abstract void UpdateDirection(float x, float y, float z);
    public abstract bool CanMove();
    public abstract Vector3 GetLocalVelocity();
}
