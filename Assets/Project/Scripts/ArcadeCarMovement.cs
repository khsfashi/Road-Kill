using UnityEngine;

public class ArcadeCarMovement : Movement
{
    // Car Info
    [SerializeField] private Rigidbody carRigidBody;
    [SerializeField] private GameObject[] wheels = new GameObject[4];
    [SerializeField] private GameObject[] frontWheelsParents = new GameObject[2];

    // Speed Physics Setting Values
    [SerializeField] private Transform accelerationPoint;
    [SerializeField] private float accelerationVal = 15f;
    [SerializeField] private float decelerationVal = 10f;
    [SerializeField] private float maxSpeed = 100f;
    [SerializeField] private float steerStrength = 15f;
    [SerializeField] private AnimationCurve turnCurve;
    [SerializeField] private float dragCoefficient = 1f;

    // Spin Wheel
    [SerializeField] private float maxSteeringAngle = 30f;
    [SerializeField] private float wheelRotationSpeed = 3000f;

    // Suspension
    [SerializeField] private Suspension suspensionCalculator;

    // Speed Physics Member Values
    private Vector3 _currentLocalVelocity = Vector3.zero;
    private float _velocityRatio = 0f;

    // Input Values
    private float _moveInput = 0f;
    private float _steerInput = 0f;

    // Ensure Movable Values
    private bool _isGrounded = false;

    // Spin Wheel Values
    private float _currentSteerAngle = 0.0f;
    private Vector3 _wheelEulerAngles = Vector3.zero;

    public override void UpdateMovement()
    {
        CalculateVelocity();
        if(_isGrounded)
        {
            Acceleration();
            Deceleration();
            Turn();
            SidewaysDrag();
        }
        SpinWheels();
    }

    public override void UpdateMovementState()
    {
        suspensionCalculator.CalculateSuspension();
        CheckGrounded();
    }

    public override void UpdateDirection(Vector3 direction)
    {
        _moveInput = direction.y;
        _steerInput = direction.x;
    }

    public override void UpdateDirection(float x, float y, float z)
    {
        _moveInput = y;
        _steerInput = x;
    }

    public override bool CanMove()
    {
        return _isGrounded;
    }

    public override Vector3 GetLocalVelocity()
    {
        return _currentLocalVelocity;
    }

    private void Acceleration()
    {
        if (_currentLocalVelocity.z < maxSpeed)
        {
            carRigidBody.AddForceAtPosition(accelerationVal * _moveInput * ProjectOnGround(transform.forward), accelerationPoint.position, ForceMode.Acceleration);
        }
    }

    private void Deceleration()
    {
        carRigidBody.AddForceAtPosition(decelerationVal * Mathf.Abs(_velocityRatio) * -ProjectOnGround(transform.forward), accelerationPoint.position, ForceMode.Acceleration);
    }

    private void Turn()
    {
        carRigidBody.AddTorque(steerStrength * _steerInput * turnCurve.Evaluate(Mathf.Abs(_velocityRatio)) * Mathf.Sign(_velocityRatio) * transform.up, ForceMode.Acceleration);
    }

    private void SidewaysDrag()
    {
        float currentSidewaysSpeed = _currentLocalVelocity.x;
        float dragMagnitude = -currentSidewaysSpeed * dragCoefficient;

        Vector3 dragForce = transform.right * dragMagnitude;
        carRigidBody.AddForceAtPosition(dragForce, carRigidBody.worldCenterOfMass, ForceMode.Acceleration);
    }

    private Vector3 ProjectOnGround(Vector3 vector)
    {
        var groundNormal = suspensionCalculator.GroundNormal;
        Vector3 projected = vector - Vector3.Dot(vector, groundNormal) * groundNormal;
        return projected.normalized;
    }

    private void CalculateVelocity()
    {
        _currentLocalVelocity = transform.InverseTransformDirection(carRigidBody.linearVelocity);
        _velocityRatio = _currentLocalVelocity.z / maxSpeed;
    }

    private void CheckGrounded()
    {
        var isGroundedWheels = suspensionCalculator.IsGroundedWheels;
        int groundedWheels = 0;
        for (int i = 0; i < isGroundedWheels.Length; ++i)
        {
            groundedWheels += isGroundedWheels[i];
        }

        _isGrounded = groundedWheels > 1 ? true : false;
    }

    private void SpinWheels()
    {
        float targetSteeringAngle = maxSteeringAngle * _steerInput;
        float steerSmoothSpeed = 5.0f;
        _currentSteerAngle = Mathf.Lerp(_currentSteerAngle, targetSteeringAngle, steerSmoothSpeed * Time.deltaTime);

        for (int i = 0; i < wheels.Length; ++i)
        {
            if (i < 2)
            {
                wheels[i].transform.Rotate(Vector3.right, wheelRotationSpeed * _velocityRatio * Time.deltaTime, Space.Self);
                // 원래 벡터 생성해서 쓰다가 new 매프레임 하는 거 좀 성능 문제 생길 것 같아서 멤버 변수 통해서 바꿈
                _wheelEulerAngles = frontWheelsParents[i].transform.localEulerAngles;
                _wheelEulerAngles.y = _currentSteerAngle;
                frontWheelsParents[i].transform.localEulerAngles = _wheelEulerAngles;
            }
            else
            {
                wheels[i].transform.Rotate(Vector3.right, wheelRotationSpeed * _moveInput * Time.deltaTime, Space.Self);
                wheels[i].transform.Rotate(Vector3.right, wheelRotationSpeed * _velocityRatio * Time.deltaTime, Space.Self);
            }
        }
    }
}
