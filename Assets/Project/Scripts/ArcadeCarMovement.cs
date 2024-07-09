using UnityEngine;

public class ArcadeCarMovement : Movement
{
    [Header("Car Info")]
    [SerializeField] private Rigidbody carRigidBody;
    [SerializeField] private GameObject[] wheels = new GameObject[4];
    [SerializeField] private GameObject[] frontWheelsParents = new GameObject[2];
    [SerializeField] private Transform[] rayTransforms = new Transform[4];
    [SerializeField] private Transform accelerationPoint;

    [Header("Car Movement Setting Data")]
    [SerializeField] private ArcadeCarMovementSettings settings;

    // Speed Physics Setting Values
    private float _accelerationVal = 15f;
    private float _decelerationVal = 10f;
    private float _maxSpeed = 100f;
    private float _steerStrength = 15f;
    private AnimationCurve _turnCurve;
    private float _dragCoefficient = 1f;

    // Spin Wheel
    private float _maxSteeringAngle = 30f;
    private float _wheelRotationSpeed = 3000f;

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

    // Suspension
    private Suspension _suspensionCalculator;

    private void Start()
    {
        // Insurance
        carRigidBody = GetComponent<Rigidbody>();

        _suspensionCalculator = new Suspension(settings, carRigidBody, wheels, rayTransforms);

        _accelerationVal = settings.accelerationVal;
        _decelerationVal = settings.decelerationVal;
        _maxSpeed = settings.maxSpeed;
        _steerStrength = settings.steerStrength;
        _turnCurve = settings.turnCurve;
        _dragCoefficient = settings.dragCoefficient;

        _maxSteeringAngle = settings.maxSteeringAngle;
        _wheelRotationSpeed = settings.wheelRotationSpeed;
    }

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
        _suspensionCalculator.CalculateSuspension();
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
        if (_currentLocalVelocity.z < _maxSpeed)
        {
            carRigidBody.AddForceAtPosition(_accelerationVal * _moveInput * ProjectOnGround(transform.forward), accelerationPoint.position, ForceMode.Acceleration);
        }
    }

    private void Deceleration()
    {
        carRigidBody.AddForceAtPosition(_decelerationVal * Mathf.Abs(_velocityRatio) * -ProjectOnGround(transform.forward), accelerationPoint.position, ForceMode.Acceleration);
    }

    private void Turn()
    {
        carRigidBody.AddTorque(_steerStrength * _steerInput * _turnCurve.Evaluate(Mathf.Abs(_velocityRatio)) * Mathf.Sign(_velocityRatio) * transform.up, ForceMode.Acceleration);
    }

    private void SidewaysDrag()
    {
        float currentSidewaysSpeed = _currentLocalVelocity.x;
        float dragMagnitude = -currentSidewaysSpeed * _dragCoefficient;

        Vector3 dragForce = transform.right * dragMagnitude;
        carRigidBody.AddForceAtPosition(dragForce, carRigidBody.worldCenterOfMass, ForceMode.Acceleration);
    }

    private Vector3 ProjectOnGround(Vector3 vector)
    {
        var groundNormal = _suspensionCalculator.GroundNormal;
        Vector3 projected = vector - Vector3.Dot(vector, groundNormal) * groundNormal;
        return projected.normalized;
    }

    private void CalculateVelocity()
    {
        _currentLocalVelocity = transform.InverseTransformDirection(carRigidBody.linearVelocity);
        _velocityRatio = _currentLocalVelocity.z / _maxSpeed;
    }

    private void CheckGrounded()
    {
        var isGroundedWheels = _suspensionCalculator.IsGroundedWheels;
        int groundedWheels = 0;
        for (int i = 0; i < isGroundedWheels.Length; ++i)
        {
            groundedWheels += isGroundedWheels[i];
        }

        _isGrounded = groundedWheels > 1 ? true : false;
    }

    private void SpinWheels()
    {
        float targetSteeringAngle = _maxSteeringAngle * _steerInput;
        float steerSmoothSpeed = 5.0f;
        _currentSteerAngle = Mathf.Lerp(_currentSteerAngle, targetSteeringAngle, steerSmoothSpeed * Time.deltaTime);

        for (int i = 0; i < wheels.Length; ++i)
        {
            if (i < 2)
            {
                wheels[i].transform.Rotate(Vector3.right, _wheelRotationSpeed * _velocityRatio * Time.deltaTime, Space.Self);
                // 원래 벡터 생성해서 쓰다가 new 매프레임 하는 거 좀 성능 문제 생길 것 같아서 멤버 변수 통해서 바꿈
                _wheelEulerAngles = frontWheelsParents[i].transform.localEulerAngles;
                _wheelEulerAngles.y = _currentSteerAngle;
                frontWheelsParents[i].transform.localEulerAngles = _wheelEulerAngles;
            }
            else
            {
                wheels[i].transform.Rotate(Vector3.right, _wheelRotationSpeed * _moveInput * Time.deltaTime, Space.Self);
                wheels[i].transform.Rotate(Vector3.right, _wheelRotationSpeed * _velocityRatio * Time.deltaTime, Space.Self);
            }
        }
    }
}
