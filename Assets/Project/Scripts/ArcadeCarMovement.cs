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
    private bool _isDrift = false;

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
        Physics.gravity = new Vector3(0, -19.62f, 0);
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
            Friction();
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
    public override void SetSprint(bool value)
    {
        _isDrift = value;
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
        if (Mathf.Abs(_currentLocalVelocity.z) < _maxSpeed)
        {
            carRigidBody.AddForceAtPosition(0.9f *_accelerationVal * _moveInput * ProjectOnGround(transform.forward), accelerationPoint.position, ForceMode.Acceleration);
            carRigidBody.AddForceAtPosition(0.1f * _moveInput * (accelerationPoint.forward), accelerationPoint.position, ForceMode.Acceleration);
        }
    }

    private void Deceleration()
    {
        if (Mathf.Abs(_velocityRatio) > 0.001f)
        {
            carRigidBody.AddForceAtPosition(0.9f * _decelerationVal * Mathf.Abs(_velocityRatio) * -ProjectOnGround(transform.forward), accelerationPoint.position, ForceMode.Acceleration);
            carRigidBody.AddForceAtPosition(0.1f * _decelerationVal * Mathf.Abs(_velocityRatio) * -(accelerationPoint.forward), accelerationPoint.position, ForceMode.Acceleration);
        }
        else
        {
            carRigidBody.linearVelocity = Vector3.zero;
        }
    }

    private void Friction()
    {
        if(Mathf.RoundToInt(_steerInput) == 0 && (int)_moveInput == 0 && Mathf.Abs(_velocityRatio) > 0.001f)
        {
            float frictionCoefficient = 10.0f;
            Vector3 frictionForce = -ProjectOnGround(carRigidBody.linearVelocity) * frictionCoefficient;
            carRigidBody.AddForce(frictionForce, ForceMode.Acceleration);
        }
    }

    private void Turn()
    {
        float adjustedSteerStrength = _isDrift ? _steerStrength * (1.5f + _velocityRatio) : _steerStrength;
        carRigidBody.AddTorque(adjustedSteerStrength * _steerInput * _turnCurve.Evaluate(Mathf.Abs(_velocityRatio)) * Mathf.Sign(_velocityRatio) * transform.up, ForceMode.Acceleration);
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
        _velocityRatio = Mathf.Abs(_currentLocalVelocity.z / _maxSpeed);
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
