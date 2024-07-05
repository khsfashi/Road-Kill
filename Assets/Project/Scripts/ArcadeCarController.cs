using System.Data.SqlTypes;
using UnityEngine;
using UnityEngine.InputSystem;

public class ArcadeCarController : MonoBehaviour
{
    private int[] isGroundedWheels = new int[4];
    private bool isGrounded = false;

    // 강체랑 바닥으로 광선 쏠 지점 네개
    // 테스트 해보고 광선 더 필요하면 추가할지도...
    [Header("Car Info")]
    [SerializeField] private Rigidbody carRigidBody;
    [SerializeField] private Collider carCollider;
    [SerializeField] private Transform[] rayTransforms;
    [SerializeField] private LayerMask movableMask;
    [SerializeField] private GameObject[] wheels = new GameObject[4];
    [SerializeField] private GameObject[] frontWheelsParents = new GameObject[4];
    [SerializeField] private float wheelRadius;

    [Header("Suspension Physics Settings")]
    [SerializeField] private float extraLength;
    [SerializeField] private float carFlexibillity;
    [SerializeField] private float springStiffness;
    [SerializeField] private float damperStiffness;

    [Header("Speed Physics Settings")]
    [SerializeField] private Transform accelerationPoint;
    [SerializeField] private float accelerationVal = 15f;
    [SerializeField] private float decelerationVal = 10f;
    [SerializeField] private float maxSpeed = 100f;
    [SerializeField] private float steerStrength = 15f;
    [SerializeField] private AnimationCurve turnCurve;
    [SerializeField] private float dragCoefficient = 1f;

    [Header("VFX")]
    [SerializeField] private float wheelRotationSpeed = 3000f;
    [SerializeField] private float maxSteeringAngle = 30f;
    [SerializeField] private TrailRenderer[] skidMarks = new TrailRenderer[2];
    [SerializeField] private ParticleSystem[] skidEffects = new ParticleSystem[2];
    [SerializeField] private float minSkidVelocity = 10f;


    [Header("Car Input")]
    private float moveInput = 0f;
    private float steerInput = 0f;

    private Vector3 currentLocalVelocity = Vector3.zero;
    private float velocityRatio = 0f;
    private Vector3 groundNormal = Vector3.zero;

    private float currentSteerAngle = 0.0f;
    private Vector3 wheelEulerAngles = Vector3.zero;

    #region Unity

    private void Start()
    {
        // 인스펙터에서 넣어주는데, 혹시 까먹을까봐 안전장치용
        carRigidBody = GetComponent<Rigidbody>();
    }

    private void Update()
    {
        GetCarInput();
    }
    private void FixedUpdate()
    {
        Suspension();
        CheckGrounded();
        CalculateVelocity();
        Movement();
        VFX();
    }
    #endregion

    #region Movement
    private void Movement()
    {
        if (isGrounded)
        {
            Acceleration();
            Deceleration();
            Turn();
            SidewaysDrag();
        }
    }

    private void Acceleration()
    {
        carRigidBody.AddForceAtPosition(accelerationVal * moveInput * ProjectOnGround(transform.forward), accelerationPoint.position, ForceMode.Acceleration);
    }

    private void Deceleration()
    {
        carRigidBody.AddForceAtPosition(decelerationVal * moveInput * -ProjectOnGround(transform.forward), accelerationPoint.position, ForceMode.Acceleration);
    }

    private void Turn()
    {
        carRigidBody.AddTorque(steerStrength * steerInput * turnCurve.Evaluate(Mathf.Abs(velocityRatio)) * Mathf.Sign(velocityRatio) * transform.up, ForceMode.Acceleration);
    }

    private void SidewaysDrag()
    {
        float currentSidewaysSpeed = currentLocalVelocity.x;
        float dragMagnitude = -currentSidewaysSpeed * dragCoefficient;

        Vector3 dragForce = transform.right * dragMagnitude;
        carRigidBody.AddForceAtPosition(dragForce, carRigidBody.worldCenterOfMass, ForceMode.Acceleration);
    }

    private Vector3 ProjectOnGround(Vector3 vector)
    {
        Vector3 projected = vector - Vector3.Dot(vector, groundNormal) * groundNormal;
        return projected.normalized;
    }

    private void GetCarInput()
    {
        moveInput = Input.GetAxis("Vertical");
        steerInput = Input.GetAxis("Horizontal");
    }

    private void CalculateVelocity()
    {
        currentLocalVelocity = transform.InverseTransformDirection(carRigidBody.linearVelocity);
        velocityRatio = currentLocalVelocity.z / maxSpeed;
        //Debug.Log(currentLocalVelocity.z * 3.6f + "km/h");
    }

    #endregion

    #region Check Car State
    private void CheckGrounded()
    {
        int groundedWheels = 0;
        for (int i = 0; i < isGroundedWheels.Length; ++i)
        {
            groundedWheels += isGroundedWheels[i];
        }

        isGrounded = groundedWheels > 1 ? true : false;
    }
    #endregion

    #region VFX

    private void VFX()
    {
        WheelVFXs();
        //SkidVFXs();
    }

    private void SkidVFXs()
    {
        if (isGrounded && currentLocalVelocity.x > minSkidVelocity)
        {
            ToggleSkidMarks(true);
            ToggleSkidEffects(true);
        }
        else
        {
            ToggleSkidMarks(false);
            ToggleSkidEffects(false);
        }
    }

    private void ToggleSkidMarks(bool toggle)
    {
        foreach(var skidMark in skidMarks)
        {
            skidMark.enabled = toggle;
        }
    }

    private void ToggleSkidEffects(bool toggle)
    {
        foreach(var skidEffect in skidEffects)
        {
            if (toggle) skidEffect.Play();
            else skidEffect.Stop();
        }
    }

    private void WheelVFXs()
    {
        float targetSteeringAngle = maxSteeringAngle * steerInput;
        float steerSmoothSpeed = 5.0f;
        currentSteerAngle = Mathf.Lerp(currentSteerAngle, targetSteeringAngle, steerSmoothSpeed * Time.deltaTime);

        for(int i = 0; i < wheels.Length; ++i)
        {
            if (i < 2)
            {
                wheels[i].transform.Rotate(Vector3.right, wheelRotationSpeed * velocityRatio * Time.deltaTime, Space.Self);
                // 원래 벡터 생성해서 쓰다가 new 매프레임 하는 거 좀 성능 문제 생길 것 같아서 멤버 변수 통해서 바꿈
                wheelEulerAngles = frontWheelsParents[i].transform.localEulerAngles;
                wheelEulerAngles.y = currentSteerAngle;
                frontWheelsParents[i].transform.localEulerAngles = wheelEulerAngles;
            }
            else
            {
                wheels[i].transform.Rotate(Vector3.right, wheelRotationSpeed * moveInput * Time.deltaTime, Space.Self);
                wheels[i].transform.Rotate(Vector3.right, wheelRotationSpeed * velocityRatio * Time.deltaTime, Space.Self);
            }
        }
    }

    private void SetWheelPosition(GameObject wheel, Vector3 wheelPosition)
    {
        wheel.transform.position = wheelPosition; 
    }

    #endregion

    #region Suspension
    private void Suspension()
    {
        for (int i = 0; i < rayTransforms.Length; ++i)
        {
            RaycastHit hit;
            float maxLength = extraLength + carFlexibillity;

            if (Physics.Raycast(rayTransforms[i].position, -rayTransforms[i].up, out hit, maxLength + wheelRadius, movableMask))
            {
                groundNormal = hit.normal;
                isGroundedWheels[i] = 1;
                float curLength = hit.distance - wheelRadius;
                float compression = (extraLength - curLength) / carFlexibillity;

                float springVel = Vector3.Dot(carRigidBody.GetPointVelocity(rayTransforms[i].position), rayTransforms[i].up);
                float damperForce = damperStiffness * springVel;

                float springForce = springStiffness * compression;
                float force = springForce - damperForce;


                carRigidBody.AddForceAtPosition(force * rayTransforms[i].up, rayTransforms[i].position);

                SetWheelPosition(wheels[i], hit.point + rayTransforms[i].up * wheelRadius);

                Debug.DrawLine(rayTransforms[i].position, hit.point, Color.red);
            }
            else
            {
                isGroundedWheels[i] = 0;

                SetWheelPosition(wheels[i], rayTransforms[i].position - rayTransforms[i].up * maxLength);

                Debug.DrawLine(rayTransforms[i].position, rayTransforms[i].position + (wheelRadius + maxLength) * -rayTransforms[i].up, Color.green);
            }
        }
    }
    #endregion
}
