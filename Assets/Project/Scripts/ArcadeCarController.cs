using UnityEngine;
using UnityEngine.InputSystem;

public class ArcadeCarController : MonoBehaviour
{
    // ��ü�� �ٴ����� ���� �� ���� �װ�
    // �׽�Ʈ �غ��� ���� �� �ʿ��ϸ� �߰�������...
    [Header("Car Info")]
    [SerializeField] private Rigidbody carRigidBody;
    [SerializeField] private Collider carCollider;
    [SerializeField] private Transform[] rayTransforms;
    [SerializeField] private LayerMask movableMask;
    [SerializeField] private float wheelRadius;

    [Header("Physics Settings")]
    [SerializeField] private float extraLength;
    [SerializeField] private float carFlexibillity;
    [SerializeField] private float springStiffness;
    [SerializeField] private float damperStiffness;

    private void Start()
    {
        // �ν����Ϳ��� �־��ִµ�, Ȥ�� �������� ������ġ��
        carRigidBody = GetComponent<Rigidbody>();
        AdjustCenterOfMass();
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.Space))
        {
            carRigidBody.AddForce(Vector3.up * 10000, ForceMode.Impulse);
        }
    }
    private void FixedUpdate()
    {
        Suspension();
    }
    private void AdjustCenterOfMass()
    {
        Vector3 centerOfMass = carRigidBody.centerOfMass;
        centerOfMass.y = -carCollider.bounds.extents.y - wheelRadius - extraLength;
        carRigidBody.centerOfMass = centerOfMass;
        Debug.DrawLine(transform.TransformPoint(centerOfMass), transform.TransformPoint(centerOfMass) + Vector3.up * 0.5f, Color.yellow, 100f);
    }

    private void Suspension()
    {
        foreach (Transform rayTransform in rayTransforms)
        {
            RaycastHit hit;
            float maxLength = extraLength + carFlexibillity;

            if(Physics.Raycast(rayTransform.position, -rayTransform.up, out hit, maxLength + wheelRadius, movableMask))
            {
                float curLength = hit.distance - wheelRadius;
                float compression = (extraLength - curLength) / carFlexibillity;

                float springVel = Vector3.Dot(carRigidBody.GetPointVelocity(rayTransform.position), rayTransform.up);
                float damperForce = damperStiffness * springVel;

                float springForce = springStiffness * compression;
                float force = springForce - damperForce;


                carRigidBody.AddForceAtPosition(force * rayTransform.up, rayTransform.position);

                Debug.DrawLine(rayTransform.position, hit.point, Color.red);
            }
            else
            {
                Debug.DrawLine(rayTransform.position, rayTransform.position + (wheelRadius + maxLength) * -rayTransform.up, Color.green);
            }
        }
    }
}
