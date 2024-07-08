using UnityEngine;

public class Suspension : MonoBehaviour
{
    // Car Info
    [SerializeField] private Rigidbody carRigidBody;
    [SerializeField] private Transform[] rayTransforms;
    [SerializeField] private GameObject[] wheels = new GameObject[4];
    [SerializeField] private LayerMask movableMask;
    [SerializeField] private float wheelRadius;

    // Suspension Physics Setting Values
    [SerializeField] private float extraLength;
    [SerializeField] private float carFlexibillity;
    [SerializeField] private float springStiffness;
    [SerializeField] private float damperStiffness;

    // Required Values By Movement Class
    private Vector3 _groundNormal = Vector3.zero;
    private int[] _isGroundedWheels = new int[4];

    public Vector3 GroundNormal => _groundNormal;
    public int[] IsGroundedWheels => _isGroundedWheels;

    private void SetWheelPosition(GameObject wheel, Vector3 wheelPosition)
    {
        wheel.transform.position = wheelPosition;
    }

    public void CalculateSuspension()
    {
        for (int i = 0; i < rayTransforms.Length; ++i)
        {
            RaycastHit hit;
            float maxLength = extraLength + carFlexibillity;

            if (Physics.Raycast(rayTransforms[i].position, -rayTransforms[i].up, out hit, maxLength + wheelRadius, movableMask))
            {
                _groundNormal = hit.normal;
                _isGroundedWheels[i] = 1;
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
                _isGroundedWheels[i] = 0;

                SetWheelPosition(wheels[i], rayTransforms[i].position - rayTransforms[i].up * maxLength);

                Debug.DrawLine(rayTransforms[i].position, rayTransforms[i].position + (wheelRadius + maxLength) * -rayTransforms[i].up, Color.green);
            }
        }
    }
}
