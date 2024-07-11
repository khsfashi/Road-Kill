using UnityEngine;

public class Suspension
{
    // Car Info
    private Transform[] _rayTransforms;
    
    private GameObject[] _wheels = new GameObject[4];
    private Rigidbody _carRigidBody;
    private LayerMask _movableMask;
    private float _wheelRadius;

    // Suspension Physics Setting Values
    private float _extraLength;
    private float _carFlexibillity;
    private float _springStiffness;
    private float _damperStiffness;

    // Required Values By Movement Class
    private Vector3 _groundNormal = Vector3.zero;
    private int[] _isGroundedWheels = new int[4];

    public Vector3 GroundNormal => _groundNormal;
    public int[] IsGroundedWheels => _isGroundedWheels;

    public Suspension(ArcadeCarMovementSettings settings, Rigidbody rb, GameObject[] wheels, Transform[] rayTransforms)
    {
        _carRigidBody = rb;
        _wheels = wheels;
        _rayTransforms = rayTransforms;

        _movableMask = settings.movableMask;
        _wheelRadius = settings.wheelRadius;
        _extraLength = settings.extraLength;
        _carFlexibillity = settings.carFlexibillity;
        _springStiffness = settings.springStiffness;
        _damperStiffness = settings.damperStiffness;
    }

    private void SetWheelPosition(GameObject wheel, Vector3 wheelPosition)
    {
        wheel.transform.position = wheelPosition;
    }

    public void CalculateSuspension()
    {
        for (int i = 0; i < _rayTransforms.Length; ++i)
        {
            RaycastHit hit;
            float maxLength = _extraLength + _carFlexibillity;

            if (Physics.Raycast(_rayTransforms[i].position, -_rayTransforms[i].up, out hit, maxLength + _wheelRadius, _movableMask))
            {
                _groundNormal = hit.normal;
                _isGroundedWheels[i] = 1;
                float curLength = hit.distance - _wheelRadius;
                float compression = (_extraLength - curLength) / _carFlexibillity;

                float springVel = Vector3.Dot(_carRigidBody.GetPointVelocity(_rayTransforms[i].position), _rayTransforms[i].up);
                float damperForce = _damperStiffness * springVel;

                float springForce = _springStiffness * compression;
                float force = springForce - damperForce;


                _carRigidBody.AddForceAtPosition(force * _rayTransforms[i].up, _rayTransforms[i].position);

                Vector3 wheelPosition = hit.point + _rayTransforms[i].up * _wheelRadius;
                if (wheelPosition.y > _rayTransforms[i].position.y - _wheelRadius) 
                {
                    wheelPosition.y = _rayTransforms[i].position.y - _wheelRadius;
                }
                SetWheelPosition(_wheels[i], wheelPosition);

                Debug.DrawLine(_rayTransforms[i].position, hit.point, Color.red);
            }
            else
            {
                _isGroundedWheels[i] = 0;

                SetWheelPosition(_wheels[i], _rayTransforms[i].position - _rayTransforms[i].up * maxLength);

                Debug.DrawLine(_rayTransforms[i].position, _rayTransforms[i].position + (_wheelRadius + maxLength) * -_rayTransforms[i].up, Color.green);
            }
        }
    }
}
