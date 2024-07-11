using UnityEngine;
using UnityEngine.InputSystem;

[RequireComponent(typeof(PlayerInput))]
public class CarController : MonoBehaviour
{
    [Header("Other Components")]
    [SerializeField] private VFX vfxPlayer;
    [SerializeField] private Movement movement;

    [Header("Skid Effect Value")]
    [SerializeField] private float minSkidVelocity = 10f;

    [Header("Force To Animal")]
    [SerializeField] private float forceToAnimal = 10f;
    [SerializeField] private float torqueToAnimal = 5f;

    private PlayerInput _playerInput;
    private CarInputs _carInputs;
    
    private Vector3 _currentLocalVelocity;

    private void Start()
    {
        _playerInput = GetComponent<PlayerInput>();
        _carInputs = GetComponent<CarInputs>();
    }

    private void Update()
    {
        movement.UpdateDirection(_carInputs.Move.x, _carInputs.Move.y, 0.0f);
        movement.SetSprint(_carInputs.Drift);
    }

    private void FixedUpdate()
    {
        movement.UpdateMovementState();
        movement.UpdateMovement();
        UpdateVFXs();
    }

    private void OnCollisionEnter(Collision collision)
    {
        AnimalCollision(collision);
    }

    private void UpdateVFXs()
    {
        _currentLocalVelocity = movement.GetLocalVelocity();
        if (_carInputs.Drift && Mathf.Abs(_currentLocalVelocity.x) > minSkidVelocity && movement.CanMove())
        {
            vfxPlayer.PlayVFX();
        }
        else
        {
            vfxPlayer.StopVFX();
        }
    }

    private void AnimalCollision(Collision collision)
    {
        int animalLayer = LayerMask.NameToLayer("Animal");

        if (collision.gameObject.layer == animalLayer)
        {
            ContactPoint contact = collision.contacts[0];

            Vector3 forceDirection = contact.point - transform.position;
            forceDirection = forceDirection.normalized;
            Rigidbody rb = collision.gameObject.GetComponent<Rigidbody>();

            if (rb != null)
            {
                rb.AddForce(forceDirection * forceToAnimal * _currentLocalVelocity.z, ForceMode.Impulse);
                Vector3 randomTorque = new Vector3(
                    Random.Range(-1f, 1f),
                    Random.Range(-1f, 1f),
                    Random.Range(-1f, 1f)
                ).normalized;

                rb.AddTorque(randomTorque * torqueToAnimal * _currentLocalVelocity.z, ForceMode.Impulse);
            }
        }
    }
}
