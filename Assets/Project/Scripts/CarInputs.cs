using UnityEngine;
using UnityEngine.InputSystem;

public class CarInputs : MonoBehaviour
{
    private Vector2 _move;
    private bool _drift;

    public Vector2 Move => _move;
    public bool Drift => _drift;

    public void OnMove(InputValue value)
    {
        MoveInput(value.Get<Vector2>());
    }

    public void OnSprint(InputValue value)
    {
        SprintInput(value.isPressed);
    }

    private void MoveInput(Vector2 newMoveDirection)
    {
        _move = newMoveDirection;
    }

    private void SprintInput(bool newSprintState)
    {
        _drift = newSprintState;
    }
}
