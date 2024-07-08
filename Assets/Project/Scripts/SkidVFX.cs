using UnityEngine;

public class SkidVFX : VFX
{
    [SerializeField] private TrailRenderer[] skidMarks = new TrailRenderer[2];
    [SerializeField] private ParticleSystem[] skidEffects = new ParticleSystem[2];

    public override void PlayVFX()
    {
        ToggleSkidMarks(true);
        ToggleSkidEffects(true);
    }

    public override void StopVFX()
    {
        ToggleSkidMarks(false);
        ToggleSkidEffects(false);
    }

    private void ToggleSkidMarks(bool toggle)
    {
        foreach (var skidMark in skidMarks)
        {
            skidMark.emitting = toggle;
        }
    }

    private void ToggleSkidEffects(bool toggle)
    {
        foreach (var skidEffect in skidEffects)
        {
            if (toggle) skidEffect.Play();
            else skidEffect.Stop();
        }
    }
}
