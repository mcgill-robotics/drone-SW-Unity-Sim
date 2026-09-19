using UnityEngine;

namespace Utils
{
    /// <summary>Gaussian noise helpers. Ported from auv-sim-unity.</summary>
    public static class Stochastic
    {
        public static float GenerateGaussian()
        {
            float u1 = 1.0f - Random.value;
            float u2 = Random.value;
            return Mathf.Sqrt(-2.0f * Mathf.Log(u1)) * Mathf.Sin(2.0f * Mathf.PI * u2);
        }

        public static Vector3 GenerateWhiteNoiseVector(float stdDev)
        {
            return new Vector3(GenerateGaussian() * stdDev, GenerateGaussian() * stdDev, GenerateGaussian() * stdDev);
        }
    }

    /// <summary>
    /// First-order Gauss-Markov process (bounded random walk) for sensor bias drift.
    /// x(k+1) = A x(k) + B w(k), A = exp(-dt/tau), B = sigma sqrt(1 - A^2).
    /// </summary>
    public class GaussMarkovVector
    {
        private float _a, _b;
        public Vector3 CurrentBias { get; private set; }

        public GaussMarkovVector(float correlationTime, float biasSigma, float dt)
        {
            RecalculateConstants(correlationTime, biasSigma, dt);
        }

        public void RecalculateConstants(float correlationTime, float biasSigma, float dt)
        {
            float beta = 1f / Mathf.Max(correlationTime, 0.001f);
            _a = Mathf.Exp(-beta * dt);
            _b = biasSigma * Mathf.Sqrt(1f - Mathf.Exp(-2f * beta * dt));
        }

        public Vector3 Step()
        {
            Vector3 n = Stochastic.GenerateWhiteNoiseVector(1f);
            CurrentBias = CurrentBias * _a + n * _b;
            return CurrentBias;
        }

        public void Reset() => CurrentBias = Vector3.zero;
    }
}
