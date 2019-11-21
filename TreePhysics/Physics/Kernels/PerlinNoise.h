class PerlinNoise {
private:
    float frequency;
    int octaves;
    float persistence;
    float lacunarity;
    int seed;
public:
    PerlinNoise(float frequency, int octaves, float persistence, float lacunarity, int seed);
    float value(float2 st);
};
