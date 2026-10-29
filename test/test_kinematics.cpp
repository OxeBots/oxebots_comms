#include <gtest/gtest.h>
#include <cmath>

namespace oxebots_comms
{

// Função de transformação cinematica de velocidades: Global -> Local do Robô
// Implementada identicamente ao ra_controller_node
inline void transformGlobalToLocalVelocity(
    float vx_global, float vy_global, float theta,
    float & v_forward, float & v_left)
{
    v_forward = vx_global * std::cos(theta) + vy_global * std::sin(theta);
    v_left = -vx_global * std::sin(theta) + vy_global * std::cos(theta);
}

} // namespace oxebots_comms

TEST(KinematicsTransformationTest, ZeroOrientationAlignsWithGlobalAxes)
{
    float v_fwd = 0.0f;
    float v_left = 0.0f;

    // Robô apontado para o eixo X (theta = 0)
    oxebots_comms::transformGlobalToLocalVelocity(1.5f, 0.5f, 0.0f, v_fwd, v_left);

    EXPECT_NEAR(v_fwd, 1.5f, 1e-5);
    EXPECT_NEAR(v_left, 0.5f, 1e-5);
}

TEST(KinematicsTransformationTest, NinetyDegreesOrientationTransformsCorrectly)
{
    float v_fwd = 0.0f;
    float v_left = 0.0f;

    // Robô apontado para o norte (theta = +pi/2)
    const float theta = static_cast<float>(M_PI_2);

    // Comando global para o norte (vy = 1.0 m/s): deve virar movimento para frente no robô
    oxebots_comms::transformGlobalToLocalVelocity(0.0f, 1.0f, theta, v_fwd, v_left);
    EXPECT_NEAR(v_fwd, 1.0f, 1e-5);
    EXPECT_NEAR(v_left, 0.0f, 1e-5);

    // Comando global para leste (vx = 1.0 m/s): deve virar movimento para a direita (-left)
    oxebots_comms::transformGlobalToLocalVelocity(1.0f, 0.0f, theta, v_fwd, v_left);
    EXPECT_NEAR(v_fwd, 0.0f, 1e-5);
    EXPECT_NEAR(v_left, -1.0f, 1e-5);
}

TEST(KinematicsTransformationTest, OppositeOrientationInvertsForwardDirection)
{
    float v_fwd = 0.0f;
    float v_left = 0.0f;

    // Robô apontado para trás (theta = pi)
    const float theta = static_cast<float>(M_PI);

    oxebots_comms::transformGlobalToLocalVelocity(2.0f, 0.0f, theta, v_fwd, v_left);
    EXPECT_NEAR(v_fwd, -2.0f, 1e-5);
    EXPECT_NEAR(v_left, 0.0f, 1e-5);
}

TEST(KinematicsTransformationTest, NormIsPreservedAcrossRotations)
{
    const float vx = 1.2f;
    const float vy = -0.9f;
    const float global_norm = std::hypot(vx, vy);

    // Testa rotações em passos de 30 graus
    for (int deg = 0; deg < 360; deg += 30) {
        float theta = static_cast<float>(deg * M_PI / 180.0);
        float v_fwd = 0.0f;
        float v_left = 0.0f;

        oxebots_comms::transformGlobalToLocalVelocity(vx, vy, theta, v_fwd, v_left);
        float local_norm = std::hypot(v_fwd, v_left);

        EXPECT_NEAR(local_norm, global_norm, 1e-4);
    }
}
