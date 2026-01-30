/**
 * @file test_pid_controller.cpp
 * @brief Unit tests for PID controller
 *
 * Tests PID computation, saturation handling, and parameter validation.
 */

#include <gtest/gtest.h>
#include "fgm_plugin/PID.h"
#include <cmath>

class PIDControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        controller_ = std::make_unique<PIDController>();
    }

    void TearDown() override {
        controller_.reset();
    }

    std::unique_ptr<PIDController> controller_;
};

/**
 * Test: PID initialization
 */
TEST_F(PIDControllerTest, Initialization) {
    EXPECT_NO_THROW({
        PIDController pid;
    });
}

/**
 * Test: Set PID coefficients
 */
TEST_F(PIDControllerTest, SetPIDCoeffs) {
    EXPECT_NO_THROW({
        controller_->setPIDCoeffs(1.0, 0.5, 0.1);
    });
}

/**
 * Test: Output saturation - upper limit
 */
TEST_F(PIDControllerTest, OutputSaturationUpper) {
    controller_->setPIDCoeffs(10.0, 0.0, 0.0);  // High P gain
    controller_->setOutputSaturations(-1.0, 1.0);
    controller_->setSampleTime(0.1);

    // Large positive error should saturate to upper limit
    double output = controller_->computeControlSignal(100.0);

    EXPECT_LE(output, 1.0);  // Should not exceed upper limit
    EXPECT_GE(output, 0.0);  // Should be positive for positive error
}

/**
 * Test: Output saturation - lower limit (Bug fix from Phase 1)
 */
TEST_F(PIDControllerTest, OutputSaturationLower) {
    controller_->setPIDCoeffs(10.0, 0.0, 0.0);  // High P gain
    controller_->setOutputSaturations(-1.0, 1.0);
    controller_->setSampleTime(0.1);

    // Large negative error should saturate to LOWER limit (not upper!)
    double output = controller_->computeControlSignal(-100.0);

    EXPECT_GE(output, -1.0);  // Should not go below lower limit
    EXPECT_LE(output, 0.0);   // Should be negative for negative error
    EXPECT_NEAR(output, -1.0, 0.01);  // Should be at lower limit
}

/**
 * Test: Proportional control
 */
TEST_F(PIDControllerTest, ProportionalControl) {
    controller_->setPIDCoeffs(2.0, 0.0, 0.0);  // P only
    controller_->setOutputSaturations(-10.0, 10.0);
    controller_->setSampleTime(0.1);

    double error = 1.0;
    double output = controller_->computeControlSignal(error);

    // Output should be Kp * error = 2.0 * 1.0 = 2.0
    EXPECT_NEAR(output, 2.0, 0.01);
}

/**
 * Test: Invalid saturation parameters
 */
TEST_F(PIDControllerTest, InvalidSaturationParameters) {
    // Lower >= Upper should throw
    EXPECT_THROW({
        controller_->setOutputSaturations(1.0, -1.0);  // Lower > Upper
    }, std::invalid_argument);

    EXPECT_THROW({
        controller_->setOutputSaturations(1.0, 1.0);  // Lower == Upper
    }, std::invalid_argument);

    EXPECT_THROW({
        controller_->setIntegratorSaturations(1.0, 0.5);  // Lower > Upper
    }, std::invalid_argument);
}

/**
 * Test: Integrator saturation
 */
TEST_F(PIDControllerTest, IntegratorSaturation) {
    controller_->setPIDCoeffs(0.0, 1.0, 0.0);  // I only
    controller_->setIntegratorSaturations(-1.0, 1.0);
    controller_->setOutputSaturations(-10.0, 10.0);
    controller_->setSampleTime(0.1);

    // Apply constant error many times
    for (int i = 0; i < 100; ++i) {
        controller_->computeControlSignal(1.0);
    }

    // Integrator should saturate
    double output = controller_->getControllerOutput();
    EXPECT_LE(std::abs(output), 10.0);  // Within output limits
}

/**
 * Test: Derivative filtered control
 */
TEST_F(PIDControllerTest, DerivativeFilteredControl) {
    controller_->setPIDCoeffs(1.0, 0.0, 0.5);  // P and D
    controller_->setOutputSaturations(-10.0, 10.0);
    controller_->setDerivativeFilterConstant(50.0);
    controller_->setSampleTime(0.1);

    // Step change in error
    double output1 = controller_->derivativeFilteredControlSignal(0.0);
    double output2 = controller_->derivativeFilteredControlSignal(1.0);

    // Output should change due to derivative term
    EXPECT_NE(output1, output2);
}

/**
 * Test: Sample time setting
 */
TEST_F(PIDControllerTest, SampleTime) {
    controller_->setSampleTime(0.05);
    EXPECT_NEAR(controller_->getSampleTime(), 0.05, 0.0001);

    controller_->setSampleTime(0.1);
    EXPECT_NEAR(controller_->getSampleTime(), 0.1, 0.0001);
}

/**
 * Test: Zero error gives zero output
 */
TEST_F(PIDControllerTest, ZeroErrorZeroOutput) {
    controller_->setPIDCoeffs(1.0, 0.5, 0.1);
    controller_->setOutputSaturations(-10.0, 10.0);
    controller_->setSampleTime(0.1);

    // With zero error consistently
    for (int i = 0; i < 10; ++i) {
        controller_->computeControlSignal(0.0);
    }

    double output = controller_->getControllerOutput();
    EXPECT_NEAR(output, 0.0, 0.1);  // Should be near zero
}

/**
 * Test: Symmetric response
 */
TEST_F(PIDControllerTest, SymmetricResponse) {
    PIDController pid1, pid2;

    pid1.setPIDCoeffs(1.0, 0.0, 0.0);
    pid1.setOutputSaturations(-10.0, 10.0);
    pid1.setSampleTime(0.1);

    pid2.setPIDCoeffs(1.0, 0.0, 0.0);
    pid2.setOutputSaturations(-10.0, 10.0);
    pid2.setSampleTime(0.1);

    double output_pos = pid1.computeControlSignal(2.0);
    double output_neg = pid2.computeControlSignal(-2.0);

    // Outputs should be symmetric
    EXPECT_NEAR(output_pos, -output_neg, 0.01);
}

// Main function
int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
