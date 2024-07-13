import pytest
from sim._tsdz2 import ffi, lib as ebike # module generated from c-code
import numpy as np

BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100 = 16
mA_to_ADC = 100/BATTERY_CURRENT_PER_10_BIT_ADC_STEP_X100 / 1000

# Set up initial values before each test
@pytest.fixture(autouse=True)
def setup_ebike():
    # Set up initial values before each test
    ebike.m_configuration_variables.ui8_wheel_speed_max = 25
    ebike.ui8_duty_cycle_target = 255 # set by assistance function
    ebike.ui8_adc_battery_current_target = int(5000 * mA_to_ADC)  # 5000mA set by assistance function
    yield
    # Teardown after each test (optional)


def apply_speed_limit_float(speed):
    speed_max = ebike.m_configuration_variables.ui8_wheel_speed_max
    speed_lo = speed_max - 2
    speed_hi = speed_max + 2
    curr_target = ebike.ui8_adc_battery_current_target
    current_lim = np.interp(speed, [speed_lo, speed_hi], [curr_target, 0])
    return current_lim

# Parameterized test function with different ticks values
@pytest.mark.parametrize("speed", [0, 22.9, 23, 23.5, 24, 24.5, 25, 25.5, 26, 26.5, 27, 27.1, 30])
def test_apply_speed_limit(speed):
    ebike.ui16_wheel_speed_x10 = int(speed * 10)

    expected = apply_speed_limit_float(speed) # this has to run first
    ebike.apply_speed_limit()
    result = ebike.ui8_adc_battery_current_target

    assert result ==pytest.approx(expected, rel=1e-1, abs=0.1), f'Expected target {expected/mA_to_ADC}mA, got {result/mA_to_ADC}mA'


# Run the tests
if __name__ == '__main__':
    pytest.main()
