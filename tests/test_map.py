import pytest
from sim._tsdz2 import ffi, lib as ebike # module generated from c-code
import numpy as np
from hypothesis import given, assume, strategies as st


@pytest.mark.parametrize(
		"x,    in_min, in_max, out_min, out_max, expected", [
		( 4,	0,		16,		16,		0,		12),
		( 1,	0,		2,		3,		0,		1),
		( 1,	0,		3,		0,		2,		1),
		])
def test_maps_simple(x, in_min, in_max, out_min, out_max, expected):
	map_ui8_result = ebike.map_ui8(x, in_min, in_max, out_min, out_max)
	map_ui16_result = ebike.map_ui16(x, in_min, in_max, out_min, out_max)
	assert map_ui8_result == pytest.approx(expected, abs=1), f'Expected map_ui8_result {expected}, got {map_ui8_result}'
	assert map_ui16_result == expected, f'Expected map_ui16_result {expected}, got {map_ui16_result}'


# Parameterized test function with different ticks values
@pytest.mark.parametrize("x", range(20, 45))
def test_compare_ui8_ui16_map_input_smaller_than_output(x):
	in_min = 23
	in_max = 43
	out_min = 5
	out_max = 250
	map_ui8_result = ebike.map_ui8(x, in_min, in_max, out_min, out_max)
	map_ui16_result = ebike.map_ui16(x, in_min, in_max, out_min, out_max)
	# ! map_ui8 has lower precision so allow for an error of 1
	assert map_ui16_result == pytest.approx(map_ui8_result, abs=1), f'Expected map_ui8_result {map_ui8_result} == map_ui16_result {map_ui16_result}'

@pytest.mark.parametrize("x", range(20, 90))
def test_compare_ui8_ui16_map_input_greater_than_output(x):
	in_min = 23
	in_max = 87
	out_min = 5
	out_max = 50
	map_ui8_result = ebike.map_ui8(x, in_min, in_max, out_min, out_max)
	map_ui16_result = ebike.map_ui16(x, in_min, in_max, out_min, out_max)
	
	# ! map_ui8 has lower precision so allow for an error of 1
	assert map_ui16_result == pytest.approx(map_ui8_result, abs=1), f'Expected map_ui8_result {map_ui8_result} == map_ui16_result {map_ui16_result}'



# Define the hypothesis test for map_ui8
@given(
	x=st.integers(min_value=0, max_value=65535),
	in_min=st.integers(min_value=0, max_value=65535),
	in_max=st.integers(min_value=0, max_value=65535),
	out_min=st.integers(min_value=0, max_value=65535),
	out_max=st.integers(min_value=0, max_value=65535))
def test_maps_full_ranges(x, in_min, in_max, out_min, out_max):
	assume(in_min <= in_max)
	
	expected = np.interp(x, [in_min, in_max], [out_min, out_max])
	# !test map_ui8 only for 8 bit ranges
	if max(x, in_min, in_max, out_min, out_max) < 2^8:
		map_ui8_result = ebike.map_ui8(x, in_min, in_max, out_min, out_max)
		# ! map_ui8 lowest precision is 1
		assert map_ui8_result == pytest.approx(expected, abs=1), \
			f"map_ui8({x}, {in_min}, {in_max}, {out_min}, {out_max}) returned {map_ui8_result}, expected {expected}"
	else:
		print(f'x={x}, in_min={in_min}, in_max={in_max}, out_min={out_min}, out_max={out_max}')
		
	map_ui16_result = ebike.map_ui16(x, in_min, in_max, out_min, out_max)
	# ! map_ui16 lowest precision is 0.5 (thanks to nearest rounding)
	assert map_ui16_result == pytest.approx(expected, abs=.5), \
		f"map_ui16({x}, {in_min}, {in_max}, {out_min}, {out_max}) returned {map_ui16_result}, expected {expected}"



# Run the tests
if __name__ == '__main__':
	pytest.main()
