'''
Calculations to provide correct calibration for the strain gauges without iterative process.
The logic is the following:

elongation_from_mV(mV) = elongation = 0;
((elongation_from_mV)^(-1))(0) = mV_for_0_elongation;
offset_to_be_applied = mV - mV_for_0_elongation;

during the measurements
mV =- offset_to_be_applied;
'''

from sympy import solve, Symbol,Eq
from sympy.abc import x,y
# print(acquisinatore_link_deformation_from_V_to_elongation(Symbol('x')))

# a = solve(-0.00142146410803127*(124319.598360656 - 745.901639344262*x)/(2.13114754098361*x + 348.801147540984) + 0.497512437810945, x)
# print(a)

# b = solve( Eq(y, -0.00142146410803127*(124319.598360656 - 745.901639344262*x)/(2.13114754098361*x + 348.801147540984) + 0.497512437810945), x)
# print(b)

STRAIN_GAUGE_R1 =   352.0
STRAIN_GAUGE_R2 =   350.0
STRAIN_GAUGE_R3 =   352.0
STRAIN_GAUGE_RNOM = 350.0
STRAIN_GAUGE_G =    100.0
STRAIN_GAUGE_VREF = 1.501
STRAIN_GAUGE_VS =   3.294
STRAIN_GAUGE_K =    2.01

def f(x):
    return (
        (STRAIN_GAUGE_RNOM -
         ((STRAIN_GAUGE_R2 * STRAIN_GAUGE_R3) -
          (STRAIN_GAUGE_R2 * ((x - STRAIN_GAUGE_VREF) / STRAIN_GAUGE_G) * (STRAIN_GAUGE_R1 + STRAIN_GAUGE_R2)) / STRAIN_GAUGE_VS) /
             (STRAIN_GAUGE_R1 +
              ((((x - STRAIN_GAUGE_VREF) / STRAIN_GAUGE_G) * (STRAIN_GAUGE_R1 + STRAIN_GAUGE_R2)) / STRAIN_GAUGE_VS))) /
        (STRAIN_GAUGE_RNOM * STRAIN_GAUGE_K))

def inverse(y):
    return 45600.0*(-153747874245039.0*y - 1403005464481.0)/(4.28360655737706e+16*y - 4.26229508196721e+16)

print(f"-12={inverse(f(-12))}")
print(f"3={inverse(f(3))}")
print(f"15={inverse(f(15))}")
print(f"160={inverse(f(160))}")
print(f"0={inverse(f(0))}")

