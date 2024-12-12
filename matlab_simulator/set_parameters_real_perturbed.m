PID_speed.p = 11.84;
PID_speed.i = 0.0061;

PID_current.p = 6;
PID_current.i = 150;

perturbation = 0.15;

coeff = @(x) (1 - perturbation) + (perturbation * 2) * x;


BLDC.RatedVoltage = 48;
BLDC.RatedSpeed = 4390 / 30 * pi * coeff(rand());
BLDC.CurrentMax = 10;
BLDC.VoltageMax = BLDC.RatedVoltage;
BLDC.PolePairs = 7;
BLDC.StatorPhaseResistance = 0.994 * coeff(rand());
BLDC.InductanceLd = 0.995e-3 * coeff(rand());
BLDC.InductanceLq = BLDC.InductanceLd;
BLDC.InductanceL0 = BLDC.InductanceLd;
BLDC.FluxLinkage = 48/BLDC.RatedSpeed/(BLDC.PolePairs);
BLDC.Inertia = 44e-07 * coeff(rand());
BLDC.BreakawayFrictionTorque = 0.002130661000000 *1e-6 * coeff(rand()); %??
BLDC.CoulombFrictionTorque = BLDC.BreakawayFrictionTorque; %??
BLDC.ViscousFrictionCoefficient = 0.0083 * coeff(rand());
BLDC.RotorPositionInit = 0;
BLDC.RotorVelocityInit = 0;

inverter.BusVoltage = BLDC.RatedVoltage;
inverter.CurrentMax = 2;
inverter.GatesActiveHigh = true;
inverter.GatesComplementary = true;

paramPwmCompareAtZeroVolt = 330;
paramPwmCompareMax = 660;
paramPwmComparePerVolt = paramPwmCompareMax/2/BLDC.RatedVoltage;

disc.Inertia =  0.7497e-03 * coeff(rand());

pwm.CounterMax = 660;
pwm.CountPerPeriod = 1320;
pwm.TimePerCount = 3.030303030303031e-08;
pwm.Period = 4.000000000000000e-04;

sampleTime.CurrentControl = 4.000000000000000e-06;
sampleTime.VelocityControl = 1.000000000000000e-03;





