

PID_current.p = 50;
PID_current.i = 1;


% perturbation = 0.15;
fprintf("(perturbation = " + perturbation + ")\n")
coeff = @(x) (1 - perturbation) + (perturbation * 2) * x;

PID_speed.p = 0.1 * coeff(rand());
PID_speed.i = 0.1 * coeff(rand());

BLDC.RatedVoltage = 48;
BLDC.RatedSpeed = 4390 / 30 * pi * coeff(rand());
BLDC.CurrentMax = 4.8;
BLDC.VoltageMax = BLDC.RatedVoltage;
BLDC.PolePairs = 7;
BLDC.StatorPhaseResistance = 0.994 * 0.357256158228637  * coeff(rand());
BLDC.InductanceLd = 0.995e-3 * 1.383264744587776  * coeff(rand());
BLDC.InductanceLq = BLDC.InductanceLd;
BLDC.InductanceL0 = BLDC.InductanceLd;
BLDC.FluxLinkage = 48/BLDC.RatedSpeed/(BLDC.PolePairs) * 1.179953752195608;
BLDC.Inertia = 44e-07 * 29.043590734405700  * coeff(rand());
BLDC.BreakawayFrictionTorque = 0.002130661000000 *1e-3  * coeff(rand()); %??
BLDC.CoulombFrictionTorque = BLDC.BreakawayFrictionTorque; %??
BLDC.ViscousFrictionCoefficient = 0.0083*1e-6  * coeff(rand());
BLDC.RotorPositionInit = 0;
BLDC.RotorVelocityInit = 0;
i_omega = 2.497  * coeff(rand());

inverter.BusVoltage = BLDC.RatedVoltage;
inverter.CurrentMax = BLDC.CurrentMax;
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





