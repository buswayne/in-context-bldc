PID_speed.p = 1;
PID_speed.i = 0.01;

PID_current.p = 40;
PID_current.i = 1;

BLDC.RatedVoltage = 48;
BLDC.RatedSpeed = 4390 / 30 * pi;
BLDC.CurrentMax = 4.8;
BLDC.VoltageMax = BLDC.RatedVoltage;
BLDC.PolePairs = 7;
BLDC.StatorPhaseResistance = 0.994 * 0.357256158228637;
BLDC.InductanceLd = 0.995e-3 * 1.383264744587776;
BLDC.InductanceLq = BLDC.InductanceLd;
BLDC.InductanceL0 = BLDC.InductanceLd;
BLDC.FluxLinkage = 48/BLDC.RatedSpeed/(BLDC.PolePairs) * 1.179953752195608;
BLDC.Inertia = 44e-07 * 29.043590734405700;
BLDC.BreakawayFrictionTorque = 0.002130661000000 *1e-3; %??
BLDC.CoulombFrictionTorque = 0.002130661000000 *1e-3; %??
BLDC.ViscousFrictionCoefficient = 0.0083*1e-6;
BLDC.RotorPositionInit = 0;
BLDC.RotorVelocityInit = 0;
i_omega = 2.497;

inverter.BusVoltage = BLDC.RatedVoltage;
inverter.CurrentMax = BLDC.CurrentMax;
inverter.GatesActiveHigh = true;
inverter.GatesComplementary = true;

paramPwmCompareAtZeroVolt = 330;
paramPwmCompareMax = 660;
paramPwmComparePerVolt = paramPwmCompareMax/2/BLDC.RatedVoltage;

disc.Inertia =  0.7497e-03;

pwm.CounterMax = 660;
pwm.CountPerPeriod = 1320;
pwm.TimePerCount = 3.030303030303031e-08;
pwm.Period = 4.000000000000000e-04;

sampleTime.CurrentControl = 4.000000000000000e-06;
sampleTime.VelocityControl = 1.000000000000000e-03;





