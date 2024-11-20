PID_speed.p = 0.5;
PID_speed.i = 10;

PID_current.p = 0.01;
PID_current.i = 100;


BLDC.RatedVoltage = 48;
BLDC.RatedSpeed = 4390 / 30 * pi;
BLDC.CurrentMax = 10;
BLDC.VoltageMax = BLDC.RatedVoltage;
BLDC.PolePairs = 7;
BLDC.StatorPhaseResistance = 0.994;
BLDC.InductanceLd = 0.995e-3;
BLDC.InductanceLq = 0.995e-3;
BLDC.InductanceL0 = 0.995e-3;
BLDC.FluxLinkage = 48/BLDC.RatedSpeed/10;
BLDC.Inertia = 44e-07;
BLDC.BreakawayFrictionTorque = 0.002130661000000 *1e-6; %??
BLDC.CoulombFrictionTorque = 0.002130661000000 *1e-6; %??
BLDC.ViscousFrictionCoefficient = 0.0083;
BLDC.RotorPositionInit = 0;
BLDC.RotorVelocityInit = 0;

inverter.BusVoltage = BLDC.RatedVoltage;
inverter.CurrentMax = 2;
inverter.GatesActiveHigh = true;
inverter.GatesComplementary = true;

paramPwmCompareAtZeroVolt = 330;
paramPwmCompareMax = 660;
paramPwmComparePerVolt = paramPwmCompareMax/2/BLDC.RatedVoltage;

disc.Inertia = 9.500000000000000e-09;

pwm.CounterMax = 660;
pwm.CountPerPeriod = 1320;
pwm.TimePerCount = 3.030303030303031e-08;
pwm.Period = 4.000000000000000e-04;

sampleTime.CurrentControl = 4.000000000000000e-06;
sampleTime.VelocityControl = 1.000000000000000e-03;





