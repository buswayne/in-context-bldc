

global BLDC
global inverter
global paramPwmCompareAtZeroVolt
global paramPwmCompareMax
global paramPwmComparePerVolt
global disc
global pwm
global sampleTime
global PID_current
global PID_speed

PID_speed.p = 11.84;
PID_speed.i = 0.0061;

PID_current.p = 6;
PID_current.i = 150;

BLDC.RatedVoltage = 48;
BLDC.RatedSpeed = 4390 / 30 * pi;
BLDC.CurrentMax = 10;
BLDC.VoltageMax = BLDC.RatedVoltage;
BLDC.PolePairs = 7;
BLDC.StatorPhaseResistance = 0.994;
BLDC.InductanceLd = 0.995e-3;
BLDC.InductanceLq = 0.995e-3;
BLDC.InductanceL0 = 0.995e-3;
BLDC.FluxLinkage = 48/BLDC.RatedSpeed/(BLDC.PolePairs);
BLDC.Inertia = 44e-07;
BLDC.BreakawayFrictionTorque = 0.002130661000000 *1e-3; %??
BLDC.CoulombFrictionTorque = 0.002130661000000 *1e-3; %??
BLDC.ViscousFrictionCoefficient = 0.0083;
BLDC.RotorPositionInit = 0;
BLDC.RotorVelocityInit = 0;

inverter.BusVoltage = BLDC.RatedVoltage;
inverter.CurrentMax = BLDC.CurrentMax;
inverter.GatesActiveHigh = true;
inverter.GatesComplementary = true;


paramPwmCompareAtZeroVolt = 330;
paramPwmCompareMax = 660;
paramPwmComparePerVolt = paramPwmCompareMax/2/BLDC.RatedVoltage;

disc.Inertia =  0.7497e-04;

pwm.CounterMax = 660;
pwm.CountPerPeriod = 1320;
pwm.TimePerCount = 3.030303030303031e-08;
pwm.Period = 4.000000000000000e-03;

sampleTime.CurrentControl = 4.000000000000000e-06;
sampleTime.VelocityControl = 1.000000000000000e-03;





