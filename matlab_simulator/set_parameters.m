load('SystemInputs.mat')
PID_speed.p = 0.5;
PID_speed.i = 10;

PID_current.p = 0.01;
PID_current.i = 100;


BLDC.RatedVoltage = 48;
BLDC.RatedSpeed = 5320/30*pi;
BLDC.CurrentMax = 1.09 * 3;
BLDC.VoltageMax = BLDC.RatedVoltage;
BLDC.PolePairs = 4;
BLDC.StatorPhaseResistance = 2.065;
BLDC.InductanceLd = 1.44e-3;
BLDC.InductanceLq = 1.44e-3;
BLDC.InductanceL0 = 1.44e-3;
BLDC.FluxLinkage = 0.0119333333333333;
BLDC.Inertia = 4.97e-07;
BLDC.BreakawayFrictionTorque = 0.002130661000000 *1e-6; %??
BLDC.CoulombFrictionTorque = 0.002130661000000 *1e-6; %??
BLDC.ViscousFrictionCoefficient = 0.001;
BLDC.RotorPositionInit = 0;
BLDC.RotorVelocityInit = 0;

inverter.BusVoltage = BLDC.RatedVoltage;
inverter.CurrentMax = 2;
inverter.GatesActiveHigh = true;
inverter.GatesComplementary = true;

paramPwmCompareAtZeroVolt = 330;
paramPwmCompareMax = 660;
paramPwmComparePerVolt = paramPwmCompareMax/2/BLDC.RatedVoltage;

disc.Inertia = 9.500000000000000e-05;

pwm.CounterMax = 660;
pwm.CountPerPeriod = 1320;
pwm.TimePerCount = 3.030303030303031e-08;
pwm.Period = 4.000000000000000e-04;

sampleTime.CurrentControl = 4.000000000000000e-06;
sampleTime.VelocityControl = 1.000000000000000e-03;





